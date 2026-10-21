"""Apply experiment inputs only in the owning cooperative plant physics loop."""
import threading
import time

import mujoco
import numpy as np
from geometry_msgs.msg import PointStamped, Vector3Stamped, WrenchStamped
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, Float64

from plant.disturbance_profiles import (
    COM_COMMAND_TOPIC, COM_OFFSET_TOPIC, PAYLOAD_WRENCH_TOPIC, ROLES,
    SIM_TIME_TOPIC, WIND_ENABLE_TOPIC, WIND_TOPIC, finite_array, positive, wind_wrench,
)


def xyz(message, value):
    message.x, message.y, message.z = map(float, value)


def array(message):
    return np.array([message.x, message.y, message.z], dtype=float)


class SimulationDisturbances:
    """Callbacks store inputs; apply() runs with CooperationPlant.lock held.

    Existing Vehicle.apply() owns each drone's original external/HF wrench.
    This helper adds wind AFTER those writes and owns the payload applied row.
    CoM changes update the real inertial frame, never the collision geometry.
    """

    def __init__(self, node):
        self.node, self.model, self.data = node, node.model, node.data
        self.inbox_lock = threading.Lock()
        self.pending_com = self.payload_wrench = self.wind = None
        self.wind_enabled = None
        self.constants_data = None
        self.last_sim_time = float(self.data.time)
        self.bodies = {'payload': node.payload_body,
                       **{vehicle.label: vehicle.base for vehicle in node.vehicles}}
        self.payload_geom = self.model.geom('payload_contact_box_collision').id
        self.center = self.model.geom_pos[self.payload_geom].copy()
        self.half_size = self.model.geom_size[self.payload_geom].copy()
        self.applied = {role: np.zeros(6) for role in ROLES}
        descriptor = ParameterDescriptor(read_only=True)
        declare = lambda name, value: node.declare_parameter(name, value, descriptor).value
        self.enabled = bool(declare('enable_disturbance_inputs', True))
        self.timeout = positive(declare('disturbance_timeout_s', .3), 'disturbance_timeout_s')
        rate = positive(declare('disturbance_telemetry_rate_hz', 50.),
                        'disturbance_telemetry_rate_hz')
        self.density = positive(declare('wind_air_density_kg_m3', 1.225),
                                'wind_air_density_kg_m3')
        self.wind_properties = {}
        for role in ROLES:
            # Payload default areas come from its compiled box, not example dimensions.
            size = 2. * self.half_size
            default_area = ([size[1]*size[2], size[0]*size[2], size[0]*size[1]]
                            if role == 'payload' else [.10, .10, .16])
            area = finite_array(declare(f'wind_{role}_area_body_m2', list(default_area)),
                                3, 'wind area', True)
            coefficient = finite_array(declare(f'wind_{role}_drag_coefficient', [1., 1., 1.]),
                                       3, 'wind drag coefficient', True)
            pressure = finite_array(declare(f'wind_{role}_pressure_center_body_m', [0., 0., 0.]),
                                    3, 'wind pressure center')
            self.wind_properties[role] = (area, coefficient, pressure)
        self.subscriptions = [
            node.create_subscription(Vector3Stamped, COM_COMMAND_TOPIC, self.on_com, 2),
            node.create_subscription(WrenchStamped, PAYLOAD_WRENCH_TOPIC, self.on_wrench, 2),
            node.create_subscription(Vector3Stamped, WIND_TOPIC, self.on_wind, 2),
            node.create_subscription(Bool, WIND_ENABLE_TOPIC, self.on_wind_enabled, 2),
        ]
        self.time_pub = node.create_publisher(Float64, SIM_TIME_TOPIC, 2)
        self.offset_pub = node.create_publisher(Vector3Stamped, COM_OFFSET_TOPIC, 2)
        self.com_pub = node.create_publisher(PointStamped, '/hih/ground_truth/payload/com', 2)
        self.wrench_pubs = {role: node.create_publisher(
            WrenchStamped, f'/hih/disturbance/{role}/applied_wrench', 2) for role in ROLES}
        self.timer = node.create_timer(1. / rate, self.publish)

    def reject(self, reason):
        self.node.get_logger().warning('Disturbance input ignored: ' + reason)

    def on_com(self, msg):
        offset = array(msg.vector)
        if (msg.header.frame_id != 'payload' or not np.isfinite(offset).all()
                or abs(offset[2]) > 1e-12
                or np.any(np.abs(offset[:2]) >= self.half_size[:2])):
            self.reject('CoM requires payload-frame XY offset strictly inside the box; z=0')
            return
        with self.inbox_lock:
            self.pending_com = self.center[:2] + offset[:2]

    def on_wrench(self, msg):
        wrench = np.r_[array(msg.wrench.force), array(msg.wrench.torque)]
        if msg.header.frame_id not in ('world', 'payload') or not np.isfinite(wrench).all():
            self.reject('payload wrench requires finite values and world/payload frame')
            return
        with self.inbox_lock:
            self.payload_wrench = (wrench, msg.header.frame_id, time.monotonic())

    def on_wind(self, msg):
        velocity = array(msg.vector)
        if msg.header.frame_id != 'world' or not np.isfinite(velocity).all():
            self.reject('wind requires finite World velocity')
            return
        with self.inbox_lock:
            self.wind = (velocity, time.monotonic())

    def on_wind_enabled(self, msg):
        with self.inbox_lock:
            self.wind_enabled = (bool(msg.data), time.monotonic())

    def move_com(self, target):
        body = self.bodies['payload']
        if np.array_equal(self.model.body_ipos[body, :2], target):
            return
        self.model.body_ipos[body, :2] = target
        # MuJoCo 3.3.7 caches these compiler shortcuts. Match the existing HF
        # implementation: otherwise changing body_ipos can leave xipos unchanged.
        self.model.body_sameframe[body] = int(mujoco.mjtSameFrame.mjSAMEFRAME_NONE)
        if self.model.body_simple[body]:
            self.model.body_simple[:] = 0
            self.model.dof_simplenum[:] = 0
        if self.constants_data is None:
            self.constants_data = mujoco.MjData(self.model)
        # mj_setConst uses its MjData as scratch at qpos0. Never pass live data:
        # qpos, qvel, controls and simulation time must survive an in-flight edit.
        mujoco.mj_setConst(self.model, self.constants_data)
        mujoco.mj_forward(self.model, self.data)

    def apply(self):
        """Compose one tick after Vehicle.apply(); caller holds physics lock."""
        now = time.monotonic()
        with self.inbox_lock:
            if float(self.data.time) < self.last_sim_time:
                self.pending_com = self.payload_wrench = self.wind = None
                self.wind_enabled = None
            target, self.pending_com = self.pending_com, None
            payload_wrench, wind = self.payload_wrench, self.wind
            wind_enabled = self.wind_enabled
        self.last_sim_time = float(self.data.time)
        for role in ROLES:
            self.applied[role][:] = 0.
        self.data.xfrc_applied[self.bodies['payload']] = 0.
        if not self.enabled:
            return
        if target is not None:
            self.move_com(target)
        if payload_wrench and 0. <= now - payload_wrench[2] <= self.timeout:
            wrench, frame, _ = payload_wrench
            if frame == 'payload':
                rotation = self.data.xmat[self.bodies['payload']].reshape(3, 3)
                wrench = np.r_[rotation @ wrench[:3], rotation @ wrench[3:]]
            self.applied['payload'] += wrench
        if (wind and wind_enabled and wind_enabled[0]
                and 0. <= now - wind[1] <= self.timeout
                and 0. <= now - wind_enabled[1] <= self.timeout):
            velocity = wind[0]
            for role, body in self.bodies.items():
                rotation = self.data.xmat[body].reshape(3, 3)
                area, coefficient, pressure = self.wind_properties[role]
                motion = np.zeros(6)
                mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY,
                                         body, motion, 0)
                point_velocity = motion[3:] + np.cross(motion[:3], rotation @ pressure)
                self.applied[role] += wind_wrench(
                    velocity, point_velocity, rotation, area, coefficient, self.density,
                    pressure - self.model.body_ipos[body])
        for role, body in self.bodies.items():
            self.data.xfrc_applied[body] += self.applied[role]

    def publish(self):
        """Ground truth and applied experiment loads; no model writes here."""
        with self.node.lock:
            sim_time = float(self.data.time)
            body = self.bodies['payload']
            offset = self.model.body_ipos[body].copy() - self.center
            offset[2] = 0.  # Command/feedback concerns XY only; original COM Z is retained.
            com_world = self.data.xipos[body].copy()
            applied = {role: value.copy() for role, value in self.applied.items()}
        stamp = self.node.get_clock().now().to_msg()
        clock = Float64()
        clock.data = sim_time
        self.time_pub.publish(clock)
        msg = Vector3Stamped()
        msg.header.stamp, msg.header.frame_id = stamp, 'payload'
        xyz(msg.vector, offset)
        self.offset_pub.publish(msg)
        point = PointStamped()
        point.header.stamp, point.header.frame_id = stamp, 'world'
        xyz(point.point, com_world)
        self.com_pub.publish(point)
        for role, value in applied.items():
            msg = WrenchStamped()
            msg.header.stamp, msg.header.frame_id = stamp, 'world'
            xyz(msg.wrench.force, value[:3])
            xyz(msg.wrench.torque, value[3:])
            self.wrench_pubs[role].publish(msg)
