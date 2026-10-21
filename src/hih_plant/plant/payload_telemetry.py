"""Output-only payload ground truth from the existing cooperative MuJoCo world.

This helper is owned by CooperationPlant; it is not a second node or simulator.
It creates publishers and a timer only. It never subscribes to commands, writes
MjData/model parameters, steps physics, or changes a controller reference.
"""
import math

import mujoco
import numpy as np
from geometry_msgs.msg import AccelStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor


# Visualization defaults are defined here; ROS parameters are startup-only.
DEFAULT_ENABLED = True
DEFAULT_RATE_HZ = 30.0
ODOMETRY_TOPIC = '/hih/ground_truth/payload/odometry'
ACCELERATION_TOPIC = '/hih/ground_truth/payload/acceleration'
WORLD_FRAME = 'world'
PAYLOAD_FRAME = 'hih_gt/payload'


def read_payload_state(model, data, body_id):
    """Copy one sample while the caller holds the original plant's lock.

    Position is the payload Body origin (xpos), not its CoM (xipos).
    q_WP rotates Body vectors into World and uses ROS xyzw order.
    R_WP is the corresponding rotation matrix.
    velocity_world=[omega_W,v_W] is evaluated at the Body origin using
    mjOBJ_XBODY and world components (flg_local=0). This is read-only.
    """
    velocity_world = np.zeros(6)
    mujoco.mj_objectVelocity(
        model, data, mujoco.mjtObj.mjOBJ_XBODY, body_id, velocity_world, 0)
    return (float(data.time), np.asarray(data.xpos[body_id]).copy(),
            np.asarray(data.xquat[body_id])[[1, 2, 3, 0]].copy(),
            np.asarray(data.xmat[body_id]).reshape(3, 3).copy(), velocity_world)


def _vector(message, values):
    message.x, message.y, message.z = map(float, values)


class PayloadTelemetry:
    """Publish the owning plant's payload state, outside its physics loop."""

    def __init__(self, node):
        self.node = node
        self.previous = None
        descriptor = ParameterDescriptor(read_only=True)
        self.enabled = bool(node.declare_parameter(
            'enable_payload_telemetry', DEFAULT_ENABLED, descriptor).value)
        self.rate_hz = float(node.declare_parameter(
            'payload_telemetry_rate_hz', DEFAULT_RATE_HZ, descriptor).value)
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError('payload_telemetry_rate_hz must be finite and positive')
        self.timer = self.odometry_pub = self.acceleration_pub = None
        if not self.enabled:
            return
        self.odometry_pub = node.create_publisher(Odometry, ODOMETRY_TOPIC, 2)
        self.acceleration_pub = node.create_publisher(
            AccelStamped, ACCELERATION_TOPIC, 2)
        self.timer = node.create_timer(1.0 / self.rate_hz, self.publish)
        node.get_logger().info(
            f'Output-only payload GT at {self.rate_hz:g} Hz: '
            f'{ODOMETRY_TOPIC}, {ACCELERATION_TOPIC}')

    def publish(self):
        """Read under the existing lock; encode/publish after releasing it."""
        with self.node.lock:
            sample = read_payload_state(
                self.node.model, self.node.data, self.node.payload_body)
        sim_time, position, quaternion, rotation, velocity_world = sample
        if (not math.isfinite(sim_time)
                or not all(np.isfinite(v).all() for v in sample[1:])
                or np.linalg.norm(quaternion) < 1e-12):
            self.previous = None
            return
        stamp = self.node.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = WORLD_FRAME
        odometry.child_frame_id = PAYLOAD_FRAME
        _vector(odometry.pose.pose.position, position)
        # MuJoCo xquat is wxyz; read_payload_state already copied it as xyzw.
        q = odometry.pose.pose.orientation
        q.x, q.y, q.z, q.w = map(float, quaternion)
        # ROS Odometry twist is expressed in child_frame_id (payload Body).
        # v_P = R_WP^T v_W, omega_P = R_WP^T omega_W.
        _vector(odometry.twist.twist.linear, rotation.T @ velocity_world[3:])
        _vector(odometry.twist.twist.angular, rotation.T @ velocity_world[:3])
        self.odometry_pub.publish(odometry)

        # Differentiate World velocity using simulation time for display.
        # Differentiating Body components would include rotating-axis artifacts.
        # This is linear acceleration, not IMU specific force; do not add gravity.
        previous = self.previous
        self.previous = (sim_time, velocity_world)
        if previous is None:
            return
        dt = sim_time - previous[0]
        if not 1e-8 < dt <= 1.0:
            # Skip identical time, resets or long gaps; pose remains available.
            return
        acceleration_world = (velocity_world - previous[1]) / dt
        acceleration = AccelStamped()
        acceleration.header.stamp = stamp
        acceleration.header.frame_id = WORLD_FRAME
        _vector(acceleration.accel.linear, acceleration_world[3:])
        _vector(acceleration.accel.angular, acceleration_world[:3])
        self.acceleration_pub.publish(acceleration)
