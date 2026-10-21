"""One MuJoCo world, two isolated ROS control interfaces.

The plant alone owns both bodies for physical contact. Neither controller sees
the other vehicle's state, command, wrench estimate or actuator input.
"""
import math
import os
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from hih_interfaces.msg import (ContactDiagnostics, CenterOfPressure,
                                 Input, MinitroneState, MobObserverInput, Wrench)
from plant.contact_geometry import MAX_CONTACTS, plate_support_cop_from_wrench
from plant.high_fidelity import HighFidelityModel, PARAMETERS
from plant.disturbance_bridge import SimulationDisturbances
from plant.payload_telemetry import PayloadTelemetry
from plant.physics import PHYSICS_HZ, quat_to_rpy


def model_id(model, kind, name):
    index = mujoco.mj_name2id(model, kind, name)
    if index < 0:
        raise ValueError(f'MJCF is missing {kind.name}: {name}')
    return int(index)


class Vehicle:
    """All simulation state and ROS endpoints belong to exactly one vehicle."""

    def __init__(self, node, label, model, data, config, seed):
        self.label, self.model, self.data = label, model, data
        self.hf = HighFidelityModel(config, seed, 1 / PHYSICS_HZ)
        prefix = label + '_'
        aid = lambda name: model_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, prefix + name)
        sid = lambda name: model_id(model, mujoco.mjtObj.mjOBJ_SENSOR, prefix + name)
        bid = lambda name: model_id(model, mujoco.mjtObj.mjOBJ_BODY, prefix + name)
        gid = lambda name: model_id(model, mujoco.mjtObj.mjOBJ_GEOM, prefix + name)
        site = lambda name: model_id(model, mujoco.mjtObj.mjOBJ_SITE, prefix + name)
        self.base = bid('drone_base')
        self.props = [aid(f'BLDC{i}') for i in range(1, 5)]
        self.servos = [aid(f'servo{i}_pos') for i in range(1, 5)]
        self.prop_sites = [site(f'prop{i}_site') for i in range(1, 5)]
        self.quat = sid('body_quat')
        self.gyro = sid('body_gyro')
        self.position = sid('base_pos')
        self.velocity = sid('base_linvel')
        self.servo_sensors = [sid(f'servo{i}_angle') for i in range(1, 5)]
        self.corner_sensors = [sid(f'cop_force_{corner}')
                               for corner in ('pp', 'np', 'nn', 'pn')]
        self.plate = gid('contact_plate_px')
        self.payload_geom = model_id(
            model, mujoco.mjtObj.mjOBJ_GEOM, 'payload_contact_box_collision')

        # Payload-side virtual 6-axis F/T sensor.  The XML site sits at the
        # center of this drone's payload contact face.  Its +X axis is defined
        # as the inward payload normal, so +Fx is compression for BOTH roles.
        self.payload_sensor_site = model_id(
            model, mujoco.mjtObj.mjOBJ_SITE,
            f'payload_contact_sensor_{label}')
        # The payload is shared physics; sampling its uncertainty once per
        # vehicle would compound it and couple their random parameter draws.
        self.hf.configure_physics(model, data, self.base,
                                  [self.plate, gid('contact_plate_nx')])
        self.thrust_limits = model.actuator_ctrlrange[self.props, 1].copy()
        if not np.all(model.actuator_ctrllimited[self.props]):
            raise ValueError(f'{label}: XML motor limits are required')
        for j, actuator in enumerate(self.props):
            if model.actuator_forcelimited[actuator]:
                self.thrust_limits[j] = min(
                    self.thrust_limits[j], model.actuator_forcerange[actuator, 1])
        self.command = np.zeros(8)
        self.command_received = False
        self.force_body = np.zeros(3)
        self.moment_body = np.zeros(3)
        self.last_wrench_time = None
        self.prev_vel = self.prev_gyro = None
        self.frame = f'/{label}/minitrone/'
        self.enable_cop_touch = node.enable_cop_touch
        self.enable_cop_solver = node.enable_cop_solver
        self.cop_solver_valid_threshold_n = node.cop_solver_valid_threshold_n
        self.input_sub = node.create_subscription(
            Input, self.frame + 'input', self.on_input, 10)
        self.wrench_sub = node.create_subscription(
            Wrench, self.frame + 'external_wrench_cmd', self.on_wrench, 10)
        self.state_pub = node.create_publisher(
            MinitroneState, self.frame + 'state', 10)
        self.observer_pub = node.create_publisher(
            MobObserverInput, self.frame + 'mob_observer_input', 10)
        self.actuation_pub = node.create_publisher(
            Wrench, self.frame + 'actuation_wrench_body', 10)
        self.cop_pub = (node.create_publisher(
            CenterOfPressure, self.frame + 'cop_real', 10)
            if self.enable_cop_touch else None)
        self.cop_solver_pub = (node.create_publisher(
            CenterOfPressure, self.frame + 'cop_solver_real', 10)
            if self.enable_cop_solver else None)
        # Ground-truth plate/payload contact telemetry from MuJoCo's contact
        # solver. The wrench is payload-on-drone, expressed in the drone Body
        # axes; the +X plate normal points from drone toward payload, so
        # compression is Fn=max(0,-Fx).
        self.contact_wrench_pub = node.create_publisher(
            Wrench, self.frame + 'contact_wrench_real', 10)
        self.contact_points_pub = node.create_publisher(
            ContactDiagnostics, self.frame + 'contact_diagnostics', 10)

        # Payload-side 6D wrench sensor topic.  Unlike contact_wrench_real
        # (payload-on-drone, drone Body axes, plate-center moment), this is
        # DRONE-ON-PAYLOAD at the payload face-center sensor frame.
        #
        # Topic:
        #   /payload/contact_wrench_from_leader
        #   /payload/contact_wrench_from_follower
        #
        # Convention:
        #   force[0]  = + normal compression [N] on both faces
        #   force[1:3] = tangential components in the face sensor frame
        #   moment[0:3] = moment about the payload face-center sensor [N*m]
        self.payload_contact_wrench_pub = node.create_publisher(
            Wrench, f'/payload/contact_wrench_from_{label}', 10)
        self.last_payload_com_force = np.zeros(3)
        self.last_payload_com_moment = np.zeros(3)

    def on_input(self, msg):
        self.command = np.asarray(msg.u, dtype=float).copy()
        self.command_received = True

    def on_wrench(self, msg):
        self.force_body = np.asarray(msg.force, dtype=float).copy()
        self.moment_body = np.asarray(msg.moment, dtype=float).copy()
        self.last_wrench_time = time.monotonic()

    def read(self, sensor):
        addr = self.model.sensor_adr[sensor]
        dim = self.model.sensor_dim[sensor]
        return np.asarray(self.data.sensordata[addr:addr + dim]).copy()

    def apply(self):
        thrust, angles = self.hf.actuate(
            float(self.data.time), self.command, self.thrust_limits)
        self.data.ctrl[self.props] = thrust
        self.data.ctrl[self.servos] = angles
        if self.last_wrench_time is not None and time.monotonic() - self.last_wrench_time > 0.2:
            self.force_body[:] = self.moment_body[:] = 0.0
        rotation = np.asarray(self.data.xmat[self.base]).reshape(3, 3)
        residual = self.hf.external_residual()
        # xfrc_applied is in world axes, applied at the actual body CoM.
        self.data.xfrc_applied[self.base, :3] = rotation @ (self.force_body + residual[:3])
        self.data.xfrc_applied[self.base, 3:] = rotation @ (self.moment_body + residual[3:])

    def motor_wrench(self):
        rotation = np.asarray(self.data.xmat[self.base]).reshape(3, 3)
        origin = np.asarray(self.data.xpos[self.base])
        force = np.zeros(3)
        moment = np.zeros(3)
        for actuator, site in zip(self.props, self.prop_sites):
            gain = float(self.data.actuator_force[actuator])
            gear = np.asarray(self.model.actuator_gear[actuator])
            site_rotation = np.asarray(self.data.site_xmat[site]).reshape(3, 3)
            local_force = site_rotation @ (gear[:3] * gain)
            force += local_force
            moment += (np.cross(self.data.site_xpos[site] - origin, local_force)
                       + site_rotation @ (gear[3:6] * gain))
        return rotation.T @ force, rotation.T @ moment

    def plate_payload_contact_sample(self):
        """Read interface wrenches, solver CoP and RViz contact points once.

        All quantities are observations of the existing MuJoCo world. Moments
        are shifted to their published reference points; physics is not changed.
        ContactDiagnostics keeps its existing ROS schema for HandinHand_rviz.
        Unused friction/slip fields are NaN rather than synthetic measurements.
        """
        force_world, moment_world = np.zeros(3), np.zeros(3)
        sensor_force_world, sensor_moment_world = np.zeros(3), np.zeros(3)
        com_force_world, com_moment_world = np.zeros(3), np.zeros(3)
        plate_center = np.asarray(self.data.geom_xpos[self.plate])
        payload_body = int(self.model.geom_bodyid[self.payload_geom])
        payload_com = np.asarray(self.data.xipos[payload_body])
        sensor_center = np.asarray(self.data.site_xpos[self.payload_sensor_site])
        rotation = np.asarray(self.data.xmat[self.base]).reshape(3, 3)
        payload_rotation = np.asarray(self.data.geom_xmat[self.payload_geom]).reshape(3, 3)
        sensor_rotation = np.asarray(self.data.site_xmat[self.payload_sensor_site]).reshape(3, 3)

        points = ContactDiagnostics()
        points.step = int(round(float(self.data.time) * PHYSICS_HZ))
        points.sim_time = float(self.data.time)
        for field in ('contact_pos_x', 'contact_pos_y', 'contact_pos_z',
                      'contact_dist', 'normal_relative_velocity', 'normal_force',
                      'tangential_relative_speed', 'local_friction_utilization',
                      'force_x', 'force_y', 'force_z'):
            setattr(points, field, [math.nan] * MAX_CONTACTS)
        points.efc_address = [-1] * MAX_CONTACTS
        for field in ('global_friction_utilization', 'max_local_friction_utilization',
                      'max_tangential_relative_speed', 'normal_force_weighted_tangential_speed',
                      'normal_moment_y', 'normal_moment_y_capacity', 'normal_moment_y_utilization',
                      'normal_moment_z', 'normal_moment_z_capacity', 'normal_moment_z_utilization'):
            setattr(points, field, math.nan)
        points.corner_face_gap = [math.nan] * 4
        for field in ('relative_position', 'relative_rpy', 'relative_linear_velocity',
                      'relative_angular_velocity'):
            setattr(points, field, [math.nan] * 3)
        distances = []
        for index in range(int(self.data.ncon)):
            contact = self.data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if {geom1, geom2} != {self.plate, self.payload_geom}:
                continue
            slot = points.contact_count
            points.contact_count += 1
            distances.append(float(contact.dist))
            active = int(contact.efc_address) >= 0
            wrench = np.zeros(6)
            if active:
                points.active_constraint_count += 1
                mujoco.mj_contactForce(self.model, self.data, index, wrench)
            frame = np.asarray(contact.frame).reshape(3, 3)
            sign = 1.0 if geom2 == self.plate else -1.0
            force = sign * (frame.T @ wrench[:3])
            torque = sign * (frame.T @ wrench[3:])
            position = np.asarray(contact.pos)
            force_world += force
            moment_world += torque + np.cross(position - plate_center, force)
            sensor_force_world -= force
            sensor_moment_world -= torque + np.cross(position - sensor_center, force)
            com_force_world -= force
            com_moment_world -= torque + np.cross(position - payload_com, force)
            fn = max(0.0, float(wrench[0])) if active else 0.0
            points.total_normal_force += fn
            if slot < MAX_CONTACTS:
                points.stored_contact_count += 1
                local = rotation.T @ (position - plate_center)
                local_force = rotation.T @ force
                for axis, value in zip('xyz', local):
                    getattr(points, 'contact_pos_' + axis)[slot] = float(value)
                for axis, value in zip('xyz', local_force):
                    getattr(points, 'force_' + axis)[slot] = float(value)
                points.contact_dist[slot] = float(contact.dist)
                points.normal_force[slot] = fn
                points.efc_address[slot] = int(contact.efc_address)
        points.contact_truncated = points.contact_count > MAX_CONTACTS
        points.min_contact_dist = min(distances) if distances else math.nan
        points.max_contact_dist = max(distances) if distances else math.nan
        force_body, moment_body = rotation.T @ force_world, rotation.T @ moment_world
        points.net_force, points.net_moment = force_body.tolist(), moment_body.tolist()
        half_x, half_y, half_z = map(float, self.model.geom_size[self.plate])
        moment_face = moment_body - np.cross([half_x, 0.0, 0.0], force_body)
        support = plate_support_cop_from_wrench(
            force_body, moment_face, half_y, half_z, self.cop_solver_valid_threshold_n)
        points.solver_cop_valid = support['valid']
        points.solver_cop_y, points.solver_cop_z = support['y'], support['z']
        cop = None
        if self.enable_cop_solver:
            cop = CenterOfPressure()
            cop.normal_force, cop.valid = support['normal_force'], support['valid']
            cop.y, cop.z = support['y'], support['z']
            cop.corner_forces = [0.0] * 4
        return (force_body, moment_body,
                sensor_rotation.T @ sensor_force_world, sensor_rotation.T @ sensor_moment_world,
                payload_rotation.T @ com_force_world, payload_rotation.T @ com_moment_world,
                points, cop)

    def publish(self):
        t = float(self.data.time)
        dt = 1 / PHYSICS_HZ
        true_pos, true_vel = self.read(self.position), self.read(self.velocity)
        true_gyro = self.read(self.gyro)
        true_rpy = quat_to_rpy(self.read(self.quat))
        true_servo = np.array([self.read(s)[0] for s in self.servo_sensors])
        rotation = np.asarray(self.data.xmat[self.base]).reshape(3, 3)
        acc_world = np.zeros(3) if self.prev_vel is None else (true_vel - self.prev_vel) / dt
        alpha = np.zeros(3) if self.prev_gyro is None else (true_gyro - self.prev_gyro) / dt
        self.prev_vel, self.prev_gyro = true_vel.copy(), true_gyro.copy()
        if self.hf.enabled('sensor_model'):
            sensed = self.hf.measure(t, dict(
                position=true_pos, velocity=true_vel, attitude=true_rpy,
                gyro=true_gyro, acceleration=rotation.T @
                (acc_world - self.model.opt.gravity),
                angular_acceleration=alpha, servo=true_servo))
        else:
            # The legacy single-vehicle plant publishes world acceleration.
            sensed = dict(position=true_pos, velocity=true_vel,
                          attitude=true_rpy, gyro=true_gyro,
                          acceleration=acc_world, angular_acceleration=alpha,
                          servo=true_servo)
        state = MinitroneState()
        state.pos = sensed['position'].tolist()
        state.vel = sensed['velocity'].tolist()
        state.acc = sensed['acceleration'].tolist()
        state.rpy = sensed['attitude'].tolist()
        state.w_rpy = sensed['gyro'].tolist()
        state.a_rpy = sensed['angular_acceleration'].tolist()
        state.servo = np.rad2deg(sensed['servo']).tolist()
        self.state_pub.publish(state)

        force, moment = self.motor_wrench()
        observed = MobObserverInput()
        observed.step = int(round(t * PHYSICS_HZ))
        observed.sim_time = t
        observed.pos, observed.vel = state.pos, state.vel
        observed.rpy, observed.w_rpy = state.rpy, state.w_rpy
        observed.actuation_force = force.astype(np.float32).tolist()
        observed.actuation_moment = moment.astype(np.float32).tolist()
        self.observer_pub.publish(observed)
        actuation = Wrench()
        actuation.force, actuation.moment = observed.actuation_force, observed.actuation_moment
        self.actuation_pub.publish(actuation)

        if self.enable_cop_touch:
            # Corner touch sensors are scoped to this drone's inward +X plate.
            loads = np.maximum(0.0, [self.read(s)[0] for s in self.corner_sensors])
            total = float(np.sum(loads))
            cop = CenterOfPressure()
            cop.valid = total >= 0.5
            cop.normal_force = total
            cop.corner_forces = loads.tolist()
            cop.y = float(0.2 * (loads[0] - loads[1] - loads[2] + loads[3]) / total) if cop.valid else 0.0
            cop.z = float(0.19 * (loads[0] + loads[1] - loads[2] - loads[3]) / total) if cop.valid else 0.0
            self.cop_pub.publish(cop)

        # Interface telemetry and RViz overlays share one solver-contact sample.
        (contact_force, contact_moment, payload_sensor_force, payload_sensor_moment,
         payload_com_force, payload_com_moment, contact_points,
         solver_cop) = self.plate_payload_contact_sample()
        contact_wrench = Wrench()
        contact_wrench.force = contact_force.astype(np.float32).tolist()
        contact_wrench.moment = contact_moment.astype(np.float32).tolist()
        self.contact_wrench_pub.publish(contact_wrench)
        self.contact_points_pub.publish(contact_points)

        payload_wrench = Wrench()
        payload_wrench.force = payload_sensor_force.astype(np.float32).tolist()
        payload_wrench.moment = payload_sensor_moment.astype(np.float32).tolist()
        self.payload_contact_wrench_pub.publish(payload_wrench)

        # Store the same interface wrench at payload CoM for the plant-level
        # Leader+Follower sum published once after both vehicle samples.
        self.last_payload_com_force = payload_com_force.copy()
        self.last_payload_com_moment = payload_com_moment.copy()

        if solver_cop is not None:
            self.cop_solver_pub.publish(solver_cop)


class CooperationPlant(Node):
    def __init__(self):
        super().__init__('cooperation_plant')
        self.enable_viewer = self.declare_parameter('enable_viewer', True).value
        # Output switches affect CoP telemetry only, not contact physics.
        self.enable_cop_touch = bool(self.declare_parameter('enable_cop_touch', True).value)
        self.enable_cop_solver = bool(self.declare_parameter('enable_cop_solver', True).value)
        self.cop_solver_valid_threshold_n = float(self.declare_parameter(
            'cop_solver_valid_threshold_n', 0.5).value)
        if not math.isfinite(self.cop_solver_valid_threshold_n) or self.cop_solver_valid_threshold_n <= 0:
            raise ValueError('cop_solver_valid_threshold_n must be finite and positive')
        seed = int(self.declare_parameter('random_seed', 1).value)
        config = {key: self.declare_parameter(
            key, definition[0], ParameterDescriptor(read_only=True)).value
            for key, definition in PARAMETERS.items()}
        xml = os.path.join(get_package_share_directory('hih_plant'),
                           'xml', 'Cooperation.xml')
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.model.opt.timestep = 1 / PHYSICS_HZ
        self.data = mujoco.MjData(self.model)
        self.lock = threading.Lock()
        self.vehicles = [Vehicle(self, label, self.model, self.data, config, seed + n)
                         for n, label in enumerate(('leader', 'follower'))]

        # Net wrench from BOTH drones on the payload, about payload CoM and
        # expressed in payload body axes.  This excludes gravity and pedestal
        # contact: it is only the sum of the two drone/payload interfaces.
        self.payload_contact_wrench_total_pub = self.create_publisher(
            Wrench, '/payload/contact_wrench_from_drones', 10)
        self.stop = False
        self.track_assembly = False
        self.payload_body = model_id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'payload_contact_box')
        mujoco.mj_forward(self.model, self.data)
        # Read-only payload telemetry from this same world, for RViz.
        self.payload_telemetry = PayloadTelemetry(self)
        self.disturbances = SimulationDisturbances(self)
        self.thread = threading.Thread(target=self.simulate, daemon=True)
        self.thread.start()
        if self.enable_viewer:
            self.viewer_thread = threading.Thread(target=self.view, daemon=True)
            self.viewer_thread.start()
        self.get_logger().info(
            'Payload 6D F/T sensors: /payload/contact_wrench_from_leader, '
            '/payload/contact_wrench_from_follower; +Fx=compression. '
            'Net drone contact wrench at payload CoM: '
            '/payload/contact_wrench_from_drones.')
        self.get_logger().info('Leader and Follower have separate ROS interfaces; only MuJoCo contact couples them.')

    def publish_payload_contact_wrench_total(self):
        """Publish the net drone contact wrench at payload CoM in payload axes."""
        force = sum(
            (vehicle.last_payload_com_force for vehicle in self.vehicles),
            np.zeros(3))
        moment = sum(
            (vehicle.last_payload_com_moment for vehicle in self.vehicles),
            np.zeros(3))

        msg = Wrench()
        msg.force = np.asarray(force, dtype=np.float32).tolist()
        msg.moment = np.asarray(moment, dtype=np.float32).tolist()
        self.payload_contact_wrench_total_pub.publish(msg)

    def simulate(self):
        next_step = time.perf_counter()
        while rclpy.ok() and not self.stop:
            # Publish the initial state while waiting for both independent
            # controller/allocator stacks. If physics advanced before their
            # first actuator commands, the drones could fall during startup.
            # No drone state or command is forwarded to the other controller.
            if not all(vehicle.command_received for vehicle in self.vehicles):
                with self.lock:
                    for vehicle in self.vehicles:
                        vehicle.publish()
                    self.publish_payload_contact_wrench_total()
                next_step = time.perf_counter()
                time.sleep(1 / PHYSICS_HZ)
                continue
            with self.lock:
                for vehicle in self.vehicles:
                    vehicle.apply()
                # Add experiment loads after Vehicle.apply() writes its own
                # manual/HF wrench, so simultaneous sources never overwrite.
                self.disturbances.apply()
                mujoco.mj_step(self.model, self.data)
                mujoco.mj_forward(self.model, self.data)
                for vehicle in self.vehicles:
                    vehicle.publish()
                self.publish_payload_contact_wrench_total()
            next_step += 1 / PHYSICS_HZ
            # Catch-up ticks remain simulation ticks; sampling uses data.time.
            delay = next_step - time.perf_counter()
            if delay > 0:
                time.sleep(delay)

    def view(self):
        def on_key(keycode):
            if keycode == ord('8'):
                self.track_assembly = not self.track_assembly
                self.get_logger().info(
                    'Assembly camera %s' % ('on' if self.track_assembly else 'off'))

        with mujoco.viewer.launch_passive(
                self.model, self.data, key_callback=on_key) as viewer:
            # Keep MuJoCo's free camera and mouse controls available.
            with viewer.lock():
                # Include the decorative bodies and translucent propeller discs.
                viewer.opt.geomgroup[1:5] = True
                viewer.opt.sitegroup[5] = False
            while viewer.is_running() and rclpy.ok() and not self.stop:
                with self.lock:
                    if self.track_assembly:
                        centers = np.array([
                            self.data.xpos[self.vehicles[0].base],
                            self.data.xpos[self.payload_body],
                            self.data.xpos[self.vehicles[1].base]])
                        lookat = centers.mean(axis=0).copy()
                        spread = float(np.ptp(centers[:, :2], axis=0).max())
                    else:
                        lookat = None
                    if lookat is not None:
                        with viewer.lock():
                            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                            viewer.cam.lookat[:] += 0.12 * (lookat - viewer.cam.lookat)
                            viewer.cam.distance += 0.12 * (
                                max(2.7, 1.7 * spread + 1.4) - viewer.cam.distance)
                            viewer.cam.azimuth = 90.0
                            viewer.cam.elevation = -17.0
                    viewer.sync()
                time.sleep(1 / 60)

    def close(self):
        self.stop = True
        self.thread.join(timeout=2)
        if self.enable_viewer:
            self.viewer_thread.join(timeout=2)


def main():
    rclpy.init()
    node = CooperationPlant()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
