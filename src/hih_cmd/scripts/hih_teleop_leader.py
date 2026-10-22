#!/usr/bin/env python3
"""Leader-only keyboard teleoperation for the cooperative HIH scene.

This is the Leader-only counterpart of hih_teleop.py.  It keeps exactly the
same Leader key mapping:

  j/l : +body-x / -body-x
  i/k : -body-y / +body-y
  y/h : +world-z / -world-z
  u/o : +yaw / -yaw

Horizontal body-frame commands are rotated by the Leader's measured yaw before
being integrated into world-frame position references.
"""
import math
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from hih_interfaces.msg import AttitudeCmd, Cmd, MinitroneState


# Same Leader mapping used by scripts/hih_teleop.py.
KEYS = {
    'j': ('x', 1),
    'l': ('x', -1),
    'i': ('y', -1),
    'k': ('y', 1),
    'y': ('z', 1),
    'h': ('z', -1),
    'u': ('yaw', 1),
    'o': ('yaw', -1),
}


def body_xy_to_world(body_x, body_y, yaw_rad):
    """Rotate horizontal body-frame motion into MuJoCo world X/Y."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return c * body_x - s * body_y, s * body_x + c * body_y


class LeaderTarget:
    def __init__(self):
        self.position = [0.0, 0.0, 0.0]
        self.yaw_deg = 0.0
        self.measured_yaw_rad = 0.0
        self.measured_position = None
        self.initialized = False
        self.motion = None
        self.last_key = 0.0

    def update_state(self, msg):
        self.measured_position = [float(value) for value in msg.pos]
        self.measured_yaw_rad = float(msg.rpy[2])

        if not self.initialized:
            # Start from the actual measured Leader pose so starting teleop never
            # causes a jump to a hard-coded fallback reference.
            self.position = self.measured_position.copy()
            self.yaw_deg = math.degrees(self.measured_yaw_rad)
            self.initialized = True

    def recapture_measured_pose(self):
        if self.measured_position is not None:
            self.position = self.measured_position.copy()
            self.yaw_deg = math.degrees(self.measured_yaw_rad)
        self.motion = None


class HIHTeleopLeader(Node):
    def __init__(self):
        super().__init__('hih_teleop_leader')

        # Keep the same defaults as the original two-drone hih_teleop.
        self.speed_xy = float(self.declare_parameter('speed_xy', 0.1).value)
        self.speed_z = float(self.declare_parameter('speed_z', 0.05).value)
        self.speed_yaw = float(
            self.declare_parameter('speed_yaw_deg', 10.0).value)
        self.key_timeout = float(
            self.declare_parameter('key_timeout', 0.25).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)

        if min(
            self.speed_xy,
            self.speed_z,
            self.speed_yaw,
            self.key_timeout,
            self.rate_hz,
        ) <= 0:
            raise ValueError(
                'teleop speed, timeout and rate parameters must be positive')

        self.target = LeaderTarget()

        # Leader-only ROS interfaces.  Absolute topic names make this node
        # independent of the namespace from which ros2 run is invoked.
        self.pos_pub = self.create_publisher(
            Cmd, '/leader/minitrone/cmd', 10)
        self.att_pub = self.create_publisher(
            AttitudeCmd, '/leader/minitrone/att_cmd', 10)
        self.state_sub = self.create_subscription(
            MinitroneState,
            '/leader/minitrone/state',
            self.on_state,
            10,
        )

        # Respect the same trajectory authority heartbeat as hih_teleop so this
        # standalone Leader teleop does not fight hih_trajectory.
        self.trajectory_active = False
        self.trajectory_heartbeat = 0.0
        self.trajectory_sub = self.create_subscription(
            Bool,
            '/hih/trajectory_active',
            self.on_trajectory,
            10,
        )

        self.last_tick = time.monotonic()
        self.fd = sys.stdin.fileno()
        self.old_terminal = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(
            'Leader-only teleop: '
            'j/l +x/-x, i/k -y/+y, y/h +z/-z, '
            'u/o +yaw/-yaw. Space: stop; Ctrl+C: quit.')

    def destroy_node(self):
        termios.tcsetattr(
            self.fd, termios.TCSADRAIN, self.old_terminal)
        return super().destroy_node()

    def on_state(self, msg):
        self.target.update_state(msg)

    def on_trajectory(self, msg):
        self.trajectory_heartbeat = time.monotonic()
        active = bool(msg.data)

        # When trajectory authority is released, restart teleop exactly from
        # the current measured Leader pose instead of an old stored reference.
        if self.trajectory_active and not active:
            self.target.recapture_measured_pose()

        self.trajectory_active = active

    def read_keys(self):
        # Drain buffered key repeats without blocking ROS callbacks.
        for _ in range(16):
            if not select.select([sys.stdin], [], [], 0)[0]:
                break

            key = sys.stdin.read(1).lower()

            if key == ' ':
                self.target.motion = None
            elif key in KEYS:
                axis, sign = KEYS[key]
                self.target.motion = (axis, sign)
                self.target.last_key = time.monotonic()

    def publish_reference(self):
        cmd = Cmd()
        cmd.pos_cmd = [float(value) for value in self.target.position]

        attitude = AttitudeCmd()
        attitude.roll_ref = 0.0
        attitude.pitch_ref = 0.0
        attitude.yaw_ref = float(self.target.yaw_deg)

        self.pos_pub.publish(cmd)
        self.att_pub.publish(attitude)

    def tick(self):
        now = time.monotonic()
        dt = max(0.0, min(now - self.last_tick, 0.1))
        self.last_tick = now

        # Match hih_teleop's authority handling.  If the trajectory process
        # disappears without publishing false, recover after heartbeat timeout.
        if self.trajectory_active and now - self.trajectory_heartbeat > 0.6:
            self.trajectory_active = False
            self.target.recapture_measured_pose()
            self.get_logger().warn(
                'Trajectory heartbeat expired; Leader measured pose recaptured')

        if self.trajectory_active:
            # Remove queued keyboard repeats so they are not executed suddenly
            # after trajectory control returns to teleop.
            while select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.read(1)
            return

        self.read_keys()

        # Do not publish until the Leader's actual initial state is known.
        if not self.target.initialized:
            return

        if now - self.target.last_key > self.key_timeout:
            self.target.motion = None

        if self.target.motion:
            axis, sign = self.target.motion

            if axis in ('x', 'y'):
                body_x = sign * self.speed_xy if axis == 'x' else 0.0
                body_y = sign * self.speed_xy if axis == 'y' else 0.0
                dx, dy = body_xy_to_world(
                    body_x, body_y, self.target.measured_yaw_rad)
                self.target.position[0] += dx * dt
                self.target.position[1] += dy * dt

            elif axis == 'z':
                self.target.position[2] += sign * self.speed_z * dt

            elif axis == 'yaw':
                self.target.yaw_deg += sign * self.speed_yaw * dt

        self.publish_reference()


def main():
    if not sys.stdin.isatty():
        raise RuntimeError(
            'Run hih_teleop_leader in a focused interactive terminal')

    rclpy.init()
    node = None
    try:
        node = HIHTeleopLeader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
