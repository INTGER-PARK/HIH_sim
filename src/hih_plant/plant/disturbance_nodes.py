"""Optional keyboard nodes for payload experiments and a shared wind field."""
import math
import select
import sys
import termios
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

from plant.disturbance_profiles import (
    COM_COMMAND_TOPIC, COM_OFFSET_TOPIC, COM_VISIBILITY_TOPIC, PAYLOAD_WRENCH_TOPIC, SIM_TIME_TOPIC,
    WIND_ENABLE_TOPIC, WIND_TOPIC, PayloadWrenchProfile, RectangleCOM, WindProfile,
    positive,
)


def xyz(message, value):
    message.x, message.y, message.z = map(float, value)


class TimedDisturbance(Node):
    """Wall-clock ROS heartbeat; profile motion advances only in MuJoCo time."""

    def __init__(self, name):
        super().__init__(name)
        self.quit = False
        self.sim_time = self.previous_time = self.received = None
        self.keyboard_enabled = bool(self.parameter('keyboard_enabled', True))
        self.rate = positive(self.parameter('publish_rate_hz', 50.), 'publish_rate_hz')
        self.clock_timeout = positive(self.parameter('clock_timeout_s', .5), 'clock_timeout_s')
        self.clock_sub = self.create_subscription(Float64, SIM_TIME_TOPIC, self.on_clock, 2)
        self.key_sub = self.create_subscription(String, '~/key', lambda msg: self.handle_key(msg.data), 2)

    def parameter(self, name, default):
        return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

    def on_clock(self, msg):
        if math.isfinite(msg.data) and msg.data >= 0.:
            if self.sim_time is not None and msg.data < self.sim_time:
                self.reset_experiment()
                self.previous_time = None
            self.sim_time, self.received = float(msg.data), time.monotonic()

    def elapsed(self):
        if self.received is None or time.monotonic() - self.received > self.clock_timeout:
            self.previous_time = None
            return None
        dt = 0. if self.previous_time is None else self.sim_time - self.previous_time
        self.previous_time = self.sim_time
        if dt < 0.:
            self.reset_experiment()
            return 0.
        return dt


class PayloadDisturbance(TimedDisturbance):
    """c: CCW CoM path; u/i/o/j/k/l: Fx/Fy/Fz/Tx/Ty/Tz; p: step/sine."""

    def __init__(self):
        super().__init__('payload_disturbance')
        self.com = RectangleCOM(self.parameter('com_half_size_m', [.025, .025]),
                                self.parameter('com_speed_mps', .01))
        self.wrench = PayloadWrenchProfile(
            self.parameter('wrench_amplitudes', [1., 1., 1., .05, .05, .05]),
            self.parameter('sine_frequency_hz', [.5, .5, .5, .5, .5, .5]))
        self.frame = str(self.parameter('wrench_frame', 'world'))
        if self.frame not in ('world', 'payload'):
            raise ValueError('wrench_frame must be world or payload')
        self.current_com = self.com_received = None
        self.com_owned = False
        self.com_feedback = self.create_subscription(
            Vector3Stamped, COM_OFFSET_TOPIC, self.on_com, 2)
        self.com_pub = self.create_publisher(Vector3Stamped, COM_COMMAND_TOPIC, 2)
        self.com_visibility_pub = self.create_publisher(Bool, COM_VISIBILITY_TOPIC, 2)
        self.wrench_pub = self.create_publisher(WrenchStamped, PAYLOAD_WRENCH_TOPIC, 2)
        self.timer = self.create_timer(1. / self.rate, self.tick)
        self.get_logger().info(
            'Payload: c=COM start/pause; r=COM return; U/I/O/J/K/L=Fx/Fy/Fz/Tx/Ty/Tz '
            'toggle; -=reverse last axis; p=step/sine; 0=clear wrench; '
            'SPACE=stop this node\'s experiment; q=quit. Inputs start OFF.')

    def on_com(self, msg):
        values = np.array([msg.vector.x, msg.vector.y], dtype=float)
        if msg.header.frame_id == 'payload' and np.isfinite(values).all():
            self.current_com, self.com_received = values, time.monotonic()

    def reset_experiment(self):
        self.wrench.clear()
        self.com.moving = self.com.returning = self.com_owned = False
        self.com.next_vertex = 0

    def handle_key(self, key):
        key = key.lower()
        if key in ('c', 'r'):
            if self.com_received is None or time.monotonic() - self.com_received > self.clock_timeout:
                self.get_logger().warning('Waiting for fresh payload COM feedback from cooperation_plant')
                return
            if not self.com_owned:
                self.com.position = self.current_com.copy()
                self.com_owned = True
            if key == 'r':
                self.com.moving = self.com.returning = True
            else:
                self.com.moving = not self.com.moving
                self.com.returning = False
        elif len(key) == 1 and key in 'uiojkl':
            self.wrench.toggle('uiojkl'.index(key))
        elif key == 'p':
            self.wrench.toggle_mode()
        elif key == '-' and self.wrench.last_axis is not None:
            self.wrench.amplitudes[self.wrench.last_axis] *= -1.
        elif key == '0':
            self.wrench.clear()
        elif key in (' ', 'q'):
            self.reset_experiment()
            self.quit = key == 'q'
        else:
            return
        self.get_logger().info(
            f'COM={"return" if self.com.returning else "moving" if self.com.moving else "paused"}; '
            f'wrench={self.wrench.mode}, active={self.wrench.active.tolist()}, '
            f'amplitudes={self.wrench.amplitudes.tolist()}')

    def publish_wrench(self, value):
        msg = WrenchStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), self.frame
        xyz(msg.wrench.force, value[:3])
        xyz(msg.wrench.torque, value[3:])
        self.wrench_pub.publish(msg)

    def tick(self):
        dt = self.elapsed()
        # Presence heartbeat, independent of C/pause: RViz shows the actual
        # COM only while this experiment node is alive and the plant is fresh.
        visible = Bool()
        visible.data = dt is not None and not self.quit
        self.com_visibility_pub.publish(visible)
        value = np.zeros(6) if dt is None else self.wrench.advance(dt)
        self.publish_wrench(value)
        if dt is not None and self.com_owned:
            msg = Vector3Stamped()
            msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'payload'
            xyz(msg.vector, self.com.advance(dt))
            self.com_pub.publish(msg)

    def stop_output(self):
        self.reset_experiment()
        self.publish_wrench(np.zeros(6))
        visible = Bool()
        visible.data = False
        self.com_visibility_pub.publish(visible)


class WindDisturbance(TimedDisturbance):
    """One common World flow, converted to three body wrenches by the plant."""

    def __init__(self):
        super().__init__('wind_disturbance')
        self.wind = WindProfile(
            self.parameter('mean_velocity_world_mps', [2., 0., 0.]),
            self.parameter('gust_amplitude_world_mps', [.5, .2, .1]),
            self.parameter('gust_frequency_hz', [.2, .2, .2]),
            self.parameter('turbulence_std_mps', [.15, .15, .05]),
            self.parameter('turbulence_correlation_s', 1.),
            self.parameter('ramp_time_s', 1.), self.parameter('random_seed', 7))
        self.wind.set_enabled(bool(self.parameter('enabled', False)))
        self.velocity_pub = self.create_publisher(Vector3Stamped, WIND_TOPIC, 2)
        self.enabled_pub = self.create_publisher(Bool, WIND_ENABLE_TOPIC, 2)
        self.timer = self.create_timer(1. / self.rate, self.tick)
        self.get_logger().info(
            f'Wind {"ON" if self.wind.enabled else "OFF"}: w=toggle; '
            '0/SPACE=off; q=quit. Same field acts on payload + leader + follower.')

    def reset_experiment(self):
        self.wind.set_enabled(False)
        self.wind.reset()

    def handle_key(self, key):
        key = key.lower()
        if key == 'w':
            self.wind.set_enabled(not self.wind.enabled)
        elif key in ('0', ' ', 'q'):
            self.wind.set_enabled(False)
            self.quit = key == 'q'
        else:
            return
        self.get_logger().info('Wind ' + ('ON' if self.wind.enabled else 'OFF'))

    def tick(self):
        dt = self.elapsed()
        enabled = dt is not None and self.wind.enabled
        value = self.wind.advance(dt) if dt is not None else np.zeros(3)
        msg = Vector3Stamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'world'
        xyz(msg.vector, value)
        self.velocity_pub.publish(msg)
        # A zero flow is still air and can exert drag on a moving body. An
        # explicit enable heartbeat makes OFF mean exactly zero new wind load.
        flag = Bool()
        flag.data = enabled
        self.enabled_pub.publish(flag)

    def stop_output(self):
        self.wind.set_enabled(False)
        flag = Bool()
        flag.data = False
        self.enabled_pub.publish(flag)


def run(node_type, args=None):
    """Use this node's terminal only; restore it on q, Ctrl+C or any error."""
    rclpy.init(args=args)
    node, original = None, None
    try:
        node = node_type()
        if node.keyboard_enabled:
            if not sys.stdin.isatty():
                raise RuntimeError('Run in a terminal, or set keyboard_enabled:=false and use ~/key')
            original = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and not node.quit:
            rclpy.spin_once(node, timeout_sec=.01)
            if original is not None and select.select([sys.stdin], [], [], 0.)[0]:
                key = sys.stdin.read(1)
                if key:
                    node.handle_key(key)
                else:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if original is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, original)
        if node is not None:
            if rclpy.ok():
                node.stop_output()
                rclpy.spin_once(node, timeout_sec=0.)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def payload_main(args=None):
    run(PayloadDisturbance, args)


def wind_main(args=None):
    run(WindDisturbance, args)
