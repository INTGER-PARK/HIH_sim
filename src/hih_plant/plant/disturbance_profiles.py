"""Simulation-time disturbance profiles; independent of ROS and MuJoCo."""
import math

import numpy as np


SIM_TIME_TOPIC = '/hih/disturbance/sim_time'
COM_COMMAND_TOPIC = '/hih/disturbance/payload/com_offset_cmd'
COM_OFFSET_TOPIC = '/hih/disturbance/payload/com_offset'
COM_VISIBILITY_TOPIC = '/hih/disturbance/payload/com_visualization_enabled'
PAYLOAD_WRENCH_TOPIC = '/hih/disturbance/payload/wrench_cmd'
WIND_TOPIC = '/hih/disturbance/wind/velocity_cmd'
WIND_ENABLE_TOPIC = '/hih/disturbance/wind/enabled'
ROLES = ('payload', 'leader', 'follower')


def finite_array(value, size, name, nonnegative=False):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must contain {size} finite values')
    if nonnegative and np.any(result < 0.):
        raise ValueError(name + ' must be nonnegative')
    return result.copy()


def positive(value, name):
    result = float(value)
    if not math.isfinite(result) or result <= 0.:
        raise ValueError(name + ' must be finite and positive')
    return result


class RectangleCOM:
    """Prescribed XY CoM, CCW as seen from +payload Z; speed in m/s."""

    def __init__(self, half_size=(.025, .025), speed=.01):
        x, y = finite_array(half_size, 2, 'com_half_size_m')
        if min(x, y) <= 0.:
            raise ValueError('COM rectangle half sizes must be positive')
        self.vertices = np.array([[x, y], [-x, y], [-x, -y], [x, -y]])
        self.speed = positive(speed, 'com_speed_mps')
        self.position = np.zeros(2)
        self.next_vertex = 0
        self.moving = self.returning = False

    def advance(self, dt):
        distance = max(0., float(dt)) * self.speed
        # Drop complete laps before the edge loop (also bounds long-gap work).
        if self.moving and not self.returning and distance > 0.:
            approach = np.linalg.norm(self.vertices[self.next_vertex] - self.position)
            if distance >= approach:
                self.position = self.vertices[self.next_vertex].copy()
                self.next_vertex = (self.next_vertex + 1) % 4
                perimeter = sum(np.linalg.norm(self.vertices[(i+1) % 4] - self.vertices[i])
                                for i in range(4))
                distance = (distance - approach) % perimeter
        while self.moving and distance > 1e-14:
            target = np.zeros(2) if self.returning else self.vertices[self.next_vertex]
            delta = target - self.position
            length = float(np.linalg.norm(delta))
            if length > distance:
                self.position += delta * (distance / length)
                break
            self.position = target.copy()
            distance -= length
            if self.returning:
                self.moving = self.returning = False
                self.next_vertex = 0
            else:
                self.next_vertex = (self.next_vertex + 1) % 4
        return np.r_[self.position, 0.]


class PayloadWrenchProfile:
    """Latched axes with independent sine phases and signed amplitudes."""

    def __init__(self, amplitudes, frequencies):
        self.amplitudes = finite_array(amplitudes, 6, 'wrench_amplitudes')
        self.frequencies = finite_array(frequencies, 6, 'sine_frequency_hz', True)
        self.active = np.zeros(6, dtype=bool)
        self.elapsed = np.zeros(6)
        self.mode = 'step'
        self.last_axis = None

    def toggle(self, axis):
        self.active[axis] = not self.active[axis]
        self.elapsed[axis] = 0.
        self.last_axis = axis

    def toggle_mode(self):
        self.mode = 'sine' if self.mode == 'step' else 'step'
        self.elapsed[:] = 0.

    def clear(self):
        self.active[:] = False
        self.elapsed[:] = 0.

    def advance(self, dt):
        self.elapsed[self.active] += max(0., float(dt))
        value = self.amplitudes * self.active
        if self.mode == 'sine':
            value = value * np.sin(2. * math.pi * self.frequencies * self.elapsed)
        return value


class WindProfile:
    """Uniform flow: mean + harmonic gust + time-correlated OU turbulence."""

    def __init__(self, mean, gust, frequency, noise_std, correlation, ramp, seed):
        self.mean = finite_array(mean, 3, 'mean_velocity_world_mps')
        self.gust = finite_array(gust, 3, 'gust_amplitude_world_mps')
        self.frequency = finite_array(frequency, 3, 'gust_frequency_hz', True)
        self.std = finite_array(noise_std, 3, 'turbulence_std_mps', True)
        self.correlation = positive(correlation, 'turbulence_correlation_s')
        self.ramp = positive(ramp, 'ramp_time_s')
        self.seed = int(seed)
        if self.seed < 0:
            raise ValueError('random_seed must be nonnegative')
        self.enabled = False
        self.reset()

    def reset(self):
        self.elapsed = 0.
        self.noise = np.zeros(3)
        self.rng = np.random.default_rng(self.seed)

    def set_enabled(self, enabled):
        if enabled and not self.enabled:
            self.reset()
        self.enabled = bool(enabled)

    def advance(self, dt):
        if not self.enabled:
            return np.zeros(3)
        dt = max(0., float(dt))
        self.elapsed += dt
        if dt > 0.:
            decay = math.exp(-dt / self.correlation)
            self.noise = (decay * self.noise + self.std * math.sqrt(1. - decay**2)
                          * self.rng.standard_normal(3))
        ramp = min(1., self.elapsed / self.ramp)
        return ramp * (self.mean + self.gust * np.sin(
            2. * math.pi * self.frequency * self.elapsed) + self.noise)


def wind_wrench(velocity_world, point_velocity_world, rotation, area, coefficient,
                density, arm_body):
    """Axis-wise quadratic drag; moment about the actual CoM, World axes."""
    relative_body = rotation.T @ (velocity_world - point_velocity_world)
    force_body = .5 * density * coefficient * area * np.abs(relative_body) * relative_body
    force_world = rotation @ force_body
    return np.r_[force_world, np.cross(rotation @ arm_body, force_world)]
