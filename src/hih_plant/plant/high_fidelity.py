"""Hardware-like, not identified, actuator/sensor/uncertainty models.

All time arguments are simulation seconds. Random streams are independent so
switching a sensor off does not change the motor gains or residual sequence.
Defaults are provisional experiment starting points, never hardware estimates.
"""
from collections import deque
import math

import numpy as np


# Single source for ROS declarations, example YAML and the README parameter table.
# name: (default, SI unit, meaning, future identification)
PARAMETERS = {
    'enable_high_fidelity': (False, '-', 'Master switch; example YAML enables it', 'configuration'),
    **{f'enable_{name}': (True, '-', f'Enable {name.replace("_", " ")}', 'configuration')
       for name in ('motor_dynamics', 'servo_dynamics', 'motor_mismatch',
                    'thrust_saturation', 'sensor_model', 'inertial_uncertainty',
                    'contact_uncertainty', 'residual_wrench')},
    'legacy_input_delay': (0.0075, 's', 'OFF-path effective legacy delay (old 4-slot ring delayed 3 ticks)', 'transport timestamp test'),
    'motor_command_delay': (0.01, 's', 'Motor transport delay, separate from lumped response', 'command/ESC timestamp test'),
    'motor_time_constant_up': (0.022, 's', 'Lumped motor+prop speed rising time constant', 'thrust stand step test'),
    'motor_time_constant_down': (0.025, 's', 'Lumped motor+prop speed falling time constant', 'thrust stand step test'),
    'motor_thrust_coefficient': (0.02, 'N/(rad/s)^2', 'Existing allocator omega convention; not calibrated physical shaft speed', 'thrust stand and command/shaft-speed calibration'),
    'motor_gains': ([1.0, 1.0, 1.0, 1.0], '-', 'Fixed per-motor gain multiplier', 'individual thrust tests'),
    'motor_gain_random_range': (0.03, '-', 'Startup uniform fractional gain half-range; zero selects fixed only', 'individual thrust tests'),
    'motor_thrust_max': (15.0, 'N', 'Final per-motor plant limit, intersected with XML limit', 'maximum continuous thrust test'),
    'servo_time_constant': (0.04, 's', 'First-order internal servo reference lag, before XML joint response', 'Dynamixel step-response identification'),
    'servo_command_delay': (0.005, 's', 'Servo transport delay', 'Dynamixel step-response identification'),
    'servo_rate_limit': (5.0, 'rad/s', 'Internal servo reference slew limit', 'Dynamixel step-response identification'),
    'servo_angle_min': (-1.1344640137963142, 'rad', 'Minimum reference angle (-65 deg)', 'servo travel test'),
    'servo_angle_max': (1.1344640137963142, 'rad', 'Maximum reference angle (+65 deg)', 'servo travel test'),
    'inertia_scale': ([1.0, 1.0, 1.0], '-', 'Fixed multipliers in XML principal-inertia frame', 'pendulum/system identification'),
    'inertia_random_range': (0.05, '-', 'Startup uniform fractional principal-inertia half-range', 'pendulum/system identification'),
    'com_offset': ([0.0, 0.0, 0.0], 'm', 'Fixed CoM displacement in body axes relative to XML ipos', 'balancing/suspension test'),
    'com_random_range': (0.003, 'm', 'Startup uniform CoM half-range per body axis', 'balancing/suspension test'),
    'contact_friction_scale': (1.0, '-', 'Multiplier on nominal geom friction', 'sliding contact experiment'),
    'contact_time_constant_scale': (1.0, '-', 'Positive solref timeconst multiplier; direct stiffness divided by scale squared', 'contact compliance test'),
    'contact_damping_scale': (1.0, '-', 'Positive solref dampratio or direct damping multiplier', 'contact damping test'),
    'contact_random_range': (0.05, '-', 'Startup uniform fractional half-range on each contact scale', 'contact experiments'),
    'residual_time_constant': (0.2, 's', 'OU correlation time', 'real-versus-sim residual autocorrelation'),
    'residual_force_std': ([0.05, 0.05, 0.05], 'N', 'Stationary OU force standard deviation in body axes', 'real-versus-sim residual logs'),
    'residual_moment_std': ([0.002, 0.002, 0.002], 'N*m', 'Stationary OU moment standard deviation about actual CoM', 'real-versus-sim residual logs'),
}
SENSOR_SPECS = {
    'position': (3, 'm', 100.0, 0.01, 0.001, 0.001, 0.0001),
    'velocity': (3, 'm/s', 100.0, 0.01, 0.001, 0.001, 0.0001),
    'attitude': (3, 'rad', 400.0, 0.0025, 0.001, 0.0005, 0.00001),
    'gyro': (3, 'rad/s', 400.0, 0.0025, 0.001, 0.001, 0.0001),
    'acceleration': (3, 'm/s^2', 400.0, 0.0025, 0.02, 0.01, 0.001),
    'angular_acceleration': (3, 'rad/s^2', 400.0, 0.0025, 0.01, 0.001, 0.0001),
    'servo': (4, 'rad', 50.0, 0.005, 0.0001, 0.0001, 0.00001),
}
for _name, (_size, _unit, _rate, _latency, _noise, _bias, _walk) in SENSOR_SPECS.items():
    for _suffix, _entry in {
        'update_rate_hz': (_rate, 'Hz', 'Acquisition rate; held between samples'),
        'latency_sec': (_latency, 's', 'Simulation-time sample delivery latency'),
        'noise_std': (_noise, _unit, 'Per-sample white noise standard deviation per axis'),
        'bias': ([0.0] * _size, _unit, 'Constant bias vector'),
        'initial_bias_std': (_bias, _unit, 'Startup Gaussian bias standard deviation per axis'),
        'bias_random_walk_std': (_walk, _unit + '/sqrt(s)', 'Bias diffusion; increments scale with sqrt(sample dt)'),
    }.items():
        PARAMETERS[f'sensor_{_name}_{_suffix}'] = (*_entry, 'static sensor logs / timestamp test')


def defaults():
    """Return a fresh configuration without sharing mutable default lists."""
    return {key: value[0].copy() if isinstance(value[0], list) else value[0]
            for key, value in PARAMETERS.items()}


def validate(config, dt):
    """Reject invalid input rather than silently inventing physical values."""
    for key, (default, _, _, _) in PARAMETERS.items():
        value = config[key]
        if isinstance(default, bool):
            if not isinstance(value, (bool, np.bool_)):
                raise ValueError(f'{key} must be boolean')
            continue
        array = np.asarray(value, dtype=float)
        expected = (len(default),) if isinstance(default, list) else ()
        if array.shape != expected or not np.isfinite(array).all():
            raise ValueError(f'{key} must have shape {expected} and finite values')
        signed = key in ('com_offset', 'servo_angle_min', 'servo_angle_max') or key.endswith('_bias')
        if not signed and np.any(array < 0):
            raise ValueError(f'{key} must be nonnegative')
        if ('time_constant' in key or key.endswith('_scale') or key in (
                'motor_thrust_coefficient', 'motor_thrust_max', 'motor_gains',
                'servo_rate_limit', 'residual_time_constant')) and np.any(array <= 0):
            raise ValueError(f'{key} must be positive')
        if key.endswith('update_rate_hz') and not 0 < value <= 1 / dt:
            raise ValueError(f'{key} must be in (0, physics_hz]')
    for key in ('motor_gain_random_range', 'inertia_random_range', 'contact_random_range'):
        if config[key] >= 1:
            raise ValueError(f'{key} must be less than 1')
    if config['servo_angle_min'] >= config['servo_angle_max']:
        raise ValueError('servo_angle_min must be less than servo_angle_max')


class DelayBuffer:
    """Causal zero-order hold, addressed by simulation timestamps, never sleep.

    Delivery occurs on the first physics tick at/after the requested time.
    Delays smaller than one tick are deliberately treated as zero: inventing a
    whole tick for a 0.15 ms transport delay would exaggerate the hardware lag.
    Larger delays round up to the next tick; no fractional interpolation.
    """

    def __init__(self, delay, dt, initial):
        if not math.isfinite(delay) or delay < 0 or dt <= 0:
            raise ValueError('delay must be finite/nonnegative and dt positive')
        self.delay = 0.0 if delay < dt else math.ceil(delay / dt - 1e-12) * dt
        self.queue = deque()
        self.value = np.array(initial, dtype=float, copy=True)

    def update(self, time, value):
        self.queue.append((time + self.delay, np.array(value, copy=True)))
        while self.queue and self.queue[0][0] <= time + 1e-12:
            _, self.value = self.queue.popleft()
        return self.value.copy()


class FirstOrderActuator:
    """One lumped ESC+motor+prop response in speed, not three cascaded lags.

    tau*domega/dt = omega_cmd - omega. Speed is retained because the existing
    Input API commands omega and thrust is quadratic in that state. Exact ZOH
    integration avoids Euler instability when tau approaches the physics dt.
    22/25 ms are provisional response assumptions, not identified constants.
    """

    def __init__(self, tau_up, tau_down, initial):
        if min(tau_up, tau_down) <= 0 or not np.isfinite([tau_up, tau_down]).all():
            raise ValueError('actuator time constants must be finite and positive')
        self.tau_up, self.tau_down = tau_up, tau_down
        self.value = np.array(initial, dtype=float, copy=True)

    def update(self, target, dt):
        tau = np.where(target > self.value, self.tau_up, self.tau_down)
        self.value += -np.expm1(-dt / tau) * (target - self.value)
        return self.value.copy()


class ServoDynamics(FirstOrderActuator):
    """Finite-bandwidth, rate-limited internal position reference.

    tau_s*dalpha_ref/dt = alpha_cmd-alpha_ref, |dalpha_ref/dt| <= rate.
    The existing MuJoCo position actuator still drives a physical hinge:
    measured alpha is that hinge's qpos, NOT this reference. Keeping that
    second-order mechanics avoids teleporting joints or erasing load response.
    The rate bound is on the reference; contact/load can move the hinge faster.
    Total measured lag includes XML kp/kv/armature and needs step identification.
    """

    def __init__(self, tau, rate, lower, upper, initial):
        super().__init__(tau, tau, initial)
        if not np.isfinite([rate, lower, upper]).all() or rate <= 0 or lower >= upper:
            raise ValueError('invalid servo rate or angle limits')
        self.rate, self.lower, self.upper = rate, lower, upper
        self.value = np.clip(self.value, lower, upper)

    def update(self, target, dt):
        target = np.clip(target, self.lower, self.upper)
        delta = -np.expm1(-dt / self.tau_up) * (target - self.value)
        self.value += np.clip(delta, -self.rate * dt, self.rate * dt)
        return self.value.copy()


class SensorChannel:
    """Independent acquisition, bias, noise, queued delivery and sample hold.

    At acquisition y=x+b+n; b += diffusion*sqrt(elapsed)*N(0,1).
    Queueing the entire measured sample also delays its bias/noise, as a real
    timestamped sensor packet does. Bias evolves only at acquisition; no noise
    is injected into MuJoCo state. Before the first packet is due, hold the
    initial truth (explicit bootstrap, since the message has no validity field).
    """

    def __init__(self, config, name, rng, dt):
        self.rng = rng
        prefix = f'sensor_{name}_'
        self.period = 1 / config[prefix + 'update_rate_hz']
        self.latency = DelayBuffer(config[prefix + 'latency_sec'], dt, []).delay
        self.noise = config[prefix + 'noise_std']
        self.walk = config[prefix + 'bias_random_walk_std']
        self.bias = np.asarray(config[prefix + 'bias'], dtype=float).copy()
        self.bias += rng.normal(0, config[prefix + 'initial_bias_std'], self.bias.shape)
        self.next_sample = 0.0
        self.last_sample = None
        self.queue = deque()
        self.value = None

    def update(self, time, truth):
        if self.value is None:
            self.value = np.array(truth, copy=True)
        if time + 1e-12 >= self.next_sample:
            elapsed = 0.0 if self.last_sample is None else time - self.last_sample
            self.bias += self.walk * math.sqrt(elapsed) * self.rng.normal(size=self.bias.shape)
            measured = np.asarray(truth) + self.bias + self.rng.normal(0, self.noise, self.bias.shape)
            self.queue.append((time + self.latency, measured))
            self.last_sample = time
            # Preserve phase at noninteger physics/sample-rate ratios; do not
            # fabricate multiple historical truth samples during one update.
            self.next_sample += (math.floor((time - self.next_sample + 1e-12) / self.period) + 1) * self.period
        while self.queue and self.queue[0][0] <= time + 1e-12:
            _, self.value = self.queue.popleft()
        return self.value.copy()


class ColoredWrenchProcess:
    """Exact discrete OU process in body axes, about actual base CoM.

    d[k+1]=rho*d[k]+std*sqrt(1-rho^2)*N(0,I), rho=exp(-dt/tau).
    std is stationary RMS, NOT continuous diffusion sigma. Start at zero to
    avoid an impulsive startup. This phenomenological residual is external,
    never part of the motor wrench reported to the momentum observer.
    """

    def __init__(self, tau, std, rng):
        if tau <= 0 or not math.isfinite(tau) or not np.isfinite(std).all() or np.any(np.asarray(std) < 0):
            raise ValueError('invalid OU time constant or standard deviation')
        self.tau, self.std, self.rng = tau, np.asarray(std), rng
        self.value = np.zeros(6)

    def update(self, dt):
        rho = math.exp(-dt / self.tau)
        self.value = rho * self.value + self.std * math.sqrt(-math.expm1(-2 * dt / self.tau)) * self.rng.normal(size=6)
        return self.value.copy()


class HighFidelityModel:
    """Small orchestration layer; no ROS dependency, independently testable."""

    def __init__(self, config, seed, dt):
        validate(config, dt)
        if seed < 0:
            raise ValueError('random_seed must be nonnegative')
        self.config, self.dt = config, dt
        streams = np.random.SeedSequence(seed).spawn(4 + len(SENSOR_SPECS))
        self.motor_rng, self.inertia_rng, self.contact_rng, residual_rng = (
            np.random.default_rng(s) for s in streams[:4])
        self.motor_gains = np.ones(4)
        if self.enabled('motor_mismatch'):
            # Manufacturing variation is sampled ONCE, not injected every tick.
            self.motor_gains = np.asarray(config['motor_gains']) * (
                1 + self.motor_rng.uniform(-config['motor_gain_random_range'], config['motor_gain_random_range'], 4))
        self.motor = FirstOrderActuator(config['motor_time_constant_up'], config['motor_time_constant_down'], np.zeros(4))
        self.servo = ServoDynamics(config['servo_time_constant'], config['servo_rate_limit'], config['servo_angle_min'], config['servo_angle_max'], np.zeros(4))
        self.motor_delay = DelayBuffer(config['motor_command_delay'] if self.enabled('motor_dynamics') else config['legacy_input_delay'], dt, np.zeros(4))
        self.servo_delay = DelayBuffer(config['servo_command_delay'] if self.enabled('servo_dynamics') else config['legacy_input_delay'], dt, np.zeros(4))
        self.sensors = {name: SensorChannel(config, name, np.random.default_rng(streams[4 + i]), dt)
                        for i, name in enumerate(SENSOR_SPECS)}
        self.residual = ColoredWrenchProcess(config['residual_time_constant'],
                                            np.concatenate((config['residual_force_std'], config['residual_moment_std'])), residual_rng)

    def enabled(self, feature):
        return self.config['enable_high_fidelity'] and self.config['enable_' + feature]

    def actuate(self, time, command, xml_thrust_limits):
        # Reject NaN/Inf per channel; clip speed BEFORE squaring to avoid
        # overflow and hidden actuator windup beyond the achievable thrust.
        u = np.nan_to_num(np.asarray(command), nan=0.0, posinf=0.0, neginf=0.0)
        upper = np.asarray(xml_thrust_limits, dtype=float)
        if self.enabled('thrust_saturation'):
            upper = np.minimum(upper, self.config['motor_thrust_max'])
        coefficient = self.config['motor_thrust_coefficient'] if self.config['enable_high_fidelity'] else PARAMETERS['motor_thrust_coefficient'][0]
        gains = self.motor_gains
        omega = np.clip(u[:4], 0, np.sqrt(upper / (coefficient * gains)))
        omega = self.motor_delay.update(time, omega)
        if self.enabled('motor_dynamics'):
            omega = self.motor.update(omega, self.dt)
        thrust = np.clip(gains * coefficient * np.square(omega), 0, upper)
        angles = self.servo_delay.update(time, u[4:])
        if self.enabled('servo_dynamics'):
            angles = self.servo.update(angles, self.dt)
        return thrust, angles

    def external_residual(self):
        return self.residual.update(self.dt) if self.enabled('residual_wrench') else np.zeros(6)

    def measure(self, time, truth):
        return {name: self.sensors[name].update(time, value) for name, value in truth.items()}

    def configure_physics(self, model, data, base_id, contact_ids):
        """Perturb compiled nominal physics once, never controller parameters."""
        import mujoco

        c = self.config
        self.inertia_scale = np.ones(3)
        self.com_offset = np.zeros(3)
        if self.enabled('inertial_uncertainty'):
            # body_inertia is the three principal moments about body_ipos in
            # body_iquat axes, NOT necessarily the body X/Y/Z diagonal. Preserve
            # iquat, scale principal moments, then validate the triangle rule.
            self.inertia_scale = np.asarray(c['inertia_scale']) * (1 + self.inertia_rng.uniform(-c['inertia_random_range'], c['inertia_random_range'], 3))
            inertia = model.body_inertia[base_id].copy() * self.inertia_scale
            if not np.isfinite(inertia).all() or np.any(inertia <= 0) or 2 * max(inertia) > sum(inertia):
                raise ValueError('sampled inertia is not physical: positive principal moments and triangle inequality required; reduce uncertainty')
            # ipos is local to the body. Moving it changes the mass distribution;
            # J remains specified ABOUT the new CoM (no extra parallel-axis J).
            self.com_offset = np.asarray(c['com_offset']) + self.inertia_rng.uniform(-c['com_random_range'], c['com_random_range'], 3)
            model.body_inertia[base_id] = inertia
            model.body_ipos[base_id] += self.com_offset
            # MuJoCo 3.3.7 caches equality of body/inertial frames at compile
            # time. mj_setConst does NOT clear body_sameframe: otherwise xipos
            # ignores this offset even though the editable ipos array changed.
            # Use general kinematics and, if needed, general inertia assembly.
            model.body_sameframe[base_id] = int(mujoco.mjtSameFrame.mjSAMEFRAME_NONE)
            if model.body_simple[base_id]:
                model.body_simple[:] = 0
                model.dof_simplenum[:] = 0
            mujoco.mj_setConst(model, data)
        self.contact_scales = np.ones(3)
        if self.enabled('contact_uncertainty'):
            self.contact_scales = np.array([c['contact_friction_scale'], c['contact_time_constant_scale'], c['contact_damping_scale']]) * (1 + self.contact_rng.uniform(-c['contact_random_range'], c['contact_random_range'], 3))
            for geom_id in sorted(set(contact_ids)):
                # Apply after A/B/C/D selection; keep zeros, condim, solimp and
                # collision masks. Positive solref=(timeconst,dampratio), while
                # negative solref=(-stiffness,-damping) needs different scaling.
                model.geom_friction[geom_id] *= self.contact_scales[0]
                ref = model.geom_solref[geom_id].copy()
                if np.all(ref > 0):
                    ref *= self.contact_scales[1:]
                    ref[0] = max(2 * self.dt, ref[0])
                elif np.all(ref <= 0):
                    ref[0] /= self.contact_scales[1] ** 2
                    ref[1] *= self.contact_scales[2]
                else:
                    raise ValueError('mixed-sign solref is unsupported for contact uncertainty')
                if not np.isfinite(ref).all():
                    raise ValueError('contact scaling produced nonfinite solref')
                model.geom_solref[geom_id] = ref
        mujoco.mj_forward(model, data)

    def summary(self, seed):
        c = self.config
        switches = {key: value for key, value in c.items() if key.startswith('enable_')}
        rates = {name: c[f'sensor_{name}_update_rate_hz'] for name in SENSOR_SPECS}
        biases = {name: channel.bias.tolist() for name, channel in self.sensors.items()}
        return (f'[high_fidelity] PROVISIONAL, not identified; seed={seed}; switches={switches}; '
                f'motor tau up/down={c["motor_time_constant_up"]}/{c["motor_time_constant_down"]} s; '
                f'motor gains={self.motor_gains.tolist()}; '
                f'effective motor/servo delays={self.motor_delay.delay}/{self.servo_delay.delay} s; '
                f'servo tau={c["servo_time_constant"]} s, reference rate limit={c["servo_rate_limit"]} rad/s; '
                f'CoM offset={self.com_offset.tolist()} m; inertia scale={self.inertia_scale.tolist()}; '
                f'contact scales={self.contact_scales.tolist()}; sensor rates={rates}; initial biases={biases}; '
                f'residual enabled={self.enabled("residual_wrench")}, tau={c["residual_time_constant"]} s')
