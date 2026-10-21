"""The one topic/schema list used by the recorder and the CSV converter."""

from collections.abc import Sequence

from hih_interfaces.msg import (AttitudeCmd, CenterOfPressure, Cmd, Input,
                                MinitroneState, MobObserverInput, NormalForce,
                                TorqueDob, Wrench)
from std_msgs.msg import Bool, Float64


# Each topic remains in its own vehicle namespace. The logger only subscribes;
# it never passes observations or commands between Leader and Follower.
TOPICS = {}
for drone in ('leader', 'follower'):
    prefix = f'/{drone}/minitrone/'
    TOPICS.update({
        prefix + 'state': (MinitroneState, 'hih_interfaces/msg/MinitroneState'),
        prefix + 'cmd': (Cmd, 'hih_interfaces/msg/Cmd'),
        prefix + 'att_cmd': (AttitudeCmd, 'hih_interfaces/msg/AttitudeCmd'),
        prefix + 'att_ref': (AttitudeCmd, 'hih_interfaces/msg/AttitudeCmd'),
        prefix + 'input': (Input, 'hih_interfaces/msg/Input'),
        prefix + 'wrench_cmd': (Wrench, 'hih_interfaces/msg/Wrench'),
        # TorqueDob fields all start with dob_, unlike legacy MoB force/moment.
        # The same logger entry works independently for /leader and /follower.
        prefix + 'dob_status': (TorqueDob, 'hih_interfaces/msg/TorqueDob'),
        prefix + 'actuation_wrench_body': (Wrench, 'hih_interfaces/msg/Wrench'),
        prefix + 'external_wrench_cmd': (Wrench, 'hih_interfaces/msg/Wrench'),
        prefix + 'external_wrench_hat_second_order':
            (Wrench, 'hih_interfaces/msg/Wrench'),
        prefix + 'mob_observer_input':
            (MobObserverInput, 'hih_interfaces/msg/MobObserverInput'),
        prefix + 'cop_real':
            (CenterOfPressure, 'hih_interfaces/msg/CenterOfPressure'),
        prefix + 'cop_solver_real':
            (CenterOfPressure, 'hih_interfaces/msg/CenterOfPressure'),
        prefix + 'cop_safety_attitude_offset':
            (AttitudeCmd, 'hih_interfaces/msg/AttitudeCmd'),
        prefix + 'cop_safety_active': (Bool, 'std_msgs/msg/Bool'),
        prefix + 'cop_safety_pitch_offset_deg': (Float64, 'std_msgs/msg/Float64'),
        prefix + 'cop_safety_yaw_offset_deg': (Float64, 'std_msgs/msg/Float64'),
        # CoP Euler-offset rate references [deg/s], before accumulated-angle saturation.
        prefix + 'cop_attitude/yaw_rate_reference_deg_s': (Float64, 'std_msgs/msg/Float64'),
        prefix + 'cop_attitude/pitch_rate_reference_deg_s': (Float64, 'std_msgs/msg/Float64'),
        # MuJoCo solver ground truth for the plate/payload interface.
        # contact_wrench_real is payload-on-drone in Body axes:
        # Fn=max(0,-force_0), Ft=sqrt(force_1^2+force_2^2).
        prefix + 'contact_wrench_real':
            (Wrench, 'hih_interfaces/msg/Wrench'),
        prefix + 'allocator_saturated': (Bool, 'std_msgs/msg/Bool'),
    })

TOPICS.update({
    # Standalone Leader normal-force P controller.
    '/leader/minitrone/normal_force':
        (NormalForce, 'hih_interfaces/msg/NormalForce'),
    '/leader/minitrone/normal_force_position_offset':
        (Cmd, 'hih_interfaces/msg/Cmd'),

    # Virtual payload-side 6-axis F/T sensors.
    #
    # Per-face topics are expressed about the corresponding payload face-center
    # sensor frame.  +Fx is inward-normal compression on BOTH Leader/Follower
    # faces.  The summed topic is shifted to payload CoM and expressed in the
    # payload body frame; it contains only drone/payload contacts.
    '/payload/contact_wrench_from_leader':
        (Wrench, 'hih_interfaces/msg/Wrench'),
    '/payload/contact_wrench_from_follower':
        (Wrench, 'hih_interfaces/msg/Wrench'),
    '/payload/contact_wrench_from_drones':
        (Wrench, 'hih_interfaces/msg/Wrench'),

    '/follower/minitrone/cmd_admittance':
        (Cmd, 'hih_interfaces/msg/Cmd'),
    '/follower/minitrone/att_cmd_admittance':
        (AttitudeCmd, 'hih_interfaces/msg/AttitudeCmd'),
    '/follower/minitrone/admittance_enable': (Bool, 'std_msgs/msg/Bool'),
    '/follower/minitrone/admittance_active': (Bool, 'std_msgs/msg/Bool'),
    '/follower/minitrone/admittance_des_force': (Float64, 'std_msgs/msg/Float64'),
    '/follower/minitrone/admittance/blend': (Float64, 'std_msgs/msg/Float64'),
    '/follower/minitrone/admittance/contact_wrench':
        (Wrench, 'hih_interfaces/msg/Wrench'),
    # Tangential-load adaptive preload. The original admittance_des_force above
    # remains the manually selected base force; target/reference include adaptation.
    '/follower/minitrone/admittance/normal_force_target':
        (Float64, 'std_msgs/msg/Float64'),
    '/follower/minitrone/admittance/normal_force_reference':
        (Float64, 'std_msgs/msg/Float64'),
    '/follower/minitrone/admittance/tangential_force':
        (Float64, 'std_msgs/msg/Float64'),
    '/follower/minitrone/admittance/adaptive_preload_enabled':
        (Bool, 'std_msgs/msg/Bool'),
    '/follower/minitrone/admittance/adaptive_preload_active':
        (Bool, 'std_msgs/msg/Bool'),
})


def flatten_message(message):
    """Flatten ROS messages into stable, self-describing CSV columns."""
    result = {}
    for name in message.get_fields_and_field_types():
        value = getattr(message, name)
        # rosidl may represent fixed numeric arrays as numpy.ndarray, which is
        # not collections.abc.Sequence. Do not serialize an entire DOB vector
        # into one opaque cell. __array__ avoids adding a new numpy dependency.
        array_like = isinstance(value, Sequence) or hasattr(value, '__array__')
        if array_like and not isinstance(value, (str, bytes)):
            for index, item in enumerate(value):
                result[f'{name}_{index}'] = item
        else:
            result[name] = value
    return result


CSV_COLUMNS = ('t_sec', 't_ns', 'topic_name', 'type_name', *sorted({
    column for message_class, _ in TOPICS.values()
    for column in flatten_message(message_class())
}))
