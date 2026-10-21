"""Launch one MuJoCo world and independent Leader/Follower control stacks.

Follower admittance and data recording are controlled by launch arguments.
Teleop, trajectory, torque DoB and CoP offset nodes run separately.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _default_log_output_dir():
    for parent in Path(__file__).resolve().parents:
        source_package = parent / 'src' / 'hih_logging'
        if source_package.is_dir():
            return str(source_package / 'recordings')
    return str(Path.cwd() / 'src' / 'hih_logging' / 'recordings')


def _bool_arg(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('hih_plant'), 'config', 'cooperation_high_fidelity.yaml'])

    nodes = [Node(
        package='hih_plant', executable='cooperation_plant',
        name='cooperation_plant', output='screen',
        parameters=[config, LaunchConfiguration('disturbance_config'),
                    {'enable_viewer': _bool_arg('viewer'),
                     'enable_cop_touch': _bool_arg('cop_touch'),
                     'enable_cop_solver': _bool_arg('cop_solver'),
                     'enable_high_fidelity': True}])]

    # Each role runs its own wrench controller, allocator and momentum observer.
    # Mass, inertia and approach speed use each node's C++ defaults.
    for label in ('leader', 'follower'):
        nodes.extend([
            Node(
                package='hih_controller',
                executable=f'{label}_wrench_controller',
                output='screen',
                parameters=[{'hold_initial_position': True}]),
            Node(
                package='hih_controller',
                executable='hih_allocator_controller',
                namespace=label,
                output='screen'),
            Node(
                package='hih_controller',
                executable=f'{label}_second_order_momentum_observer',
                output='screen'),
        ])

    # Start Follower admittance inactive; enable it through its normal interface.
    nodes.append(Node(
        package='hih_controller',
        executable='follower_admittance_controller',
        condition=IfCondition(LaunchConfiguration('embedded_admittance')),
        output='screen',
        parameters=[{'enabled': False, 'auto_approach': True}]))

    nodes.append(Node(
        package='hih_logging', executable='hih_record_csv',
        condition=IfCondition(LaunchConfiguration('record_data')),
        output='screen',
        arguments=['--output-dir', LaunchConfiguration('log_output_dir')],
        sigterm_timeout='120', sigkill_timeout='10'))

    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('disturbance_config', default_value=PathJoinSubstitution([
            FindPackageShare('hih_plant'), 'config', 'disturbances.yaml'])),
        # CoP combinations: true/true=touch+solver, true/false=touch only,
        # false/true=solver only, false/false=both disabled.
        # MoB CoP and contact-wrench telemetry are independent of these switches.
        DeclareLaunchArgument('cop_touch', default_value='true'),
        DeclareLaunchArgument('cop_solver', default_value='true'),
        DeclareLaunchArgument('record_data', default_value='true'),
        DeclareLaunchArgument(
            'log_output_dir', default_value=_default_log_output_dir()),
        DeclareLaunchArgument('embedded_admittance', default_value='true'),
        *nodes,
    ])

