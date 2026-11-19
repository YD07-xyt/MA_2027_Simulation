#!/usr/bin/env python3

import os
import shlex

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


class WorldType:
    RMUC = 'RMUC'
    RMUL = 'RMUL'
    RMUL2026H = 'RMUL2026H'
    RMUC2027 = 'RMUC2027'


def get_world_config(world_type):
    world_configs = {
        WorldType.RMUC: {
            'x': '6.35',
            'y': '7.6',
            'z': '0.2',
            'yaw': '0.0',
            'world_path': 'RMUC2024_world/RMUC2024_world.world',
        },
        WorldType.RMUL: {
            'x': '4.3',
            'y': '3.35',
            'z': '1.16',
            'yaw': '0.0',
            'world_path': 'RMUL2024_world/RMUL2024_world.world',
        },
        WorldType.RMUL2026H: {
            'x': '5.0',
            'y': '-3.0',
            'z': '0.10',
            'yaw': '3.14159',
            'world_path': 'RMUL2026_world/RMUL2026_world.world',
        },
        WorldType.RMUC2027: {
            # 出生点下方地面高约 0.29m，z 留出轮子半径和余量
            'x': '3.21',
            'y': '9.6',
            'z': '0.4',
            'yaw': '0.0',
            'world_path': 'RMUC2027_world/RMUC2027_world.world',
        },
    }
    return world_configs.get(world_type, None)


def _str_to_bool(value: str) -> bool:
    return value.strip().lower() in ('true', '1', 'yes', 'on')


def _launch_world_and_spawn(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('pb_rm_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_type = LaunchConfiguration('world').perform(context)
    world_config = get_world_config(world_type)
    if world_config is None:
        raise ValueError(
            f"Unsupported world '{world_type}'. "
            f"Expected one of: {WorldType.RMUC}, {WorldType.RMUL}, "
            f"{WorldType.RMUL2026H}, {WorldType.RMUC2027}"
        )

    gui = _str_to_bool(LaunchConfiguration('gui').perform(context))
    gz_verbosity = LaunchConfiguration('gz_verbosity').perform(context)
    world_file = os.path.join(bringup_dir, 'world', world_config['world_path'])

    gz_flags = ['-r', '-v', gz_verbosity]
    if not gui:
        gz_flags = ['-s', '--headless-rendering'] + gz_flags
    gz_args = ' '.join(gz_flags + [shlex.quote(world_file)])

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-world', 'default',
            '-name', 'robot',
            '-topic', 'robot_description',
            '-x', world_config['x'],
            '-y', world_config['y'],
            '-z', world_config['z'],
            '-Y', world_config['yaw'],
        ],
    )

    return [gz_sim_launch, spawn_robot]


def generate_launch_description():
    bringup_dir = get_package_share_directory('pb_rm_simulation')
    install_lib_dir = os.path.abspath(os.path.join(bringup_dir, '..', '..', 'lib'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_livox = LaunchConfiguration('use_livox')
    robot_description = LaunchConfiguration('robot_description')

    default_robot_description = Command([
        'xacro ',
        os.path.join(bringup_dir, 'urdf', 'simulation_waking_robot.xacro'),
        ' use_livox:=', use_livox,
    ])

    append_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.pathsep.join([
            os.path.join(bringup_dir, 'meshes'),
            os.path.dirname(bringup_dir),
        ]),
    )

    append_plugin_path = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        install_lib_dir,
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=WorldType.RMUC,
        description='Choose <RMUC>, <RMUL>, <RMUL2026H> or <RMUC2027>',
    )

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo Harmonic GUI (set false for headless GPU simulation)',
    )

    declare_gz_verbosity_cmd = DeclareLaunchArgument(
        'gz_verbosity',
        default_value='2',
        description='Gazebo Harmonic console verbosity level (0-4)',
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 if true',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
        description='Full path to the RViz2 config file to use',
    )

    declare_use_livox_cmd = DeclareLaunchArgument(
        'use_livox',
        default_value='false',
        description='Enable Livox Mid-360 GPU lidar xacro macro',
    )

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=default_robot_description,
        description='Robot description URDF XML',
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description, value_type=str),
        }],
        output='screen',
    )

    start_ros_gz_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pb_rm_gz_bridge',
        parameters=[{
            'config_file': os.path.join(bringup_dir, 'config', 'gz_bridge.yaml'),
        }],
        output='screen',
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        namespace='',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(append_resource_path)
    ld.add_action(append_plugin_path)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_gz_verbosity_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_livox_cmd)
    ld.add_action(declare_robot_description_cmd)

    ld.add_action(OpaqueFunction(function=_launch_world_and_spawn))
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_ros_gz_bridge_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
