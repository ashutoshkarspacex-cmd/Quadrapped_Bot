import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'pet_bot_test5'
    xacro_file_name = 'pet_bot_test5.urdf.xacro'

    # Path to custom world file
    world_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'debug_world.sdf'
    )

    # --- Launch Ignition Gazebo with custom world ---
    gazebo = ExecuteProcess(
        cmd=[
            'ign', 'gazebo',
            '-r',  # run immediately
            '-v', '4',  # verbose level
            world_file  # force this world file
        ],
        output='screen'
    )

    # --- Static transform publisher ---
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare(package_name),
                        'urdf',
                        xacro_file_name
                    ])
                ]),
                value_type=str
            )
        }]
    )

    # --- Spawn robot after world loads ---
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_ign_gazebo', 'create',
                    '-world', 'default',
                    '-name', 'pet_bot_test5',
                    '-topic', 'robot_description',
                    '-x', '0', '-y', '0', '-z', '0.2'
                ],
                output='screen'
            )
        ]
    )

    # --- Calibration topic ---
    calibrate = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/calibrated', 'std_msgs/msg/Bool', '{data: true}'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        static_tf,
        robot_state_publisher,
        spawn_entity,
        calibrate
    ])
