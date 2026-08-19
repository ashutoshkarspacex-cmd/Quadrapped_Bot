import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node

from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'pet_bot_test5'
    xacro_file_name = 'pet_bot_test5.urdf.xacro'

    # ============================================================
    # PACKAGE PATHS
    # ============================================================

    pkg_share = get_package_share_directory(package_name)

    world_file = os.path.join(
        pkg_share,
        'worlds',
        'small_warehouse_fixed.world'
    )

    bridge_yaml = os.path.join(
        pkg_share,
        'config',
        'bridge.yaml'
    )


    # ============================================================
    # INFO
    # ============================================================

    info = LogInfo(
        msg='[sim.launch.py] Launching PetBot simulation'
    )


    # ============================================================
    # IGNITION GAZEBO
    # ============================================================

    gazebo = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-r',
            '-v',
            '4',
            world_file
        ],
        output='screen'
    )


    # ============================================================
    # ROBOT STATE PUBLISHER
    # ============================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',

        parameters=[
            {
                'use_sim_time': True,

                'robot_description': ParameterValue(
                    Command([
                        'xacro ',
                        PathJoinSubstitution([
                            pkg_share,
                            'urdf',
                            xacro_file_name
                        ])
                    ]),
                    value_type=str
                )
            }
        ]
    )


    # ============================================================
    # SPAWN ROBOT
    # ============================================================

    spawn_entity = TimerAction(
        period=6.0,

        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'run',
                    'ros_ign_gazebo',
                    'create',

                    '-world',
                    'default',

                    '-name',
                    'pet_bot_test5',

                    '-topic',
                    'robot_description',

                    '-x',
                    '0',
                    '-y',
                    '0',
                    '-z',
                    '0.3'
                ],

                output='screen'
            )
        ]
    )


    # ============================================================
    # JOINT STATE BROADCASTER
    # ============================================================

    load_joint_state_broadcaster = TimerAction(
        period=15.0,

        actions=[
            Node(
                package='controller_manager',
                executable='spawner',

                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager',
                    '/controller_manager'
                ],

                output='screen'
            )
        ]
    )


    # ============================================================
    # WHOLE BODY POSITION CONTROLLER
    # ============================================================

    load_forward_position_controller = TimerAction(
        period=16.0,

        actions=[
            Node(
                package='controller_manager',
                executable='spawner',

                arguments=[
                    'forward_position_controller',
                    '--controller-manager',
                    '/controller_manager'
                ],

                output='screen'
            )
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_yaml}'
        ],

        output='screen'
    )


    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================

    return LaunchDescription([

        info,

        # Gazebo
        gazebo,

        # TF / robot description
        robot_state_publisher,

        # Spawn robot
        spawn_entity,

        # Controllers
        load_joint_state_broadcaster,
        load_forward_position_controller,

        # All ROS <-> Gazebo bridges
        bridge
    ])