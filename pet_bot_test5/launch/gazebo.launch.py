import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ============================================================
    # PACKAGE PATHS
    # ============================================================

    package_name = 'pet_bot_test5'

    pkg_share = get_package_share_directory(package_name)

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'pet_bot_test5.urdf.xacro'
    )

    world_file = os.path.join(
        pkg_share,
        'worlds',
        'small_warehouse_fixed.world'
    )

    models_path = os.path.join(
        pkg_share,
        'models'
    )

    # Parent of the installed package share.
    # This allows:
    # model://pet_bot_test5/meshes/...
    package_parent = os.path.dirname(pkg_share)

    # ============================================================
    # GAZEBO RESOURCE PATH
    # ============================================================

    gazebo_resource_path = (
        package_parent
        + ':' +
        models_path
        + ':' +
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gazebo_resource_path
    )

    # ============================================================
    # INFO
    # ============================================================

    info = LogInfo(
        msg='[sim.launch] Starting Pet Bot + Small Warehouse'
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
                        xacro_file
                    ]),
                    value_type=str
                )
            }
        ]
    )

    # ============================================================
    # SPAWN PET BOT
    # ============================================================

    spawn_entity = TimerAction(
        period=7.0,

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
                    '0.0',

                    '-y',
                    '0.0',

                    '-z',
                    '0.30'
                ],

                output='screen'
            )
        ]
    )

    # ============================================================
    # JOINT STATE BROADCASTER
    # ============================================================

    joint_state_broadcaster = TimerAction(
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

    forward_position_controller = TimerAction(
        period=17.0,

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

    # ============================================================
    # PET BOT POSE BRIDGE
    # ============================================================

    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[
            '/model/pet_bot_test5/pose'
            '@geometry_msgs/msg/PoseArray'
            '@gz.msgs.Pose_V'
        ],

        output='screen'
    )

    # ============================================================
    # DEPTH CAMERA IMAGE BRIDGE
    # ============================================================

    # ============================================================
    # LIDAR BRIDGE
    # ============================================================

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],

        output='screen'
    )

    # ============================================================
    # LAUNCH EVERYTHING
    # ============================================================

    return LaunchDescription([

        # Gazebo resource paths must be set before Gazebo starts
        set_gazebo_resource_path,

        info,

        # Start world
        gazebo,

        # Publish robot_description
        robot_state_publisher,

        # Spawn robot
        spawn_entity,

        # Controllers
        joint_state_broadcaster,
        forward_position_controller,

        # ROS <-> Gazebo bridges
        pose_bridge,
        lidar_bridge,
    ])