"""Panda wrist-camera launch with capped home and button-controlled admittance."""

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_bringup'),
                'launch',
                'franka.launch.py',
            ])
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': 'true',
            'use_rviz': 'false',
            'use_fake_hardware': 'false',
            'load_admittance_controller': 'false',
            'load_cartesian_impedance_controller': 'false',
        }.items(),
    )

    from launch.actions import DeclareLaunchArgument
    cartesian_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_cartesian_impedance_controller',
        arguments=['cartesian_impedance_controller'],
        output='screen',
    )
    robot_group = GroupAction([
        PushRosNamespace('panda'),
        robot,
        cartesian_spawner,
    ])

    admittance_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_admittance_controller',
        namespace='panda',
        arguments=['admittance_controller'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='Panda robot IP address'),
        RegisterEventHandler(
            OnProcessExit(
                target_action=cartesian_spawner,
                on_exit=[admittance_spawner],
            )
        ),
        robot_group,
    ])
