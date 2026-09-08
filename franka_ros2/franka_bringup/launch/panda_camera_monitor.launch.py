"""Monitor-only Panda state and wrist-camera launch.

Starts the Franka hardware interface and state broadcasters, but no motion
controller. The robot can therefore be hand-guided while camera calibration
samples are collected.
"""

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
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
        }.items(),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_mobile_sensors'),
                'launch',
                'cameras',
                'realsense_cameras.launch.py',
            ])
        ),
        launch_arguments={'config_file': 'simple_front_camera'}.items(),
    )

    from launch.actions import DeclareLaunchArgument
    robot_group = GroupAction([
        PushRosNamespace('panda'),
        robot,
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_impedance_zero_controller', '--inactive'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_impedance_hold_controller'],
            output='screen',
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='Panda robot IP address'),
        robot_group,
        camera,
    ])
