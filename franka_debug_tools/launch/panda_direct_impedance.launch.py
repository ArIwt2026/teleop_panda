from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('franka_description'), 'robots', 'panda_arm.urdf.xacro']),
            ' hand:=true robot_ip:=', robot_ip,
            ' use_fake_hardware:=false fake_sensor_commands:=false',
        ])
    }
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])),
        launch_arguments={'robot_ip': robot_ip, 'use_fake_hardware': 'false'}.items())
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_mobile_sensors'), 'launch', 'cameras',
            'realsense_cameras.launch.py'])),
        launch_arguments={'config_file': 'simple_front_camera'}.items())

    panda_group = GroupAction([
        PushRosNamespace('panda'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen', parameters=[robot_description]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', output='screen', parameters=[{
                 'source_list': ['franka/joint_states', 'panda_gripper/joint_states'],
                 'rate': 250,
             }]),
        gripper_launch,
        Node(package='franka_debug_tools', executable='direct_cartesian_impedance',
             name='direct_cartesian_impedance', output='screen',
             parameters=[PathJoinSubstitution([
                 FindPackageShare('franka_debug_tools'), 'config', 'joint_limits.yaml']),
                 {'robot_ip': robot_ip}]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.2'),
        panda_group,
        camera_launch,
    ])
