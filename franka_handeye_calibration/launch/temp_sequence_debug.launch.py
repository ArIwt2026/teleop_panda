from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    launch_moveit_rviz = LaunchConfiguration("launch_moveit_rviz")
    enable_preview = LaunchConfiguration("enable_preview")

    debug_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("franka_handeye_calibration"), "launch", "handeye_calibration.launch.py"]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": "true",
            "start_cameras": "false",
            "job_name": "temp_sequence_debug",
            "autostart": "false",
            "autostart_solve_after_collection": "false",
            "enable_preview": enable_preview,
            "launch_moveit_rviz": launch_moveit_rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="dont-care"),
            DeclareLaunchArgument("launch_moveit_rviz", default_value="true"),
            DeclareLaunchArgument("enable_preview", default_value="true"),
            debug_stack,
        ]
    )
