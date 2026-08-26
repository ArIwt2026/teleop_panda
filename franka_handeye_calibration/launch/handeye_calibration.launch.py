from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    start_moveit_stack = LaunchConfiguration("start_moveit_stack")
    start_executor = LaunchConfiguration("start_executor")
    start_cameras = LaunchConfiguration("start_cameras")
    camera_config_file = LaunchConfiguration("camera_config_file")
    launch_moveit_rviz = LaunchConfiguration("launch_moveit_rviz")
    job_name = LaunchConfiguration("job_name")
    job_config = LaunchConfiguration("job_config")
    autostart = LaunchConfiguration("autostart")
    autostart_solve_after_collection = LaunchConfiguration("autostart_solve_after_collection")
    enable_preview = LaunchConfiguration("enable_preview")
    executor_namespace = LaunchConfiguration("executor_namespace")
    enable_executor_robot_state_monitoring = LaunchConfiguration(
        "enable_executor_robot_state_monitoring"
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("franka_moveit_config"), "launch", "moveit.launch.py"]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "launch_rviz": launch_moveit_rviz,
        }.items(),
        condition=IfCondition(start_moveit_stack),
    )

    executor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("franka_teach_executor"), "launch", "executor.launch.py"]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "enable_robot_state_monitoring": enable_executor_robot_state_monitoring,
        }.items(),
        condition=IfCondition(start_executor),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("franka_mobile_sensors"), "launch", "cameras", "realsense_cameras.launch.py"]
            )
        ),
        launch_arguments={"config_file": camera_config_file}.items(),
        condition=IfCondition(start_cameras),
    )

    coordinator = Node(
        package="franka_handeye_calibration",
        executable="coordinator_node",
        output="screen",
        parameters=[
            {
                "job_name": job_name,
                "job_config_path": job_config,
                "enable_preview": ParameterValue(enable_preview, value_type=bool),
                "executor_namespace": executor_namespace,
                "autostart_job_name": job_name,
                "autostart_job_config_path": job_config,
                "autostart": ParameterValue(autostart, value_type=bool),
                "autostart_solve_after_collection": ParameterValue(
                    autostart_solve_after_collection, value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="dont-care"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
            DeclareLaunchArgument("start_moveit_stack", default_value="true"),
            DeclareLaunchArgument("start_executor", default_value="true"),
            DeclareLaunchArgument("start_cameras", default_value="true"),
            DeclareLaunchArgument("camera_config_file", default_value="handeye_d455_front"),
            DeclareLaunchArgument("launch_moveit_rviz", default_value="false"),
            DeclareLaunchArgument("job_name", default_value="d455_handeye"),
            DeclareLaunchArgument("job_config", default_value=""),
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("autostart_solve_after_collection", default_value="true"),
            DeclareLaunchArgument("enable_preview", default_value="true"),
            DeclareLaunchArgument("executor_namespace", default_value=""),
            DeclareLaunchArgument(
                "enable_executor_robot_state_monitoring",
                default_value=PythonExpression(
                    ["'false' if '", use_fake_hardware, "' == 'true' else 'true'"]
                ),
            ),
            moveit_launch,
            executor_launch,
            camera_launch,
            # Static transform to bridge robot and camera TF trees
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="robot_to_camera_bridge",
                arguments=[
                    '0.03268399504516506', '-0.02267864090603775', '-0.03775216539950842',  # Translation XYZ [m]
                    '-0.023201672357022737', '0.027023357208746875', '-0.7093108444496831', '0.7039954165412466',  # Quat XYWZ
                    'panda_hand_tcp',
                    'camera_front_link'
                ],
            ),
            coordinator,
        ]
    )
