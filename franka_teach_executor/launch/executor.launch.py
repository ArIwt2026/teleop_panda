import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    storage_dir = LaunchConfiguration("storage_dir")
    planning_group = LaunchConfiguration("planning_group")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    robot_state_topic = LaunchConfiguration("robot_state_topic")
    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    calibration_result_path = LaunchConfiguration("calibration_result_path")
    error_recovery_action = LaunchConfiguration("error_recovery_action")
    enable_robot_state_monitoring = LaunchConfiguration("enable_robot_state_monitoring")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")
    gripper_grasp_action = LaunchConfiguration("gripper_grasp_action")
    gripper_move_action = LaunchConfiguration("gripper_move_action")
    pipeline = LaunchConfiguration("pipeline")
    planner_id = LaunchConfiguration("planner_id")
    
    # Global motion profile parameters
    ptp_velocity_scale = LaunchConfiguration("ptp_velocity_scale")
    ptp_acceleration_scale = LaunchConfiguration("ptp_acceleration_scale")
    lin_velocity_scale = LaunchConfiguration("lin_velocity_scale")
    lin_acceleration_scale = LaunchConfiguration("lin_acceleration_scale")
    pick_place_velocity_scale = LaunchConfiguration("pick_place_velocity_scale")
    pick_place_acceleration_scale = LaunchConfiguration("pick_place_acceleration_scale")

    franka_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "panda_arm.urdf.xacro",
    )
    robot_description_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_xacro_file,
            " hand:=",
            load_gripper,
            " robot_ip:=",
            robot_ip,
            " ee_id:=",
            ee_id,
            " use_fake_hardware:=",
            use_fake_hardware,
            " fake_sensor_commands:=",
            fake_sensor_commands,
            " ros2_control:=true",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_config, value_type=str)
    }

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory("franka_moveit_config"),
        "srdf",
        "panda_arm.srdf.xacro",
    )
    robot_description_semantic_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_semantic_xacro_file,
            " hand:=",
            load_gripper,
            " ee_id:=",
            ee_id,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_config, value_type=str
        )
    }

    kinematics_yaml = load_yaml("franka_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = {"robot_description_planning": load_yaml("franka_moveit_config", "config/joint_limits.yaml")}
    ompl_planning_yaml = load_yaml("franka_moveit_config", "config/ompl_planning.yaml")
    pilz_planning_yaml = load_yaml("franka_moveit_config", "config/pilz_industrial_motion_planner_planning.yaml")

    executor_node = Node(
        package="franka_teach_executor",
        executable="franka_teach_executor_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {
                "storage_dir": storage_dir,
                "planning_group": planning_group,
                "joint_state_topic": joint_state_topic,
                "robot_state_topic": robot_state_topic,
                "base_frame": base_frame,
                "camera_frame": camera_frame,
                "calibration_result_path": calibration_result_path,
                "error_recovery_action": error_recovery_action,
                "enable_robot_state_monitoring": enable_robot_state_monitoring,
                "use_fake_hardware": use_fake_hardware,
                "gripper_grasp_action": gripper_grasp_action,
                "gripper_move_action": gripper_move_action,
                "planning_pipeline_id": pipeline,
                "planner_id": planner_id,
                "ptp_velocity_scale": ptp_velocity_scale,
                "ptp_acceleration_scale": ptp_acceleration_scale,
                "lin_velocity_scale": lin_velocity_scale,
                "lin_acceleration_scale": lin_acceleration_scale,
                "pick_place_velocity_scale": pick_place_velocity_scale,
                "pick_place_acceleration_scale": pick_place_acceleration_scale,
            },
            ompl_planning_yaml,
            pilz_planning_yaml,
            joint_limits_yaml,
        ],
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '-0.022542648636446606', '-0.04542167475614861', '0.03449884974187979',
            '0.023201672357022737', '-0.027023357208746875', '0.7093108444496831', '0.7039954165412466',
            'panda_link8', 'camera_front_link'
        ],
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("storage_dir", default_value=""),
            DeclareLaunchArgument("planning_group", default_value="panda_arm"),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            DeclareLaunchArgument("robot_state_topic", default_value="/franka_robot_state_broadcaster/robot_state"),
            DeclareLaunchArgument("base_frame", default_value="panda_link0"),
            DeclareLaunchArgument("camera_frame", default_value="camera_front_link"),
            DeclareLaunchArgument("calibration_result_path", default_value=""),
            DeclareLaunchArgument("enable_robot_state_monitoring", default_value="true"),
            DeclareLaunchArgument("robot_ip", default_value="dont-care"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
            DeclareLaunchArgument("load_gripper", default_value="true"),
            DeclareLaunchArgument("ee_id", default_value="franka_hand"),
            DeclareLaunchArgument(
                "error_recovery_action", default_value="/action_server/error_recovery"
            ),
            DeclareLaunchArgument(
                "gripper_grasp_action", default_value="/panda_gripper/grasp"
            ),
            DeclareLaunchArgument(
                "gripper_move_action", default_value="/panda_gripper/move"
            ),
            DeclareLaunchArgument("pipeline", default_value="pilz"),
            DeclareLaunchArgument("planner_id", default_value="PTP"),
            DeclareLaunchArgument("ptp_velocity_scale", default_value="0.6"),
            DeclareLaunchArgument("ptp_acceleration_scale", default_value="0.5"),
            DeclareLaunchArgument("lin_velocity_scale", default_value="0.3"),
            DeclareLaunchArgument("lin_acceleration_scale", default_value="0.2"),
            DeclareLaunchArgument("pick_place_velocity_scale", default_value="0.2"),
            DeclareLaunchArgument("pick_place_acceleration_scale", default_value="0.2"),
            # static_tf_publisher, # Removed to prevent conflict with MoveIt's camera_bridge_tf
            executor_node,
        ]
    )
