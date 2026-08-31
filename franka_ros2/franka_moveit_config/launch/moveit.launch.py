#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, Shutdown, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import launch


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    teaching_mode_parameter_name = 'teaching_mode'

    ros2_controllers_path_real = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'config',
        'panda_ros_controllers.yaml',
    )
    ros2_controllers_path_fake = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'config',
        'panda_ros_controllers_fake.yaml',
    )

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    teaching_mode = LaunchConfiguration(teaching_mode_parameter_name)

    # Command-line arguments

    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    # planning_context
    # franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
    #                                  'panda_arm.urdf.xacro')
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'panda_arm_with_env.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])

    robot_description = {'robot_description': robot_description_config}

    franka_semantic_xacro_file = os.path.join(get_package_share_directory('franka_moveit_config'),
                                              'srdf',
                                              'panda_arm.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=true']
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'franka_moveit_config', 'config/kinematics.yaml'
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'franka_moveit_config', 'config/panda_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    planning_pipelines_config = PathJoinSubstitution(
        [
            FindPackageShare('franka_moveit_config'),
            "config",
            "planning_pipelines_config.yaml",
        ]
    )

    move_group_capabilities = {
        "capabilities": ""
    }

    move_group_request_adapters = {
        "request_adapters": """default_planner_request_adapters/AddRuckigTrajectorySmoothing \
           default_planner_request_adapters/AddTimeOptimalParameterization"""
    }

    pilz_planning_pipeline_config = {
        "move_group": {},
        "robot_description_planning":{},
    }

    pilz_planning_yaml = load_yaml("franka_moveit_config", "config/pilz_industrial_motion_planner_planning.yaml")
    pilz_planning_pipeline_config["move_group"].update(pilz_planning_yaml)
    pilz_joint_limits_yaml = load_yaml("franka_moveit_config", "config/joint_limits.yaml")
    pilz_planning_pipeline_config["robot_description_planning"].update(pilz_joint_limits_yaml)

    pilz_cartesian_limits_yaml = load_yaml("franka_moveit_config", "config/pilz_cartesian_limits.yaml")
    pilz_planning_pipeline_config["robot_description_planning"].update(pilz_cartesian_limits_yaml)

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_pipelines_config,
            ompl_planning_pipeline_config,
            pilz_planning_pipeline_config,
            move_group_capabilities,
            move_group_request_adapters,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory('franka_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = PythonExpression(
        [
            "'",
            ros2_controllers_path_real,
            "' if '",
            use_fake_hardware,
            "' == 'false' else '",
            ros2_controllers_path_fake,
            "'",
        ]
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        arguments=['--ros-args', '--log-level', 'controller_manager:=FATAL'],
        output='screen',
        # The persistent robot_supervisor restarts the complete stack. Respawning
        # only controller_manager would leave the one-shot spawners behind.
        respawn=False,
    )

    arm_controller_supervised = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'panda_arm_controller', '--inactive',
             '--controller-manager-timeout', '30', '--service-call-timeout', '30'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('supervised_controller_start')),
    )
    arm_controller_standalone = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'panda_arm_controller', '--controller-manager-timeout', '30',
             '--service-call-timeout', '30'],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('supervised_controller_start'), "' == 'false' and '",
            teaching_mode, "' == 'false'"
        ])),
    )
    arm_controller_teaching = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'panda_arm_controller', '--inactive',
             '--controller-manager-timeout', '30', '--service-call-timeout', '30'],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('supervised_controller_start'), "' == 'false' and '",
            teaching_mode, "' == 'true'"
        ])),
    )

    # Load both state broadcasters through one spawner process. Starting three
    # independent spawners concurrently caused controller_manager service calls
    # to time out under load. The arm controller is spawned only after both
    # broadcasters have completed.
    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'franka_robot_state_broadcaster', 'joint_state_broadcaster',
            '--controller-manager-timeout', '30', '--service-call-timeout', '30'
        ],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", use_fake_hardware, "' == 'false' and '",
            LaunchConfiguration('supervised_controller_start'), "' == 'false'"
        ])),
    )
    fake_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '30', '--service-call-timeout', '30'
        ],
        output='screen',
        condition=IfCondition(use_fake_hardware),
    )
    fake_arm_controller_normal = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'panda_arm_controller',
             '--controller-manager-timeout', '30', '--service-call-timeout', '30'],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", use_fake_hardware, "' == 'true' and '",
            teaching_mode, "' == 'false'"
        ])),
    )
    fake_arm_controller_teaching = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'panda_arm_controller', '--inactive',
             '--controller-manager-timeout', '30', '--service-call-timeout', '30'],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", use_fake_hardware, "' == 'true' and '",
            teaching_mode, "' == 'true'"
        ])),
    )
    load_controllers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=franka_robot_state_broadcaster,
                on_exit=[
                    arm_controller_supervised,
                    arm_controller_standalone,
                    arm_controller_teaching,
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=fake_joint_state_broadcaster,
                on_exit=[fake_arm_controller_normal, fake_arm_controller_teaching],
            )
        ),
        fake_joint_state_broadcaster,
    ]

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            robot_description,
            {'source_list': ['franka/joint_states', 'panda_gripper/joint_states'], 'rate': 30}],
    )

    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper',
        default_value='true',
        description='Launch and home the gripper')

    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        default_value='dont-care',
        description='Hostname or IP address of the robot.')

    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description='Use fake hardware')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))
            
    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={'robot_ip': robot_ip,
                          use_fake_hardware_parameter_name: use_fake_hardware}.items(),
        condition=launch.conditions.IfCondition(LaunchConfiguration('load_gripper'))
    )

    # Optional Camera Launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_mobile_sensors'), 'launch', 'cameras', 'realsense_cameras.launch.py'])]),
        launch_arguments={'config_file': LaunchConfiguration('camera_config_file')}.items(),
        condition=IfCondition(LaunchConfiguration('start_cameras'))
    )

    # Camera to Robot Bridge TF
    camera_bridge_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_to_camera_bridge',
        arguments=[
            '-0.022542648636446606', '-0.04542167475614861', '0.03449884974187979', 
            '0.023201672357022737', '-0.027023357208746875', '0.7093108444496831', '0.7039954165412466',
            'panda_hand_tcp', 'panda_hand_camera_link'
        ],
        condition=IfCondition(LaunchConfiguration('start_cameras'))
    )

    #########
    # world_frame_publisher = Node(
    # package='tf2_ros',
    # executable='static_transform_publisher',
    # name='world_to_base_link',
    # arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'panda_link0']
    # )

    # Dynamic search for franka_fault_manager.py without hardcoded paths
    fault_manager_path = ""
    try:
        fault_manager_path = os.path.join(
            get_package_share_directory('franka_debug_tools'),
            'franka_fault_manager.py'
        )
    except Exception:
        pass

    if not fault_manager_path or not os.path.exists(fault_manager_path):
        curr = os.path.dirname(os.path.abspath(__file__))
        while curr and curr != os.path.dirname(curr):
            target = os.path.join(curr, 'franka_debug_tools', 'franka_fault_manager.py')
            if os.path.exists(target):
                fault_manager_path = target
                break
            src_dir = os.path.join(curr, 'src')
            if os.path.exists(src_dir) and os.path.isdir(src_dir):
                for item in os.listdir(src_dir):
                    subdir = os.path.join(src_dir, item)
                    if os.path.isdir(subdir):
                        target = os.path.join(subdir, 'franka_debug_tools', 'franka_fault_manager.py')
                        if os.path.exists(target):
                            fault_manager_path = target
                            break
                if fault_manager_path:
                    break
            curr = os.path.dirname(curr)

    franka_fault_manager = ExecuteProcess(
        cmd=['python3', fault_manager_path],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", use_fake_hardware, "' == 'false' and '",
            LaunchConfiguration('supervised_controller_start'), "' == 'false' and '",
            teaching_mode, "' == 'false'"
        ]))
    )

    return LaunchDescription(
        [
         DeclareLaunchArgument('start_cameras', default_value='true', description='Start Realsense cameras'),
         DeclareLaunchArgument('camera_config_file', default_value='handeye_d455_front', description='Camera config file'),
         DeclareLaunchArgument(
             'supervised_controller_start',
             default_value='false',
             description='Load the arm controller inactive so an external supervisor can safely activate it'),
         DeclareLaunchArgument(
             teaching_mode_parameter_name,
             default_value='false',
             description='Keep the arm controller inactive for hand-guided point teaching'),
         load_gripper_arg,
         robot_arg,
         use_fake_hardware_arg,
         fake_sensor_commands_arg,
         db_arg,
         rviz_node,
         robot_state_publisher,
         run_move_group_node,
         ros2_control_node,
         joint_state_publisher,
         franka_robot_state_broadcaster,
         gripper_launch_file,
         camera_launch,
         camera_bridge_tf,
         franka_fault_manager
         # world_frame_publisher
         ]
        + load_controllers
    )
