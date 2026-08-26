#include "franka_teach_executor/executor_node.hpp"
#include "franka_teach_executor/scoped_allowed_collisions.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <random>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fs = std::filesystem;

namespace franka_teach_executor {

namespace {

double resolveOverride(double requested, double fallback) {
  return requested >= 0.0 ? requested : fallback;
}

constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
constexpr const char* kOmplPipelineId = "ompl";
constexpr const char* kPilzPipelineId = "pilz";
constexpr const char* kDefaultOmplPlannerId = "RRTConnectkConfigDefault";
constexpr const char* kDefaultEeFrame = "panda_hand_tcp";
constexpr double kAprilTagSizeM = 0.06;
constexpr std::size_t kHeadlessMarkerSamples = 100;
constexpr std::size_t kHeadlessMarkerMinSamples = 60;
constexpr double kMarkerTranslationInlierFloorM = 0.0015;
constexpr double kMarkerYawInlierFloorRad = 0.5 * 3.14159265358979323846 / 180.0;

bool isPilzPlannerId(const std::string& planner_id) {
  return planner_id == "PTP" || planner_id == "LIN" || planner_id == "CIRC";
}

std::string plannerForPipeline(const std::string& pipeline_id,
                               const std::string& requested_planner_id,
                               const std::string& fallback_planner_id,
                               const std::string& default_planner_id) {
  std::string planner_id = requested_planner_id.empty() ? fallback_planner_id : requested_planner_id;
  if (pipeline_id == kOmplPipelineId && (planner_id.empty() || isPilzPlannerId(planner_id))) {
    return kDefaultOmplPlannerId;
  }
  return planner_id.empty() ? default_planner_id : planner_id;
}

double yawFromRotationMatrix(const Eigen::Matrix3d& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

Eigen::Matrix3d rotationAboutZ(double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

double wrapToPi(double angle) {
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {
    angle -= 2.0 * pi;
  }
  while (angle < -pi) {
    angle += 2.0 * pi;
  }
  return angle;
}

double medianValue(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  const double upper = *mid;
  if (values.size() % 2 == 1) {
    return upper;
  }
  const auto lower_mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2 - 1);
  std::nth_element(values.begin(), lower_mid, values.end());
  return 0.5 * (*lower_mid + upper);
}

double meanYaw(const std::vector<double>& yaws) {
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (double yaw : yaws) {
    sin_sum += std::sin(yaw);
    cos_sum += std::cos(yaw);
  }
  return std::atan2(sin_sum / static_cast<double>(yaws.size()),
                    cos_sum / static_cast<double>(yaws.size()));
}

Eigen::Matrix4d fuseMarkerTransforms(const std::vector<Eigen::Matrix4d>& transforms) {
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  std::vector<double> yaws;
  xs.reserve(transforms.size());
  ys.reserve(transforms.size());
  zs.reserve(transforms.size());
  yaws.reserve(transforms.size());

  for (const auto& transform : transforms) {
    xs.push_back(transform(0, 3));
    ys.push_back(transform(1, 3));
    zs.push_back(transform(2, 3));
    yaws.push_back(yawFromRotationMatrix(transform.block<3, 3>(0, 0)));
  }

  const Eigen::Vector3d median_translation(medianValue(xs), medianValue(ys), medianValue(zs));
  const double yaw_mean = meanYaw(yaws);

  std::vector<double> translation_errors;
  std::vector<double> yaw_errors;
  translation_errors.reserve(transforms.size());
  yaw_errors.reserve(transforms.size());
  for (std::size_t i = 0; i < transforms.size(); ++i) {
    const Eigen::Vector3d translation = transforms.at(i).block<3, 1>(0, 3);
    translation_errors.push_back((translation - median_translation).norm());
    yaw_errors.push_back(std::abs(wrapToPi(yaws.at(i) - yaw_mean)));
  }

  const double median_translation_error = medianValue(translation_errors);
  const double median_yaw_error = medianValue(yaw_errors);
  const double translation_threshold =
      median_translation_error < 1e-9
          ? kMarkerTranslationInlierFloorM
          : std::max(kMarkerTranslationInlierFloorM, 3.0 * median_translation_error);
  const double yaw_threshold =
      median_yaw_error < 1e-9
          ? kMarkerYawInlierFloorRad
          : std::max(kMarkerYawInlierFloorRad, 3.0 * median_yaw_error);

  std::vector<const Eigen::Matrix4d*> inliers;
  for (std::size_t i = 0; i < transforms.size(); ++i) {
    if (translation_errors.at(i) <= translation_threshold &&
        yaw_errors.at(i) <= yaw_threshold) {
      inliers.push_back(&transforms.at(i));
    }
  }
  if (inliers.size() < std::max<std::size_t>(3, std::min(kHeadlessMarkerMinSamples, transforms.size()))) {
    inliers.clear();
    for (const auto& transform : transforms) {
      inliers.push_back(&transform);
    }
  }

  std::vector<double> inlier_xs;
  std::vector<double> inlier_ys;
  std::vector<double> inlier_zs;
  Eigen::Matrix3d rotation_sum = Eigen::Matrix3d::Zero();
  for (const auto* transform : inliers) {
    inlier_xs.push_back((*transform)(0, 3));
    inlier_ys.push_back((*transform)(1, 3));
    inlier_zs.push_back((*transform)(2, 3));
    rotation_sum += transform->block<3, 3>(0, 0);
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotation_sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d mean_rotation = svd.matrixU() * svd.matrixV().transpose();
  if (mean_rotation.determinant() < 0.0) {
    Eigen::Matrix3d u = svd.matrixU();
    u.col(2) *= -1.0;
    mean_rotation = u * svd.matrixV().transpose();
  }

  Eigen::Matrix4d fused = Eigen::Matrix4d::Identity();
  fused.block<3, 3>(0, 0) = mean_rotation;
  fused(0, 3) = medianValue(inlier_xs);
  fused(1, 3) = medianValue(inlier_ys);
  fused(2, 3) = medianValue(inlier_zs);
  return fused;
}

Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose& pose, std::string& error_message) {
  const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                             pose.orientation.z);
  if (q.norm() < 1e-8) {
    error_message = "Target pose has an invalid zero-length quaternion.";
    return Eigen::Matrix4d::Identity();
  }

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  transform(0, 3) = pose.position.x;
  transform(1, 3) = pose.position.y;
  transform(2, 3) = pose.position.z;
  return transform;
}

geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d& transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);

  const Eigen::Quaterniond q(transform.block<3, 3>(0, 0));
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

double quaternionAngularDistance(const geometry_msgs::msg::Quaternion& a,
                                 const geometry_msgs::msg::Quaternion& b) {
  Eigen::Quaterniond qa(a.w, a.x, a.y, a.z);
  Eigen::Quaterniond qb(b.w, b.x, b.y, b.z);
  if (qa.norm() < 1e-8 || qb.norm() < 1e-8) {
    return std::numeric_limits<double>::infinity();
  }

  qa.normalize();
  qb.normalize();
  const double dot = std::clamp(std::abs(qa.dot(qb)), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

}  // namespace

ExecutorNode::ExecutorNode() : rclcpp::Node("franka_teach_executor") {
  planning_group_ = declare_parameter<std::string>("planning_group", "panda_arm");
  joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/franka/joint_states");
  robot_state_topic_ = declare_parameter<std::string>("robot_state_topic", "/franka/robot_state");
  base_frame_ = declare_parameter<std::string>("base_frame", "panda_link0");
  camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_front_link");
  calibration_result_path_ = declare_parameter<std::string>("calibration_result_path", "");
  
  // Try to auto-detect calibration if not provided
  if (calibration_result_path_.empty()) {
    auto storage_root = detectStorageRoot();
    auto auto_calib = storage_root / "latest_calibration.yaml";
    if (std::filesystem::exists(auto_calib)) {
      calibration_result_path_ = auto_calib.string();
      RCLCPP_INFO(get_logger(), "Auto-detected calibration file: %s", calibration_result_path_.c_str());
    }
  }
  error_recovery_action_name_ =
      declare_parameter<std::string>("error_recovery_action", "/action_server/error_recovery");
  enable_robot_state_monitoring_ =
      declare_parameter<bool>("enable_robot_state_monitoring", true);
  joint_state_timeout_sec_ = declare_parameter<double>("joint_state_timeout_sec", 1.0);
  robot_state_timeout_sec_ = declare_parameter<double>("robot_state_timeout_sec", 1.0);
  default_velocity_scale_ = declare_parameter<double>("default_velocity_scale", 0.2);
  default_acceleration_scale_ = declare_parameter<double>("default_acceleration_scale", 0.2);
  default_planning_time_ = declare_parameter<double>("default_planning_time", 5.0);
  default_goal_tolerance_ = declare_parameter<double>("default_goal_tolerance", 0.0001);
  default_orientation_tolerance_ =
      declare_parameter<double>("default_orientation_tolerance", 0.001);
  use_fake_hardware_ = declare_parameter<bool>("use_fake_hardware", false);
  gripper_grasp_action_name_ =
      declare_parameter<std::string>("gripper_grasp_action", "/panda_gripper/grasp");
  gripper_move_action_name_ =
      declare_parameter<std::string>("gripper_move_action", "/panda_gripper/move");
  gripper_stop_service_name_ =
      declare_parameter<std::string>("gripper_stop_service", "/panda_gripper/stop");
  planning_pipeline_id_ = declare_parameter<std::string>("planning_pipeline_id", "");
  planner_id_ = declare_parameter<std::string>("planner_id", "");
  relative_approach_height_ = declare_parameter<double>("relative_approach_height", 0.10);
  
  // Global motion profile parameters
  ptp_velocity_scale_ = declare_parameter<double>("ptp_velocity_scale", 0.6);
  ptp_acceleration_scale_ = declare_parameter<double>("ptp_acceleration_scale", 0.5);
  lin_velocity_scale_ = declare_parameter<double>("lin_velocity_scale", 0.3);
  lin_acceleration_scale_ = declare_parameter<double>("lin_acceleration_scale", 0.2);
  pick_place_velocity_scale_ = declare_parameter<double>("pick_place_velocity_scale", 0.2);
  pick_place_acceleration_scale_ = declare_parameter<double>("pick_place_acceleration_scale", 0.2);
  storage_rack_collision_override_enabled_ =
      declare_parameter<bool>("storage_rack_collision_override_enabled", true);
  storage_rack_collision_robot_links_ = declare_parameter<std::vector<std::string>>(
      "storage_rack_collision_robot_links", {"panda_hand"});
  storage_rack_collision_environment_links_ = declare_parameter<std::vector<std::string>>(
      "storage_rack_collision_environment_links",
      {"wooden_board", "wooden_board_level2", "wooden_board_level3",
       "storage_rack_origin",
       "left_vertical_profile", "right_vertical_profile",
       "left_outward_profile", "right_outward_profile",
       "left_outward_profile_level2", "right_outward_profile_level2",
       "left_outward_profile_level3", "right_outward_profile_level3"});

  const auto storage_dir_param = declare_parameter<std::string>("storage_dir", "");
  storage_root_ = storage_dir_param.empty() ? detectStorageRoot() : fs::path(storage_dir_param);
  declare_parameter<std::string>("workpiece_orientation", "left");
}

void ExecutorNode::initialize() {
  move_group_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);

  // Initialize TF2 listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  loadCalibration();

  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 20,
      std::bind(&ExecutorNode::jointStateCallback, this, std::placeholders::_1));

  if (!robot_state_topic_.empty()) {
    robot_state_subscription_ = create_subscription<franka_msgs::msg::FrankaRobotState>(
        robot_state_topic_, 20,
        std::bind(&ExecutorNode::robotStateCallback, this, std::placeholders::_1));
  }

  error_recovery_client_ = rclcpp_action::create_client<ErrorRecovery>(this, error_recovery_action_name_);
  gripper_grasp_client_ = rclcpp_action::create_client<GripperGrasp>(this, gripper_grasp_action_name_);
  gripper_move_client_ = rclcpp_action::create_client<GripperMove>(this, gripper_move_action_name_);
  gripper_stop_client_ = this->create_client<std_srvs::srv::Trigger>(gripper_stop_service_name_);
  apply_planning_scene_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  get_planning_scene_client_ = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  if (storage_rack_collision_override_enabled_) {
    validateStorageRackCollisionLinks();
    startStorageRackCollisionCleanup();
  } else {
    storage_rack_collision_safety_ready_.store(true);
  }

  go_to_camera_pose_server_ = rclcpp_action::create_server<GoToCameraPose>(
      this, "~/go_to_camera_pose",
      std::bind(&ExecutorNode::handleGoToCameraPoseGoal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ExecutorNode::handleGoToCameraPoseCancel, this, std::placeholders::_1),
      std::bind(&ExecutorNode::handleGoToCameraPoseAccepted, this, std::placeholders::_1));

  play_point_server_ = rclcpp_action::create_server<PlayPoint>(
      this, "~/play_point",
      std::bind(&ExecutorNode::handlePlayPointGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ExecutorNode::handlePlayPointCancel, this, std::placeholders::_1),
      std::bind(&ExecutorNode::handlePlayPointAccepted, this, std::placeholders::_1));

  execute_relative_point_server_ = rclcpp_action::create_server<ExecuteRelativePoint>(
      this, "~/execute_relative_point",
      std::bind(&ExecutorNode::handleExecuteRelativePointGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ExecutorNode::handleExecuteRelativePointCancel, this, std::placeholders::_1),
      std::bind(&ExecutorNode::handleExecuteRelativePointAccepted, this, std::placeholders::_1));

  play_sequence_server_ = rclcpp_action::create_server<PlaySequence>(
      this, "~/play_sequence",
      std::bind(&ExecutorNode::handlePlaySequenceGoal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ExecutorNode::handlePlaySequenceCancel, this, std::placeholders::_1),
      std::bind(&ExecutorNode::handlePlaySequenceAccepted, this, std::placeholders::_1));

  pick_place_server_ = rclcpp_action::create_server<PickPlace>(
      this, "~/pick_place",
      std::bind(&ExecutorNode::handlePickPlaceGoal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ExecutorNode::handlePickPlaceCancel, this, std::placeholders::_1),
      std::bind(&ExecutorNode::handlePickPlaceAccepted, this, std::placeholders::_1));

  pause_service_ = create_service<std_srvs::srv::Trigger>(
      "~/pause", std::bind(&ExecutorNode::handlePause, this, std::placeholders::_1, std::placeholders::_2));
  continue_service_ = create_service<std_srvs::srv::Trigger>(
      "~/continue",
      std::bind(&ExecutorNode::handleContinue, this, std::placeholders::_1, std::placeholders::_2));
  stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", std::bind(&ExecutorNode::handleStop, this, std::placeholders::_1, std::placeholders::_2));
  recover_service_ = create_service<std_srvs::srv::Trigger>(
      "~/recover",
      std::bind(&ExecutorNode::handleRecover, this, std::placeholders::_1, std::placeholders::_2));
  status_service_ = create_service<franka_teach_msgs::srv::GetExecutorStatus>(
      "~/status",
      std::bind(&ExecutorNode::handleStatus, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize Workpiece Detector Config & Topics
  workpiece_config_path_ = ament_index_cpp::get_package_share_directory("iwt_bandlaufsystem") + "/config/task_config.yaml";
  loadWorkpieceConfig();

  workpiece_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera_front/camera_front/color/image_raw", 10,
      std::bind(&ExecutorNode::colorCallback, this, std::placeholders::_1));

  workpiece_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera_front/camera_front/aligned_depth_to_color/image_raw", 10,
      std::bind(&ExecutorNode::depthCallback, this, std::placeholders::_1));

  workpiece_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_front/camera_front/color/camera_info", 10,
      std::bind(&ExecutorNode::cameraInfoCallback, this, std::placeholders::_1));

  workpiece_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_marker_pose", 10);

  // Advertise services with EXACT original names so that task_manager client works out-of-the-box!
  workpiece_trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/workpiece_detector_node/trigger_check",
      std::bind(&ExecutorNode::handleTriggerCheck, this, std::placeholders::_1, std::placeholders::_2));

  for (int i = 0; i < 4; ++i) {
      std::string service_name = "/workpiece_detector_node/clear_slot_" + std::to_string(i + 1);
      workpiece_clear_slot_services_[i] = this->create_service<std_srvs::srv::Trigger>(
          service_name,
          [this, i](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
              this->handleClearSlot(i, request, response);
          });
  }

  workpiece_clear_all_slots_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/workpiece_detector_node/clear_all_slots",
      std::bind(&ExecutorNode::handleClearAllSlots, this, std::placeholders::_1, std::placeholders::_2));

  place_to_avgarea_to_converyerblet_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/place_to_avgarea_to_converyerblet",
      std::bind(&ExecutorNode::handlePlaceToAvgareaToConveryerblet, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "franka_teach_executor ready. Storage root: %s",
              storage_root_.string().c_str());
  RCLCPP_INFO(get_logger(), "Frames: base=%s camera=%s", base_frame_.c_str(), camera_frame_.c_str());
}

rclcpp_action::GoalResponse ExecutorNode::handleGoToCameraPoseGoal(
    const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const GoToCameraPose::Goal> goal) {
  if (goal->target_pose.header.frame_id.empty()) {
    RCLCPP_WARN(get_logger(), "Rejected go_to_camera_pose goal with empty frame_id");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::scoped_lock lock(runtime_mutex_);
  if (runtime_.busy) {
    RCLCPP_WARN(get_logger(), "Rejected go_to_camera_pose goal while executor is busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecutorNode::handleGoToCameraPoseCancel(
    const std::shared_ptr<GoToCameraPoseGoalHandle>& /*goal_handle*/) {
  {
    std::scoped_lock lock(runtime_mutex_);
    runtime_.stop_requested = true;
    runtime_.last_error = "Camera pose goal canceled";
  }
  setState(ExecutorState::STOPPING);
  if (move_group_) {
    move_group_->stop();
  }
  runtime_cv_.notify_all();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecutorNode::handleGoToCameraPoseAccepted(
    const std::shared_ptr<GoToCameraPoseGoalHandle>& goal_handle) {
  std::thread(&ExecutorNode::executeCameraPoseGoal, this, goal_handle).detach();
}

rclcpp_action::GoalResponse ExecutorNode::handlePlayPointGoal(
    const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const PlayPoint::Goal> goal) {
  if (goal->point_name.empty()) {
    RCLCPP_WARN(get_logger(), "Rejected play_point goal with empty point name");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::scoped_lock lock(runtime_mutex_);
  if (runtime_.busy) {
    RCLCPP_WARN(get_logger(), "Rejected play_point goal while executor is busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecutorNode::handlePlayPointCancel(
    const std::shared_ptr<PlayPointGoalHandle>& /*goal_handle*/) {
  {
    std::scoped_lock lock(runtime_mutex_);
    runtime_.stop_requested = true;
    runtime_.last_error = "Play point goal canceled";
  }
  setState(ExecutorState::STOPPING);
  if (move_group_) {
    move_group_->stop();
  }
  runtime_cv_.notify_all();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecutorNode::handlePlayPointAccepted(const std::shared_ptr<PlayPointGoalHandle>& goal_handle) {
  std::thread(&ExecutorNode::executePointGoal, this, goal_handle).detach();
}

rclcpp_action::GoalResponse ExecutorNode::handleExecuteRelativePointGoal(
    const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const ExecuteRelativePoint::Goal> goal) {
  if (goal->point_name.empty()) {
    RCLCPP_WARN(get_logger(), "Rejected execute_relative_point goal with empty point name");
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::scoped_lock lock(runtime_mutex_);
  if (runtime_.busy) {
    RCLCPP_WARN(get_logger(), "Rejected execute_relative_point goal while executor is busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecutorNode::handleExecuteRelativePointCancel(
    const std::shared_ptr<ExecuteRelativePointGoalHandle>& /*goal_handle*/) {
  {
    std::scoped_lock lock(runtime_mutex_);
    runtime_.stop_requested = true;
  }
  setState(ExecutorState::STOPPING);
  if (move_group_) {
    move_group_->stop();
  }
  runtime_cv_.notify_all();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecutorNode::handleExecuteRelativePointAccepted(
    const std::shared_ptr<ExecuteRelativePointGoalHandle>& goal_handle) {
  std::thread(&ExecutorNode::executeRelativePointGoal, this, goal_handle).detach();
}

rclcpp_action::GoalResponse ExecutorNode::handlePlaySequenceGoal(
    const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const PlaySequence::Goal> goal) {
  if (goal->sequence_name.empty()) {
    RCLCPP_WARN(get_logger(), "Rejected play_sequence goal with empty sequence name");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::scoped_lock lock(runtime_mutex_);
  if (runtime_.busy) {
    RCLCPP_WARN(get_logger(), "Rejected play_sequence goal while executor is busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecutorNode::handlePlaySequenceCancel(
    const std::shared_ptr<PlaySequenceGoalHandle>& /*goal_handle*/) {
  {
    std::scoped_lock lock(runtime_mutex_);
    runtime_.stop_requested = true;
    runtime_.last_error = "Play sequence goal canceled";
  }
  setState(ExecutorState::STOPPING);
  if (move_group_) {
    move_group_->stop();
  }
  runtime_cv_.notify_all();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecutorNode::handlePlaySequenceAccepted(const std::shared_ptr<PlaySequenceGoalHandle>& goal_handle) {
  std::thread(&ExecutorNode::executeSequenceGoal, this, goal_handle).detach();
}

rclcpp_action::GoalResponse ExecutorNode::handlePickPlaceGoal(
    const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const PickPlace::Goal> goal) {
  if (goal->point_name.empty()) {
    RCLCPP_WARN(get_logger(), "Rejected pick_place goal with empty point name");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->mode != "pick" && goal->mode != "place") {
    RCLCPP_WARN(get_logger(), "Rejected pick_place goal with invalid mode: %s (expected 'pick' or 'place')", goal->mode.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::scoped_lock lock(runtime_mutex_);
  if (runtime_.busy) {
    RCLCPP_WARN(get_logger(), "Rejected pick_place goal while executor is busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecutorNode::handlePickPlaceCancel(
    const std::shared_ptr<PickPlaceGoalHandle>& /*goal_handle*/) {
  {
    std::scoped_lock lock(runtime_mutex_);
    runtime_.stop_requested = true;
    runtime_.last_error = "Pick/Place goal canceled";
  }
  setState(ExecutorState::STOPPING);
  if (move_group_) {
    move_group_->stop();
  }
  runtime_cv_.notify_all();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecutorNode::handlePickPlaceAccepted(const std::shared_ptr<PickPlaceGoalHandle>& goal_handle) {
  std::thread(&ExecutorNode::executePickPlaceGoal, this, goal_handle).detach();
}

void ExecutorNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::scoped_lock lock(state_sample_mutex_);
  latest_joint_state_.message = *msg;
  latest_joint_state_.received_time = now();
  latest_joint_state_.valid = true;
}

void ExecutorNode::robotStateCallback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg) {
  std::scoped_lock lock(state_sample_mutex_);
  latest_robot_state_.message = *msg;
  latest_robot_state_.received_time = now();
  latest_robot_state_.valid = true;
}

void ExecutorNode::handlePause(const std::shared_ptr<std_srvs::srv::Trigger::Request>& /*request*/,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::scoped_lock lock(runtime_mutex_);
  if (!runtime_.busy) {
    response->success = false;
    response->message = "Executor is idle.";
    return;
  }

  if (state_ == ExecutorState::WAITING_CONTINUE) {
    runtime_.waiting_for_continue = false;
    state_ = ExecutorState::PAUSED;
    response->success = true;
    response->message = "Sequence paused while waiting for continue.";
    runtime_cv_.notify_all();
    return;
  }

  if (state_ == ExecutorState::EXECUTING || state_ == ExecutorState::PLANNING) {
    runtime_.pause_requested = true;
    state_ = ExecutorState::PAUSE_REQUESTED;
    response->success = true;
    response->message = "Pause requested. Executor will pause after the current step.";
    runtime_cv_.notify_all();
    return;
  }

  if (state_ == ExecutorState::PAUSED) {
    response->success = true;
    response->message = "Executor is already paused.";
    return;
  }

  response->success = false;
  response->message = "Pause is not valid in the current state.";
}

void ExecutorNode::handleContinue(const std::shared_ptr<std_srvs::srv::Trigger::Request>& /*request*/,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::scoped_lock lock(runtime_mutex_);
  if (!runtime_.busy) {
    response->success = false;
    response->message = "Executor is idle.";
    return;
  }

  if (state_ == ExecutorState::FAULT_REFLEX || state_ == ExecutorState::RECOVERING) {
    response->success = false;
    response->message = "Recover from reflex before continuing.";
    return;
  }

  if (state_ == ExecutorState::WAITING_CONTINUE || state_ == ExecutorState::PAUSED ||
      state_ == ExecutorState::FAULT_STOPPED) {
    runtime_.continue_requested = true;
    response->success = true;
    response->message = "Continue requested.";
    runtime_cv_.notify_all();
    return;
  }

  response->success = false;
  response->message = "Continue is not valid in the current state.";
}

void ExecutorNode::handleStop(const std::shared_ptr<std_srvs::srv::Trigger::Request>& /*request*/,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  {
    std::scoped_lock lock(runtime_mutex_);
    if (!runtime_.busy) {
      response->success = false;
      response->message = "Executor is idle.";
      return;
    }
    runtime_.stop_requested = true;
    runtime_.last_error = "Execution stopped by service request";
  }
  setState(ExecutorState::STOPPING);
  if (move_group_) {
    move_group_->stop();
  }
  runtime_cv_.notify_all();
  response->success = true;
  response->message = "Stop requested.";
}

void ExecutorNode::handleRecover(const std::shared_ptr<std_srvs::srv::Trigger::Request>& /*request*/,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::scoped_lock lock(runtime_mutex_);
  if (state_ != ExecutorState::FAULT_REFLEX) {
    response->success = false;
    response->message = "Recover is only valid in FAULT_REFLEX.";
    return;
  }
  runtime_.recover_requested = true;
  response->success = true;
  response->message = "Recovery requested.";
  runtime_cv_.notify_all();
}

void ExecutorNode::handleStatus(
    const std::shared_ptr<franka_teach_msgs::srv::GetExecutorStatus::Request>& /*request*/,
    std::shared_ptr<franka_teach_msgs::srv::GetExecutorStatus::Response> response) {
  response->status = makeStatusMessage();
}

fs::path ExecutorNode::pointPath(const std::string& point_name) const {
  return storage_root_ / "points" / (point_name + ".yaml");
}

fs::path ExecutorNode::sequencePath(const std::string& sequence_name) const {
  return storage_root_ / "sequences" / (sequence_name + ".yaml");
}

fs::path ExecutorNode::detectStorageRoot() const {
  try {
    auto share_dir = fs::path(ament_index_cpp::get_package_share_directory("franka_teach_executor"));
    for (auto current = share_dir; current.has_parent_path(); current = current.parent_path()) {
      if (current.filename() == "install") {
        return current.parent_path() / "teach_data";
      }
    }
  } catch (const std::exception&) {
  }

  const char* home = std::getenv("HOME");
  if (home != nullptr) {
    return fs::path(home) / ".ros" / "franka_teach_tools";
  }
  return fs::current_path() / "teach_data";
}

PointData ExecutorNode::loadPoint(const std::string& point_name) const {
  const auto path = pointPath(point_name);
  if (!fs::exists(path)) {
    throw std::runtime_error("Point file does not exist: " + path.string());
  }

  const auto root = YAML::LoadFile(path.string());
  PointData point;
  point.name = root["name"] ? root["name"].as<std::string>() : point_name;
  point.type = root["type"] ? root["type"].as<std::string>() : "point";

  if (point.type == "cartesian_relative") {
    if (!root["relative_position"]) {
      throw std::runtime_error("Cartesian relative point is missing relative_position: " + path.string());
    }
    if (!root["reference_orientation"]) {
      throw std::runtime_error(
          "Cartesian relative point is missing reference_orientation: " + path.string());
    }
    if (!root["marker_yaw_at_teach_rad"]) {
      throw std::runtime_error(
          "Cartesian relative point is missing marker_yaw_at_teach_rad: " + path.string());
    }

    auto position_node = root["relative_position"];
    point.relative_motion_mode =
        root["relative_motion_mode"] ? root["relative_motion_mode"].as<std::string>() : "";
    point.relative_position = Eigen::Vector3d(position_node["x"].as<double>(),
                                              position_node["y"].as<double>(),
                                              position_node["z"].as<double>());
    point.marker_yaw_at_teach_rad = root["marker_yaw_at_teach_rad"].as<double>();

    geometry_msgs::msg::Quaternion reference_orientation;
    auto orientation_node = root["reference_orientation"];
    reference_orientation.x = orientation_node["x"].as<double>();
    reference_orientation.y = orientation_node["y"].as<double>();
    reference_orientation.z = orientation_node["z"].as<double>();
    reference_orientation.w = orientation_node["w"].as<double>();
    point.reference_orientation = reference_orientation;
    point.frame_id = root["frame_id"] ? root["frame_id"].as<std::string>() : "marker";
    return point;
  }

  if (!root["joint_names"] || !root["positions"]) {
    throw std::runtime_error("Point file is missing joint_names or positions: " + path.string());
  }

  point.joint_names = root["joint_names"].as<std::vector<std::string>>();
  point.positions = root["positions"].as<std::vector<double>>();

  if (point.joint_names.size() != point.positions.size()) {
    throw std::runtime_error("Point file has mismatched joint_names and positions: " + path.string());
  }
  if (point.joint_names.empty()) {
    throw std::runtime_error("Point file contains no joints: " + path.string());
  }
  return point;
}

SequenceData ExecutorNode::loadSequence(const std::string& sequence_name) const {
  const auto path = sequencePath(sequence_name);
  if (!fs::exists(path)) {
    throw std::runtime_error("Sequence file does not exist: " + path.string());
  }

  const auto root = YAML::LoadFile(path.string());
  if (!root["steps"] || !root["steps"].IsSequence()) {
    throw std::runtime_error("Sequence file is missing a valid steps list: " + path.string());
  }

  SequenceData sequence;
  sequence.name = root["name"] ? root["name"].as<std::string>() : sequence_name;

  for (const auto& step_node : root["steps"]) {
    SequenceStep step;
    step.index = step_node["index"] ? step_node["index"].as<uint32_t>() : static_cast<uint32_t>(sequence.steps.size() + 1);
    step.type = step_node["type"] ? step_node["type"].as<std::string>() : "move_to_point";

    if (step.type == "gripper") {
      // Parse gripper-specific fields
      if (!step_node["gripper_action"]) {
        throw std::runtime_error("Gripper step is missing gripper_action: " + path.string());
      }
      step.gripper_action = step_node["gripper_action"].as<std::string>();
      if (step.gripper_action != "grasp" && step.gripper_action != "open") {
        throw std::runtime_error(
            "Invalid gripper_action '" + step.gripper_action + "' (expected 'grasp' or 'open'): " + path.string());
      }
      step.gripper_width = step_node["gripper_width"] ? step_node["gripper_width"].as<double>() : 0.08;
      step.gripper_speed = step_node["gripper_speed"] ? step_node["gripper_speed"].as<double>() : 0.1;
      step.gripper_force = step_node["gripper_force"] ? step_node["gripper_force"].as<double>() : 50.0;
      step.gripper_epsilon_inner = step_node["gripper_epsilon_inner"] ? step_node["gripper_epsilon_inner"].as<double>() : 0.005;
      step.gripper_epsilon_outer = step_node["gripper_epsilon_outer"] ? step_node["gripper_epsilon_outer"].as<double>() : 0.005;
      step.point_name = "gripper_" + step.gripper_action;
    } else {
      // Parse move_to_point fields (existing logic)
      if (step_node["point_name"]) {
        step.point_name = step_node["point_name"].as<std::string>();
      }
      if (step_node["point_file"]) {
        step.point_file = step_node["point_file"].as<std::string>();
      }
      if (step.point_name.empty() && !step.point_file.empty()) {
        step.point_name = fs::path(step.point_file).stem().string();
      }
      if (step.point_name.empty()) {
        throw std::runtime_error("Sequence step is missing point_name and point_file: " + path.string());
      }
      if (step_node["advance_mode"]) {
        step.options.advance_mode = step_node["advance_mode"].as<std::string>();
      }
      if (step_node["velocity_scale"]) {
        step.options.velocity_scale = step_node["velocity_scale"].as<double>();
      }
      if (step_node["acceleration_scale"]) {
        step.options.acceleration_scale = step_node["acceleration_scale"].as<double>();
      }
      if (step_node["planning_time"]) {
        step.options.planning_time = step_node["planning_time"].as<double>();
      }
      if (step_node["goal_tolerance"]) {
        step.options.goal_tolerance = step_node["goal_tolerance"].as<double>();
      }
    }
    sequence.steps.push_back(step);
  }

  if (sequence.steps.empty()) {
    throw std::runtime_error("Sequence contains no steps: " + path.string());
  }
  return sequence;
}

StepOptions ExecutorNode::mergeOptions(const StepOptions& step_options,
                                       const RuntimeOverrides& overrides) const {
  StepOptions merged = step_options;
  merged.advance_mode = merged.advance_mode.empty() ? "auto" : merged.advance_mode;
  merged.velocity_scale = resolveOverride(overrides.velocity_scale,
                                          resolveOverride(step_options.velocity_scale, default_velocity_scale_));
  merged.acceleration_scale =
      resolveOverride(overrides.acceleration_scale,
                      resolveOverride(step_options.acceleration_scale, default_acceleration_scale_));
  merged.planning_time =
      resolveOverride(overrides.planning_time,
                      resolveOverride(step_options.planning_time, default_planning_time_));
  merged.goal_tolerance =
      resolveOverride(overrides.goal_tolerance,
                      resolveOverride(step_options.goal_tolerance, default_goal_tolerance_));
  merged.planner_id = step_options.planner_id.empty() ? planner_id_ : step_options.planner_id;
  merged.planning_pipeline_id =
      step_options.planning_pipeline_id.empty() ? planning_pipeline_id_ : step_options.planning_pipeline_id;

  if (merged.planner_id == "ompl" || merged.planner_id == "OMPL") {
    merged.planning_pipeline_id = "ompl";
    merged.planner_id = "";
  }
  return merged;
}

std::vector<double> ExecutorNode::reorderPointToGroup(
    const PointData& point, const std::vector<std::string>& group_joint_names) const {
  std::map<std::string, double> values_by_name;
  for (std::size_t index = 0; index < point.joint_names.size(); ++index) {
    const auto& joint_name = point.joint_names.at(index);
    if (values_by_name.contains(joint_name)) {
      throw std::runtime_error("Point '" + point.name + "' contains duplicate joint name '" + joint_name + "'");
    }
    values_by_name[joint_name] = point.positions.at(index);
  }

  std::vector<double> ordered;
  ordered.reserve(group_joint_names.size());
  for (const auto& joint_name : group_joint_names) {
    const auto it = values_by_name.find(joint_name);
    if (it == values_by_name.end()) {
      throw std::runtime_error("Point '" + point.name + "' is missing required joint '" + joint_name + "'");
    }
    ordered.push_back(it->second);
  }
  return ordered;
}

bool ExecutorNode::isRobotStateFresh() const {
  if (use_fake_hardware_) {
    return true;
  }
  std::scoped_lock lock(state_sample_mutex_);
  if (!latest_robot_state_.valid) {
    return false;
  }
  return (now() - latest_robot_state_.received_time).seconds() <= robot_state_timeout_sec_;
}

bool ExecutorNode::isJointStateFresh() const {
  if (use_fake_hardware_) {
    return true;
  }
  std::scoped_lock lock(state_sample_mutex_);
  if (!latest_joint_state_.valid) {
    return false;
  }
  return (now() - latest_joint_state_.received_time).seconds() <= joint_state_timeout_sec_;
}

bool ExecutorNode::isCurrentStepAtTarget(const std::vector<std::string>& joint_names,
                                         const std::vector<double>& target_positions,
                                         double tolerance) const {
  std::scoped_lock lock(state_sample_mutex_);
  if (!latest_joint_state_.valid) {
    return false;
  }

  std::map<std::string, double> current_positions;
  std::map<std::string, double> current_velocities;
  for (std::size_t index = 0; index < latest_joint_state_.message.name.size(); ++index) {
    const auto& joint_name = latest_joint_state_.message.name.at(index);
    if (index < latest_joint_state_.message.position.size()) {
      current_positions[joint_name] = latest_joint_state_.message.position.at(index);
    }
    if (index < latest_joint_state_.message.velocity.size()) {
      current_velocities[joint_name] = latest_joint_state_.message.velocity.at(index);
    }
  }

  constexpr double kVelocityThreshold = 1e-2;
  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    const auto position_it = current_positions.find(joint_names.at(index));
    if (position_it == current_positions.end()) {
      return false;
    }
    if (std::fabs(position_it->second - target_positions.at(index)) > tolerance) {
      return false;
    }
    const auto velocity_it = current_velocities.find(joint_names.at(index));
    if (velocity_it != current_velocities.end() && std::fabs(velocity_it->second) > kVelocityThreshold) {
      return false;
    }
  }
  return true;
}

bool ExecutorNode::robotModeIsReflex() const {
  if (!enable_robot_state_monitoring_) {
    return false;
  }
  std::scoped_lock lock(state_sample_mutex_);
  return latest_robot_state_.valid &&
         latest_robot_state_.message.robot_mode == franka_msgs::msg::FrankaRobotState::ROBOT_MODE_REFLEX;
}

bool ExecutorNode::robotModeIsStopped() const {
  if (!enable_robot_state_monitoring_) {
    return false;
  }
  std::scoped_lock lock(state_sample_mutex_);
  return latest_robot_state_.valid &&
         latest_robot_state_.message.robot_mode == franka_msgs::msg::FrankaRobotState::ROBOT_MODE_USER_STOPPED;
}

bool ExecutorNode::stopConditionCleared() const {
  if (!enable_robot_state_monitoring_) {
    return true;
  }
  std::scoped_lock lock(state_sample_mutex_);
  if (!latest_robot_state_.valid) {
    return false;
  }
  return latest_robot_state_.message.robot_mode !=
         franka_msgs::msg::FrankaRobotState::ROBOT_MODE_USER_STOPPED;
}

bool ExecutorNode::runRecoveryAction(std::string& error_message) {
  if (!error_recovery_client_->wait_for_action_server(std::chrono::seconds(2))) {
    error_message = "Error recovery action server is not available at " + error_recovery_action_name_;
    return false;
  }

  ErrorRecovery::Goal goal;
  auto goal_future = error_recovery_client_->async_send_goal(goal);
  if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    error_message = "Timed out while sending error recovery goal.";
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    error_message = "Error recovery goal was rejected.";
    return false;
  }

  auto result_future = error_recovery_client_->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    error_message = "Timed out while waiting for error recovery result.";
    return false;
  }

  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    error_message = "Error recovery action did not succeed.";
    return false;
  }
  return true;
}

bool ExecutorNode::transformCameraPoseToBase(const geometry_msgs::msg::PoseStamped& camera_pose,
                                             geometry_msgs::msg::PoseStamped& base_pose,
                                             std::string& error_message) const {
  if (camera_pose.header.frame_id.empty()) {
    error_message = "Camera pose target is missing frame_id.";
    return false;
  }

  try {
    // Transform the pose to the base frame using TF
    // This matches the logic in the working calibration node and correctly 
    // accounts for all frame offsets and timestamps in the TF tree.
    base_pose = tf_buffer_->transform(camera_pose, base_frame_);
    
    RCLCPP_INFO(get_logger(), "Transformed camera pose to base via TF: [%.3f, %.3f, %.3f] (frame: %s)",
                base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z,
                base_pose.header.frame_id.c_str());
    
    return true;
  } catch (const std::exception& e) {
    error_message = std::string("TF Transform failed: ") + e.what();
    return false;
  }
}

bool ExecutorNode::isCurrentPoseAtTarget(const geometry_msgs::msg::Pose& target_pose,
                                         double position_tolerance,
                                         double orientation_tolerance) const {
  const auto current_pose = move_group_->getCurrentPose().pose;
  const double dx = current_pose.position.x - target_pose.position.x;
  const double dy = current_pose.position.y - target_pose.position.y;
  const double dz = current_pose.position.z - target_pose.position.z;
  const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (position_error > position_tolerance) {
    return false;
  }

  return quaternionAngularDistance(current_pose.orientation, target_pose.orientation) <=
         orientation_tolerance;
}

bool ExecutorNode::isNearPoint(const std::string& point_name, double tolerance_rad) const {
  PointData point;
  try {
    point = loadPoint(point_name);
  } catch (...) {
    return false;
  }
  
  std::scoped_lock lock(state_sample_mutex_);
  if (!latest_joint_state_.valid) {
    return false;
  }
  
  std::map<std::string, double> current_positions;
  for (std::size_t index = 0; index < latest_joint_state_.message.name.size(); ++index) {
    const auto& name = latest_joint_state_.message.name.at(index);
    if (index < latest_joint_state_.message.position.size()) {
      current_positions[name] = latest_joint_state_.message.position.at(index);
    }
  }
  
  try {
    const auto group_joint_names = move_group_->getJointNames();
    const auto target_positions = reorderPointToGroup(point, group_joint_names);
    
    if (target_positions.size() != group_joint_names.size()) {
      return false;
    }
    
    for (size_t i = 0; i < group_joint_names.size(); ++i) {
      auto it = current_positions.find(group_joint_names[i]);
      if (it == current_positions.end()) {
        return false;
      }
      if (std::abs(it->second - target_positions[i]) > tolerance_rad) {
        return false;
      }
    }
    return true;
  } catch (...) {
    return false;
  }
}

void ExecutorNode::loadCalibration() {
  if (calibration_result_path_.empty()) {
    RCLCPP_WARN(get_logger(),
                "No calibration_result_path specified. Using identity camera-to-gripper transform.");
    T_ee_camera_ = Eigen::Matrix4d::Identity();
    return;
  }

  if (!fs::exists(calibration_result_path_)) {
    RCLCPP_ERROR(get_logger(), "Calibration file not found: %s", calibration_result_path_.c_str());
    T_ee_camera_ = Eigen::Matrix4d::Identity();
    return;
  }

  try {
    const auto root = YAML::LoadFile(calibration_result_path_);
    const auto matrix_node = root["camera_to_gripper"]["matrix"];
    if (!matrix_node || !matrix_node.IsSequence()) {
      throw std::runtime_error("Missing camera_to_gripper.matrix in calibration file.");
    }

    for (int row = 0; row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        T_ee_camera_(row, column) = matrix_node[row][column].as<double>();
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded hand-eye calibration from: %s",
                calibration_result_path_.c_str());
    RCLCPP_INFO(get_logger(), "T_ee_camera translation: [%.4f, %.4f, %.4f]",
                T_ee_camera_(0, 3), T_ee_camera_(1, 3), T_ee_camera_(2, 3));
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Failed to load calibration: %s", error.what());
    T_ee_camera_ = Eigen::Matrix4d::Identity();
  }
  calibration_loaded_ = true;
}

std::string ExecutorNode::stateToString(ExecutorState state) const {
  switch (state) {
    case ExecutorState::IDLE:
      return "IDLE";
    case ExecutorState::PLANNING:
      return "PLANNING";
    case ExecutorState::EXECUTING:
      return "EXECUTING";
    case ExecutorState::WAITING_CONTINUE:
      return "WAITING_CONTINUE";
    case ExecutorState::PAUSE_REQUESTED:
      return "PAUSE_REQUESTED";
    case ExecutorState::PAUSED:
      return "PAUSED";
    case ExecutorState::FAULT_REFLEX:
      return "FAULT_REFLEX";
    case ExecutorState::FAULT_STOPPED:
      return "FAULT_STOPPED";
    case ExecutorState::RECOVERING:
      return "RECOVERING";
    case ExecutorState::STOPPING:
      return "STOPPING";
    case ExecutorState::SUCCEEDED:
      return "SUCCEEDED";
    case ExecutorState::FAILED:
      return "FAILED";
    case ExecutorState::STOPPED:
      return "STOPPED";
  }
  return "UNKNOWN";
}

franka_teach_msgs::msg::ExecutorStatus ExecutorNode::makeStatusMessage() const {
  franka_teach_msgs::msg::ExecutorStatus status;
  std::scoped_lock lock(runtime_mutex_);
  status.executor_state = stateToString(state_);
  status.active_sequence_name = runtime_.sequence_name;
  status.active_point_name = runtime_.point_name;
  status.active_step_index = static_cast<uint32_t>(runtime_.active_step_index);
  status.total_steps = static_cast<uint32_t>(runtime_.total_steps);
  status.completed_steps = static_cast<uint32_t>(runtime_.completed_steps);
  status.failed_step_index = static_cast<uint32_t>(runtime_.failed_step_index);
  status.failed_point_name = runtime_.failed_point_name;
  status.last_error = runtime_.last_error;
  status.busy = runtime_.busy;
  status.pause_requested = runtime_.pause_requested;
  status.waiting_for_continue = runtime_.waiting_for_continue;
  status.fault_reflex = runtime_.fault_reflex;
  status.fault_stopped = runtime_.fault_stopped;
  status.recover_required = runtime_.recover_required;
  status.stop_requested = runtime_.stop_requested;
  return status;
}

void ExecutorNode::setState(ExecutorState state) {
  std::scoped_lock lock(runtime_mutex_);
  state_ = state;
}

void ExecutorNode::resetRuntime() {
  std::scoped_lock lock(runtime_mutex_);
  runtime_ = RuntimeContext{};
  state_ = ExecutorState::IDLE;
}

void ExecutorNode::setRuntimeError(const std::string& error_message) {
  std::scoped_lock lock(runtime_mutex_);
  runtime_.last_error = error_message;
}

bool ExecutorNode::beginJob(const std::string& sequence_name, const std::string& point_name,
                            std::size_t total_steps) {
  if (!storage_rack_collision_safety_ready_.load()) {
    RCLCPP_ERROR(get_logger(),
                 "Rejected robot job: storage-rack collision safety cleanup is not confirmed.");
    return false;
  }
  std::scoped_lock lock(runtime_mutex_);
  if (runtime_.busy) {
    return false;
  }
  runtime_ = RuntimeContext{};
  runtime_.busy = true;
  runtime_.sequence_name = sequence_name;
  runtime_.point_name = point_name;
  runtime_.total_steps = total_steps;
  state_ = ExecutorState::IDLE;
  return true;
}

void ExecutorNode::validateStorageRackCollisionLinks() const {
  const auto robot_model = move_group_->getRobotModel();
  if (!robot_model) {
    throw std::runtime_error("Cannot validate storage-rack collision links without a robot model.");
  }

  std::unordered_set<std::string> unique_names;
  auto validate = [&](const std::string& name) {
    if (name.empty()) {
      throw std::runtime_error("Storage-rack collision link names must not be empty.");
    }
    if (!unique_names.insert(name).second) {
      throw std::runtime_error("Duplicate storage-rack collision link: " + name);
    }
    if (std::find(storage_rack_collision_environment_links_.begin(),
                  storage_rack_collision_environment_links_.end(), name) !=
        storage_rack_collision_environment_links_.end()) {
      return;
    }
    const auto* link = robot_model->getLinkModel(name);
    if (!link) {
      throw std::runtime_error("Robot collision link is missing from the robot model: " + name);
    }
    if (link->getShapes().empty()) {
      throw std::runtime_error("Storage-rack collision link has no collision geometry: " + name);
    }
  };

  for (const auto& name : storage_rack_collision_robot_links_) {
    validate(name);
  }
  for (const auto& name : storage_rack_collision_environment_links_) {
    validate(name);
  }
}

void ExecutorNode::startStorageRackCollisionCleanup() {
  storage_rack_collision_safety_ready_.store(false);
  bool expected = false;
  if (!storage_rack_collision_cleanup_running_.compare_exchange_strong(expected, true)) {
    return;
  }

  auto self = std::static_pointer_cast<ExecutorNode>(shared_from_this());
  std::thread([self]() {
    while (rclcpp::ok()) {
      if (ScopedAllowedCollisions::setPairsAllowed(
              self->get_logger(), self->get_planning_scene_client_,
              self->apply_planning_scene_client_,
              self->storage_rack_collision_robot_links_,
              self->storage_rack_collision_environment_links_, false,
              std::chrono::seconds(5))) {
        self->storage_rack_collision_safety_ready_.store(true);
        self->storage_rack_collision_cleanup_running_.store(false);
        RCLCPP_INFO(self->get_logger(),
                    "Storage-rack collision safety cleanup verified; executor jobs are enabled.");
        return;
      }
      RCLCPP_ERROR(self->get_logger(),
                   "Storage-rack collision cleanup is not yet verified; robot jobs remain blocked.");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    self->storage_rack_collision_cleanup_running_.store(false);
  }).detach();
}

std::vector<std::string> ExecutorNode::storageRackCollisionLinksForTarget(
    const geometry_msgs::msg::Pose& target_pose) const {
  (void)target_pose;
  // Rack objects live in the planning-scene world, not in Panda's RobotModel,
  // so their transforms cannot be queried through RobotState. Keep the
  // temporary override scoped to the configured rack objects.
  return storage_rack_collision_environment_links_;
}

void ExecutorNode::finishJob() {
  std::scoped_lock lock(runtime_mutex_);
  runtime_.busy = false;
  runtime_.pause_requested = false;
  runtime_.continue_requested = false;
  runtime_.waiting_for_continue = false;
  runtime_.stop_requested = false;
  runtime_.recover_requested = false;
}

ExecutorNode::StepExecutionResult ExecutorNode::waitForStoppedConditionClear(
    const std::string& error_message) {
  const auto loop_sleep = std::chrono::milliseconds(100);

  {
    std::scoped_lock lock(runtime_mutex_);
    runtime_.fault_stopped = true;
    runtime_.fault_reflex = false;
    runtime_.recover_required = false;
    runtime_.waiting_for_continue = false;
    runtime_.last_error = error_message;
  }
  setState(ExecutorState::FAULT_STOPPED);

  while (rclcpp::ok()) {
    {
      std::unique_lock lock(runtime_mutex_);
      if (runtime_.stop_requested) {
        state_ = ExecutorState::STOPPED;
        return StepExecutionResult::STOPPED;
      }
    }

    if (isRobotStateFresh() && stopConditionCleared()) {
      std::unique_lock lock(runtime_mutex_);
      if (runtime_.continue_requested) {
        runtime_.continue_requested = false;
        runtime_.fault_stopped = false;
        state_ = ExecutorState::PAUSED;
        return StepExecutionResult::RETRY;
      }
    }

    std::this_thread::sleep_for(loop_sleep);
  }

  setRuntimeError("ROS shutdown while waiting for the stop condition to clear.");
  setState(ExecutorState::FAILED);
  return StepExecutionResult::FAILED;
}

ExecutorNode::StepExecutionResult ExecutorNode::waitForReflexRecovery() {
  while (rclcpp::ok()) {
    bool should_recover = false;
    {
      std::unique_lock lock(runtime_mutex_);
      runtime_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
        return runtime_.stop_requested || runtime_.recover_requested;
      });
      if (runtime_.stop_requested) {
        state_ = ExecutorState::STOPPED;
        return StepExecutionResult::STOPPED;
      }
      should_recover = runtime_.recover_requested;
      if (should_recover) {
        runtime_.recover_requested = false;
        state_ = ExecutorState::RECOVERING;
      }
    }

    if (!should_recover) {
      continue;
    }

    std::string recovery_error;
    if (!runRecoveryAction(recovery_error)) {
      {
        std::scoped_lock lock(runtime_mutex_);
        runtime_.last_error = recovery_error;
        runtime_.recover_required = true;
        state_ = ExecutorState::FAULT_REFLEX;
      }
      continue;
    }

    {
      std::scoped_lock lock(runtime_mutex_);
      runtime_.fault_reflex = false;
      runtime_.fault_stopped = false;
      runtime_.recover_required = false;
      runtime_.waiting_for_continue = false;
      runtime_.last_error.clear();
      state_ = ExecutorState::PAUSED;
    }

    while (rclcpp::ok()) {
      std::unique_lock lock(runtime_mutex_);
      runtime_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
        return runtime_.stop_requested || runtime_.continue_requested;
      });
      if (runtime_.stop_requested) {
        state_ = ExecutorState::STOPPED;
        return StepExecutionResult::STOPPED;
      }
      if (runtime_.continue_requested) {
        runtime_.continue_requested = false;
        return StepExecutionResult::RETRY;
      }
    }
  }

  setRuntimeError("ROS shutdown while waiting for reflex recovery.");
  setState(ExecutorState::FAILED);
  return StepExecutionResult::FAILED;
}

bool ExecutorNode::executeCameraPoseInternal(
    const geometry_msgs::msg::PoseStamped& target_pose_camera, const StepOptions& step_options,
    bool plan_only, std::size_t step_index, std::size_t total_steps,
    geometry_msgs::msg::PoseStamped& transformed_target_pose) {
  RuntimeOverrides no_overrides;
  const auto merged_options = mergeOptions(step_options, no_overrides);
  const std::string target_name = "camera_pose_target";

  while (rclcpp::ok()) {
    {
      std::scoped_lock lock(runtime_mutex_);
      runtime_.point_name = target_name;
      runtime_.active_step_index = step_index;
      runtime_.total_steps = total_steps;
      if (runtime_.stop_requested) {
        state_ = ExecutorState::STOPPED;
        return false;
      }
    }

    if (!isJointStateFresh()) {
      setRuntimeError("Joint states are not fresh enough to start planning.");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (enable_robot_state_monitoring_ && !use_fake_hardware_ && !isRobotStateFresh()) {
      setRuntimeError("Robot state is not fresh enough to start planning.");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (robotModeIsReflex()) {
      {
        std::scoped_lock lock(runtime_mutex_);
        runtime_.fault_reflex = true;
        runtime_.fault_stopped = false;
        runtime_.recover_required = true;
        runtime_.last_error = "Cannot plan while robot is in REFLEX mode.";
      }
      setState(ExecutorState::FAULT_REFLEX);
      const auto retry_result = waitForReflexRecovery();
      if (retry_result == StepExecutionResult::RETRY) {
        continue;
      }
      return false;
    }

    if (robotModeIsStopped()) {
      const auto retry_result =
          waitForStoppedConditionClear("Cannot plan while robot is in USER_STOPPED mode.");
      if (retry_result == StepExecutionResult::RETRY) {
        continue;
      }
      return false;
    }

    std::string transform_error;
    if (!transformCameraPoseToBase(target_pose_camera, transformed_target_pose, transform_error)) {
      setRuntimeError(transform_error);
      setState(ExecutorState::FAILED);
      return false;
    }

    setState(ExecutorState::PLANNING);
    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningTime(merged_options.planning_time);
    move_group_->setMaxVelocityScalingFactor(merged_options.velocity_scale);
    move_group_->setMaxAccelerationScalingFactor(merged_options.acceleration_scale);
    move_group_->setGoalPositionTolerance(merged_options.goal_tolerance);
    move_group_->setGoalOrientationTolerance(default_orientation_tolerance_);

    if (!merged_options.planning_pipeline_id.empty()) {
      move_group_->setPlanningPipelineId(merged_options.planning_pipeline_id);
    }
    const std::string active_planner_id = plannerForPipeline(
        merged_options.planning_pipeline_id, merged_options.planner_id, planner_id_, "");
    if (!active_planner_id.empty()) {
      move_group_->setPlannerId(active_planner_id);
    }

    RCLCPP_INFO(get_logger(), "Attempting to plan to camera pose target: x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                transformed_target_pose.pose.position.x, transformed_target_pose.pose.position.y, 
                transformed_target_pose.pose.position.z, transformed_target_pose.pose.orientation.x,
                transformed_target_pose.pose.orientation.y, transformed_target_pose.pose.orientation.z,
                transformed_target_pose.pose.orientation.w);

    bool ik_success = move_group_->setJointValueTarget(transformed_target_pose.pose, "panda_hand_tcp");
    if (!ik_success) {
      setRuntimeError("Inverse Kinematics failed to find a valid joint configuration for the target pose.");
      setState(ExecutorState::FAILED);
      return false;
    }
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);
    move_group_->clearPoseTargets();

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      setRuntimeError("MoveIt planning failed for the camera pose target.");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (plan_only) {
      return true;
    }

    setState(ExecutorState::EXECUTING);
    const auto execute_result = move_group_->execute(plan);
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
      if (!isCurrentPoseAtTarget(transformed_target_pose.pose, merged_options.goal_tolerance,
                                 default_orientation_tolerance_)) {
        RCLCPP_WARN(get_logger(),
                    "MoveIt reported success for the camera pose target, but the final tool pose "
                    "was not within tolerance yet.");
      }
      return true;
    }

    {
      std::scoped_lock lock(runtime_mutex_);
      if (runtime_.stop_requested) {
        runtime_.last_error = runtime_.last_error.empty() ? "Execution stopped." : runtime_.last_error;
        state_ = ExecutorState::STOPPED;
        return false;
      }
    }

    if (!isJointStateFresh()) {
      setRuntimeError("Joint state updates went stale during execution.");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (enable_robot_state_monitoring_ && !use_fake_hardware_ && !isRobotStateFresh()) {
      const auto result =
          waitForStoppedConditionClear("Robot state updates went stale during execution.");
      if (result == StepExecutionResult::RETRY) {
        continue;
      }
      return false;
    }

    if (robotModeIsReflex()) {
      {
        std::scoped_lock lock(runtime_mutex_);
        runtime_.fault_reflex = true;
        runtime_.fault_stopped = false;
        runtime_.recover_required = true;
        runtime_.last_error = "Robot entered REFLEX mode.";
      }
      setState(ExecutorState::FAULT_REFLEX);
      const auto result = waitForReflexRecovery();
      if (result == StepExecutionResult::RETRY) {
        continue;
      }
      return false;
    }

    if (robotModeIsStopped()) {
      const auto result = waitForStoppedConditionClear("Robot entered USER_STOPPED mode.");
      if (result == StepExecutionResult::RETRY) {
        continue;
      }
      return false;
    }

    if (isCurrentPoseAtTarget(transformed_target_pose.pose, merged_options.goal_tolerance,
                              default_orientation_tolerance_)) {
      RCLCPP_WARN(
          get_logger(),
          "MoveIt execution for the camera pose target returned a non-success code, but the robot "
          "appears to be at the target. Treating the step as complete.");
      return true;
    }

    setRuntimeError("MoveIt execution failed for the camera pose target.");
    setState(ExecutorState::FAILED);
    return false;
  }

  setRuntimeError("ROS shutdown while executing the camera pose target.");
  setState(ExecutorState::FAILED);
  return false;
}

bool ExecutorNode::executePointInternal(const std::string& point_name, const StepOptions& step_options,
                                        bool plan_only, std::size_t step_index,
                                        std::size_t total_steps) {
  RuntimeOverrides no_overrides;
  const auto merged_options = mergeOptions(step_options, no_overrides);

  while (rclcpp::ok()) {
    {
      std::scoped_lock lock(runtime_mutex_);
      runtime_.point_name = point_name;
      runtime_.active_step_index = step_index;
      runtime_.total_steps = total_steps;
      if (runtime_.stop_requested) {
        state_ = ExecutorState::STOPPED;
        return false;
      }
    }

    if (!isJointStateFresh()) {
      setRuntimeError("Joint states are not fresh enough to start planning.");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (enable_robot_state_monitoring_ && !use_fake_hardware_ && !isRobotStateFresh()) {
      setRuntimeError("Robot state is not fresh enough to start planning.");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (robotModeIsReflex()) {
      {
        std::scoped_lock lock(runtime_mutex_);
        runtime_.fault_reflex = true;
        runtime_.fault_stopped = false;
        runtime_.recover_required = true;
        runtime_.last_error = "Cannot plan while robot is in REFLEX mode.";
      }
      setState(ExecutorState::FAULT_REFLEX);
      auto retry_result = waitForReflexRecovery();
      if (retry_result == StepExecutionResult::RETRY) {
        continue;
      }
      return false;
    }

    if (robotModeIsStopped()) {
      auto retry_result = waitForStoppedConditionClear("Cannot plan while robot is in USER_STOPPED mode.");
      if (retry_result == StepExecutionResult::RETRY) {
        continue;
      }
      return false;
    }

    PointData point;
    try {
      point = loadPoint(point_name);
    } catch (const std::exception& error) {
      setRuntimeError(error.what());
      setState(ExecutorState::FAILED);
      return false;
    }

    const auto group_joint_names = move_group_->getJointNames();
    std::vector<double> target_positions;
    try {
      target_positions = reorderPointToGroup(point, group_joint_names);
    } catch (const std::exception& error) {
      setRuntimeError(error.what());
      setState(ExecutorState::FAILED);
      return false;
    }

    if (merged_options.base_only) {
      std::vector<double> source_positions;
      std::string source_name = "current robot state";
      bool source_loaded = false;

      if (!merged_options.ref_point_name.empty()) {
        try {
          PointData ref_point = loadPoint(merged_options.ref_point_name);
          source_positions = reorderPointToGroup(ref_point, group_joint_names);
          source_name = "reference point '" + merged_options.ref_point_name + "'";
          source_loaded = true;
        } catch (const std::exception& error) {
          RCLCPP_WARN(get_logger(), "Failed to load reference point '%s' for base_only: %s. Falling back to current state.",
                      merged_options.ref_point_name.c_str(), error.what());
        }
      }

      if (!source_loaded) {
        source_positions = move_group_->getCurrentJointValues();
      }

      if (source_positions.size() == target_positions.size() && !target_positions.empty()) {
        RCLCPP_INFO(get_logger(), "base_only enabled: Keeping joints 2-7 from %s and only moving joint 1 to target: %.4f (source: %.4f)",
                    source_name.c_str(), target_positions[0], source_positions[0]);
        for (size_t i = 1; i < target_positions.size(); ++i) {
          target_positions[i] = source_positions[i];
        }
      } else {
        RCLCPP_WARN(get_logger(), "base_only requested but joint vector sizes mismatch or empty: source=%zu, target=%zu",
                    source_positions.size(), target_positions.size());
      }
    }

    setState(ExecutorState::PLANNING);
    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningTime(merged_options.planning_time);
    move_group_->setMaxVelocityScalingFactor(merged_options.velocity_scale);
    move_group_->setMaxAccelerationScalingFactor(merged_options.acceleration_scale);
    move_group_->setGoalJointTolerance(merged_options.goal_tolerance);

    if (!merged_options.planning_pipeline_id.empty()) {
      move_group_->setPlanningPipelineId(merged_options.planning_pipeline_id);
    }
    std::string active_planner_id = plannerForPipeline(
        merged_options.planning_pipeline_id, merged_options.planner_id, planner_id_, "");
    if (!active_planner_id.empty()) {
      move_group_->setPlannerId(active_planner_id);
    }
    if (!move_group_->setJointValueTarget(target_positions)) {
      setRuntimeError("Failed to set MoveIt joint target for point '" + point_name + "'.");
      setState(ExecutorState::FAILED);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      setRuntimeError("MoveIt planning failed for point '" + point_name + "'.");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (plan_only) {
      return true;
    }

    setState(ExecutorState::EXECUTING);
    const auto execute_result = move_group_->execute(plan);
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
      if (!isCurrentStepAtTarget(group_joint_names, target_positions, merged_options.goal_tolerance)) {
        RCLCPP_WARN(
            get_logger(),
            "MoveIt reported success for point '%s', but the final joint state was not within tolerance yet.",
            point_name.c_str());
      }
      return true;
    }

    {
      std::scoped_lock lock(runtime_mutex_);
      if (runtime_.stop_requested) {
        runtime_.last_error = runtime_.last_error.empty() ? "Execution stopped." : runtime_.last_error;
        state_ = ExecutorState::STOPPED;
        return false;
      }
    }

    if (!isJointStateFresh()) {
      setRuntimeError("Joint state updates went stale during execution.");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (enable_robot_state_monitoring_ && !use_fake_hardware_ && !isRobotStateFresh()) {
      const auto result = waitForStoppedConditionClear("Robot state updates went stale during execution.");
      if (result == StepExecutionResult::RETRY) {
        continue;
      }
      return false;
    }

    if (robotModeIsReflex()) {
      {
        std::scoped_lock lock(runtime_mutex_);
        runtime_.fault_reflex = true;
        runtime_.fault_stopped = false;
        runtime_.recover_required = true;
        runtime_.last_error = "Robot entered REFLEX mode.";
      }
      setState(ExecutorState::FAULT_REFLEX);
      const auto result = waitForReflexRecovery();
      if (result == StepExecutionResult::RETRY) {
        continue;
      }
      return false;
    }

    if (robotModeIsStopped()) {
      const auto result = waitForStoppedConditionClear("Robot entered USER_STOPPED mode.");
      if (result == StepExecutionResult::RETRY) {
        continue;
      }
      return false;
    }

    if (isCurrentStepAtTarget(group_joint_names, target_positions, merged_options.goal_tolerance)) {
      RCLCPP_WARN(
          get_logger(),
          "MoveIt execution for point '%s' returned a non-success code, but the robot appears to be at the target. "
          "Treating the step as complete.",
          point_name.c_str());
      return true;
    }

    setRuntimeError("MoveIt execution failed for point '" + point_name + "'.");
    setState(ExecutorState::FAILED);
    return false;
  }

  setRuntimeError("ROS shutdown while executing point.");
  setState(ExecutorState::FAILED);
  return false;
}

bool ExecutorNode::executeGripperStep(const SequenceStep& step, std::size_t step_index,
                                      std::size_t total_steps) {
  {
    std::scoped_lock lock(runtime_mutex_);
    runtime_.point_name = "gripper_" + step.gripper_action;
    runtime_.active_step_index = step_index;
    runtime_.total_steps = total_steps;
    if (runtime_.stop_requested) {
      state_ = ExecutorState::STOPPED;
      return false;
    }
  }

  if (step.gripper_action == "grasp") {
    RCLCPP_INFO(get_logger(), "Executing gripper GRASP (width=%.4f, force=%.1f, speed=%.2f)",
                step.gripper_width, step.gripper_force, step.gripper_speed);

    if (!gripper_grasp_client_->wait_for_action_server(std::chrono::seconds(5))) {
      setRuntimeError("Gripper grasp action server not available at " + gripper_grasp_action_name_);
      setState(ExecutorState::FAILED);
      return false;
    }

    GripperGrasp::Goal goal;
    goal.width = step.gripper_width;
    goal.speed = step.gripper_speed;
    goal.force = step.gripper_force;
    // Use generous epsilons to ensure the hardware maintains force even if object width varies
    goal.epsilon.inner = std::max(step.gripper_epsilon_inner, 0.04);
    goal.epsilon.outer = std::max(step.gripper_epsilon_outer, 0.04);

    setState(ExecutorState::EXECUTING);
    auto goal_future = gripper_grasp_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      setRuntimeError("Timed out sending gripper grasp goal.");
      setState(ExecutorState::FAILED);
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      setRuntimeError("Gripper grasp goal was rejected.");
      setState(ExecutorState::FAILED);
      return false;
    }

    auto result_future = gripper_grasp_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
      setRuntimeError("Timed out waiting for gripper grasp result.");
      setState(ExecutorState::FAILED);
      return false;
    }

    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(get_logger(), "Gripper grasp action did not return SUCCEEDED (may have grasped an object of different size). Treating as success.");
    }
    RCLCPP_INFO(get_logger(), "Gripper grasp step completed.");
    return true;

  } else if (step.gripper_action == "open") {
    RCLCPP_INFO(get_logger(), "Executing gripper OPEN (width=%.4f, speed=%.2f)",
                0.08, step.gripper_speed);

    if (!gripper_move_client_->wait_for_action_server(std::chrono::seconds(5))) {
      setRuntimeError("Gripper move action server not available at " + gripper_move_action_name_);
      setState(ExecutorState::FAILED);
      return false;
    }

    GripperMove::Goal goal;
    goal.width = 0.08;  // fully open
    goal.speed = step.gripper_speed;

    setState(ExecutorState::EXECUTING);
    auto goal_future = gripper_move_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      setRuntimeError("Timed out sending gripper move goal.");
      setState(ExecutorState::FAILED);
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      setRuntimeError("Gripper move goal was rejected.");
      setState(ExecutorState::FAILED);
      return false;
    }

    auto result_future = gripper_move_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
      setRuntimeError("Timed out waiting for gripper move result.");
      setState(ExecutorState::FAILED);
      return false;
    }

    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      setRuntimeError("Gripper open (move) action failed.");
      setState(ExecutorState::FAILED);
      return false;
    }
    RCLCPP_INFO(get_logger(), "Gripper open step completed.");
    return true;
  }

  setRuntimeError("Unknown gripper_action: " + step.gripper_action);
  setState(ExecutorState::FAILED);
  return false;
}

bool ExecutorNode::waitForStepGateOrFault(const std::string& point_name, std::size_t step_index,
                                          std::size_t total_steps) {
  while (rclcpp::ok()) {
    {
      std::unique_lock lock(runtime_mutex_);
      runtime_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
        return runtime_.stop_requested || runtime_.continue_requested;
      });
      if (runtime_.stop_requested) {
        state_ = ExecutorState::STOPPED;
        return false;
      }
      if (runtime_.continue_requested) {
        runtime_.continue_requested = false;
        runtime_.waiting_for_continue = false;
        return true;
      }
    }

    if (enable_robot_state_monitoring_ && !use_fake_hardware_ && !isRobotStateFresh()) {
      setRuntimeError("Robot state updates went stale while waiting.");
      setState(ExecutorState::FAULT_STOPPED);
    }

    if (robotModeIsReflex()) {
      {
        std::scoped_lock lock(runtime_mutex_);
        runtime_.fault_reflex = true;
        runtime_.fault_stopped = false;
        runtime_.recover_required = true;
        runtime_.last_error = "Robot entered REFLEX mode while waiting.";
      }
      setState(ExecutorState::FAULT_REFLEX);
      auto retry_result = waitForReflexRecovery();
      if (retry_result == StepExecutionResult::RETRY) {
        std::scoped_lock lock(runtime_mutex_);
        runtime_.point_name = point_name;
        runtime_.active_step_index = step_index;
        runtime_.total_steps = total_steps;
        state_ = ExecutorState::WAITING_CONTINUE;
        runtime_.waiting_for_continue = true;
      } else {
        return false;
      }
    }

    if (robotModeIsStopped()) {
      auto retry_result = waitForStoppedConditionClear("Robot entered USER_STOPPED mode while waiting.");
      if (retry_result == StepExecutionResult::RETRY) {
        std::scoped_lock lock(runtime_mutex_);
        runtime_.point_name = point_name;
        runtime_.active_step_index = step_index;
        runtime_.total_steps = total_steps;
        state_ = ExecutorState::WAITING_CONTINUE;
        runtime_.waiting_for_continue = true;
      } else {
        return false;
      }
    }
  }
  setRuntimeError("ROS shutdown while waiting for continue.");
  setState(ExecutorState::FAILED);
  return false;
}

void ExecutorNode::executeCameraPoseGoal(
    const std::shared_ptr<GoToCameraPoseGoalHandle>& goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<GoToCameraPose::Result>();

  if (!beginJob("", "camera_pose_target", 1)) {
    result->success = false;
    result->final_state = "FAILED";
    result->target_frame = base_frame_;
    result->error_message = "Executor is already busy.";
    result->final_status = makeStatusMessage();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to abort goal: %s", e.what());
    }
    return;
  }

  StepOptions options;
  options.velocity_scale = goal->velocity_scale;
  options.acceleration_scale = goal->acceleration_scale;
  options.planning_time = goal->planning_time;
  options.goal_tolerance = goal->goal_tolerance;

  geometry_msgs::msg::PoseStamped transformed_target_pose;
  const bool success = executeCameraPoseInternal(goal->target_pose, options, goal->plan_only, 1, 1,
                                                 transformed_target_pose);
  result->transformed_target_pose = transformed_target_pose;

  if (success) {
    {
      std::scoped_lock lock(runtime_mutex_);
      runtime_.completed_steps = 1;
      state_ = ExecutorState::SUCCEEDED;
    }
    fillResultCommon(*result, true, stateToString(ExecutorState::SUCCEEDED), base_frame_);
    finishJob();
    try {
      goal_handle->succeed(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish succeed result: %s", e.what());
    }
  } else if (goal_handle->is_canceling() || makeStatusMessage().stop_requested ||
             makeStatusMessage().executor_state == "STOPPED") {
    fillResultCommon(*result, false, stateToString(ExecutorState::STOPPED), base_frame_);
    finishJob();
    try {
      goal_handle->canceled(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish canceled result: %s", e.what());
    }
  } else {
    if (makeStatusMessage().executor_state != "FAULT_REFLEX" &&
        makeStatusMessage().executor_state != "FAULT_STOPPED") {
      setState(ExecutorState::FAILED);
    }
    fillResultCommon(*result, false, stateToString(state_), base_frame_);
    finishJob();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish abort result: %s", e.what());
    }
  }
}

void ExecutorNode::executeRelativePointGoal(
    const std::shared_ptr<ExecuteRelativePointGoalHandle>& goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ExecuteRelativePoint::Result>();

  if (!beginJob("", goal->point_name, 1)) {
    result->success = false;
    result->error_message = "Executor is already busy.";
    result->final_status = makeStatusMessage();
    finishJob();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to abort goal: %s", e.what());
    }
    return;
  }

  StepOptions options;
  options.velocity_scale = goal->velocity_scale;
  options.acceleration_scale = goal->acceleration_scale;
  options.planning_time = goal->planning_time;
  options.goal_tolerance = goal->goal_tolerance;
  options.approach_height = goal->approach_height;
  options.gripper_width = goal->gripper_width;
  options.gripper_force = goal->gripper_force;
  options.gripper_speed = goal->gripper_speed;
  options.planner_id = goal->planner_id;
  options.planning_pipeline_id = goal->planning_pipeline_id;
  options.skip_observation_pose = goal->skip_observation_pose;

  geometry_msgs::msg::PoseStamped transformed_target_pose;
  bool success = executeRelativePointInternal(goal->point_name, goal->marker_pose, options, 
                                               goal->mode, goal->plan_only, 1, 1, transformed_target_pose);

  if (success) {
    {
      std::scoped_lock lock(runtime_mutex_);
      runtime_.completed_steps = 1;
      state_ = ExecutorState::SUCCEEDED;
    }
    fillResultCommon(*result, true, stateToString(ExecutorState::SUCCEEDED), goal->point_name);
    finishJob();
    try {
      goal_handle->succeed(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish succeed result: %s", e.what());
    }
  } else if (goal_handle->is_canceling() || makeStatusMessage().stop_requested ||
             makeStatusMessage().executor_state == "STOPPED") {
    fillResultCommon(*result, false, stateToString(ExecutorState::STOPPED), goal->point_name);
    finishJob();
    try {
      goal_handle->canceled(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish canceled result: %s", e.what());
    }
  } else {
    if (makeStatusMessage().executor_state != "FAULT_REFLEX" &&
        makeStatusMessage().executor_state != "FAULT_STOPPED") {
      setState(ExecutorState::FAILED);
    }
    fillResultCommon(*result, false, stateToString(state_), goal->point_name);
    finishJob();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish abort result: %s", e.what());
    }
  }
}

bool ExecutorNode::executeRelativePointInternal(const std::string& point_name,
                                                const geometry_msgs::msg::PoseStamped& marker_pose,
                                                const StepOptions& step_options,
                                                const std::string& mode,
                                                bool plan_only,
                                                std::size_t step_index,
                                                std::size_t total_steps,
                                                geometry_msgs::msg::PoseStamped& transformed_target_pose) {
  RuntimeOverrides no_overrides;
  const auto merged_options = mergeOptions(step_options, no_overrides);

  while (rclcpp::ok()) {
    {
      std::scoped_lock lock(runtime_mutex_);
      runtime_.point_name = point_name;
      runtime_.active_step_index = step_index;
      runtime_.total_steps = total_steps;
      if (runtime_.stop_requested) {
        state_ = ExecutorState::STOPPED;
        return false;
      }
    }

    if (point_name == "scan_only") {
      if (!marker_pose.header.frame_id.empty()) {
        cacheMarkerPose(marker_pose);
        RCLCPP_INFO(get_logger(), "Relative Move: Scanned marker pose registered in C++ executor memory (scan-only mode).");
        return true;
      }

      geometry_msgs::msg::PoseStamped scanned_marker_pose;
      std::string scan_error;
      if (captureHeadlessMarkerPose(scanned_marker_pose, scan_error)) {
        cacheMarkerPose(scanned_marker_pose);
        RCLCPP_INFO(get_logger(), "Relative Move: Headless AprilTag pose scanned and cached in C++ executor memory.");
        return true;
      }

      setRuntimeError("Headless scan_only failed: " + scan_error);
      setState(ExecutorState::FAILED);
      return false;
    }

    PointData point;
    try {
      point = loadPoint(point_name);
    } catch (const std::exception& error) {
      setRuntimeError(error.what());
      setState(ExecutorState::FAILED);
      return false;
    }

    if (point.type != "cartesian_relative" || !point.relative_position.has_value() ||
        !point.reference_orientation.has_value()) {
      setRuntimeError("Point '" + point_name + "' is not a Cartesian relative point.");
      setState(ExecutorState::FAILED);
      return false;
    }
    if (point.relative_motion_mode != "marker_yaw_only") {
      setRuntimeError("Point '" + point_name + "' does not use the marker_yaw_only relative mode.");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (merged_options.skip_observation_pose) {
      RCLCPP_INFO(get_logger(), "Relative Move Phase 0: Skipping observation_pose; using supplied marker snapshot.");
    } else {
      // --- Phase 0: Move to observation_pose ---
      RCLCPP_INFO(get_logger(), "Relative Move Phase 0: Moving to observation_pose");
      try {
        PointData obs_point = loadPoint("observation_pose");
        const auto group_joint_names = move_group_->getJointNames();
        std::vector<double> obs_positions = reorderPointToGroup(obs_point, group_joint_names);

        move_group_->setJointValueTarget(obs_positions);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);

        moveit::planning_interface::MoveGroupInterface::Plan obs_plan;
        if (move_group_->plan(obs_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          if (!plan_only) {
            move_group_->execute(obs_plan);
          }
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to plan to observation_pose. Aborting relative move.");
          setRuntimeError("Failed to plan to observation_pose.");
          setState(ExecutorState::FAILED);
          return false;
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(),
                    "Could not load 'observation_pose'. Continuing without automatic move. Error: %s",
                    e.what());
      }
    }

    // Step 1: Get T_base_marker
    geometry_msgs::msg::PoseStamped active_marker_pose;
    if (!marker_pose.header.frame_id.empty()) {
      active_marker_pose = marker_pose;
      last_marker_pose_ = marker_pose;
      has_cached_marker_ = true;
      RCLCPP_INFO(get_logger(), "Relative Move: Scanned marker pose stored in C++ executor memory.");
    } else {
      if (has_cached_marker_) {
        active_marker_pose = last_marker_pose_;
        RCLCPP_INFO(get_logger(), "Relative Move: Reusing cached marker pose from C++ executor memory.");
      } else {
        RCLCPP_ERROR(get_logger(), "Relative Move: Incoming marker pose is empty, and no cached marker exists in executor memory.");
        setRuntimeError("No cached AprilTag pose available in executor memory.");
        setState(ExecutorState::FAILED);
        return false;
      }
    }

    geometry_msgs::msg::PoseStamped marker_pose_base;
    try {
      RCLCPP_INFO(get_logger(),
                  "Relative Move Phase 1: Transforming marker snapshot from '%s' to '%s': pos=[%.4f, %.4f, %.4f], quat=[%.4f, %.4f, %.4f, %.4f]",
                  active_marker_pose.header.frame_id.c_str(), base_frame_.c_str(),
                  active_marker_pose.pose.position.x, active_marker_pose.pose.position.y, active_marker_pose.pose.position.z,
                  active_marker_pose.pose.orientation.x, active_marker_pose.pose.orientation.y,
                  active_marker_pose.pose.orientation.z, active_marker_pose.pose.orientation.w);
      marker_pose_base = tf_buffer_->transform(active_marker_pose, base_frame_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "TF Transform of marker failed: %s", e.what());
      setRuntimeError(std::string("TF Transform of marker failed: ") + e.what());
      setState(ExecutorState::FAILED);
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "Relative Move Phase 1: Marker in '%s': pos=[%.4f, %.4f, %.4f], quat=[%.4f, %.4f, %.4f, %.4f]",
                marker_pose_base.header.frame_id.c_str(),
                marker_pose_base.pose.position.x, marker_pose_base.pose.position.y,
                marker_pose_base.pose.position.z,
                marker_pose_base.pose.orientation.x, marker_pose_base.pose.orientation.y,
                marker_pose_base.pose.orientation.z, marker_pose_base.pose.orientation.w);

    transformed_target_pose.header.frame_id = base_frame_;
    transformed_target_pose.header.stamp = now();

    std::string err;
    Eigen::Matrix4d T_base_marker = poseToMatrix(marker_pose_base.pose, err);
    const double marker_yaw_current = yawFromRotationMatrix(T_base_marker.block<3, 3>(0, 0));

    const Eigen::Vector3d marker_position = T_base_marker.block<3, 1>(0, 3);
    const Eigen::Vector3d target_position =
        marker_position + rotationAboutZ(marker_yaw_current) * point.relative_position.value();

    const auto& reference_orientation_msg = point.reference_orientation.value();
    Eigen::Quaterniond reference_orientation(reference_orientation_msg.w,
                                            reference_orientation_msg.x,
                                            reference_orientation_msg.y,
                                            reference_orientation_msg.z);
    if (reference_orientation.norm() < 1e-8) {
      setRuntimeError("Point '" + point_name + "' has an invalid reference orientation.");
      setState(ExecutorState::FAILED);
      return false;
    }
    reference_orientation.normalize();

    // Apply yaw delta correction: rotate the taught orientation by the delta between current and taught marker yaw
    const double delta_yaw = marker_yaw_current - point.marker_yaw_at_teach_rad;
    const Eigen::Matrix3d yaw_correction = rotationAboutZ(delta_yaw);
    const Eigen::Matrix3d corrected_orientation = yaw_correction * reference_orientation.toRotationMatrix();

    Eigen::Matrix4d T_base_target = Eigen::Matrix4d::Identity();
    T_base_target.block<3, 3>(0, 0) = corrected_orientation;
    T_base_target.block<3, 1>(0, 3) = target_position;
    transformed_target_pose.pose = matrixToPose(T_base_target);

    // Get seed joint positions from the taught point for configuration consistency
    const auto group_joint_names = move_group_->getJointNames();
    std::vector<double> seed_positions;
    try {
        seed_positions = reorderPointToGroup(point, group_joint_names);
    } catch (...) {
        // Fallback to current joints if taught joints are missing/invalid
        seed_positions = move_group_->getCurrentJointValues();
    }

    if (mode == "pick" || mode == "place") {
        return executePickPlaceAtPose(transformed_target_pose.pose, seed_positions, mode,
                                      merged_options, false, plan_only, step_index, total_steps);
    }

    // --- Legacy "move" mode (Phase 1 & 2 only) ---
    // --- Phase 1: Approach (PTP to above target) ---
    Eigen::Matrix4d T_base_approach = T_base_target;
    T_base_approach(2, 3) += resolveOverride(merged_options.approach_height, relative_approach_height_);

    geometry_msgs::msg::PoseStamped approach_pose;
    approach_pose.header = transformed_target_pose.header;
    approach_pose.pose = matrixToPose(T_base_approach);

    const std::string active_approach_planner = plannerForPipeline(
        merged_options.planning_pipeline_id, merged_options.planner_id, planner_id_, "PTP");
    RCLCPP_INFO(get_logger(), "Relative Move Phase 1: %s/%s Approach to z=%.3f (height=%.3f)",
                merged_options.planning_pipeline_id.empty() ? "default" : merged_options.planning_pipeline_id.c_str(),
                active_approach_planner.c_str(), approach_pose.pose.position.z, relative_approach_height_);

    setState(ExecutorState::PLANNING);
    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningTime(merged_options.planning_time);
    move_group_->setMaxVelocityScalingFactor(ptp_velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(ptp_acceleration_scale_);
    move_group_->setGoalPositionTolerance(merged_options.goal_tolerance);
    move_group_->setGoalOrientationTolerance(default_orientation_tolerance_);

    if (!merged_options.planning_pipeline_id.empty()) {
      move_group_->setPlanningPipelineId(merged_options.planning_pipeline_id);
    }
    move_group_->setPlannerId(active_approach_planner);
    move_group_->setEndEffectorLink("panda_hand_tcp");

    if (!move_group_->setJointValueTarget(approach_pose.pose)) {
      setRuntimeError("IK failed for relative approach point (Phase 1).");
      setState(ExecutorState::FAILED);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    auto plan_result = move_group_->plan(approach_plan);
    move_group_->clearPoseTargets();

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      setRuntimeError("MoveIt planning failed for relative approach (Phase 1).");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (!plan_only) {
      setState(ExecutorState::EXECUTING);
      const auto execute_result = move_group_->execute(approach_plan);
      if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
          setRuntimeError("MoveIt execution failed for relative approach (Phase 1).");
          setState(ExecutorState::FAILED);
          return false;
      }
    }

    // --- Phase 2: Descent (LIN to target) ---
    RCLCPP_INFO(get_logger(), "Relative Move Phase 2: LIN Descent to z=%.3f", 
                transformed_target_pose.pose.position.z);

    setState(ExecutorState::PLANNING);
    // If we are planning only, we still need to set the start state to the end of phase 1
    if (plan_only) {
      moveit_msgs::msg::RobotState approach_state;
      approach_state.joint_state.name = approach_plan.trajectory_.joint_trajectory.joint_names;
      approach_state.joint_state.position = approach_plan.trajectory_.joint_trajectory.points.back().positions;
      move_group_->setStartState(approach_state);
    } else {
      move_group_->setStartStateToCurrentState();
    }

    // Pilz LIN can generate high joint accelerations during Cartesian moves.
    // Lower the scaling factors for the descent phase to ensure a safe motion.
    move_group_->setMaxVelocityScalingFactor(std::min(0.30, merged_options.velocity_scale));
    move_group_->setMaxAccelerationScalingFactor(std::min(0.30, merged_options.acceleration_scale));

    move_group_->setPlanningPipelineId(kPilzPipelineId);
    move_group_->setPlannerId("LIN");
    if (!move_group_->setJointValueTarget(transformed_target_pose.pose)) {
      setRuntimeError("IK failed for relative target point (Phase 2).");
      setState(ExecutorState::FAILED);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan descent_plan;
    plan_result = move_group_->plan(descent_plan);
    move_group_->clearPoseTargets();

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      setRuntimeError("MoveIt planning failed for relative descent (Phase 2).");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (plan_only) return true;

    setState(ExecutorState::EXECUTING);
    const auto execute_result = move_group_->execute(descent_plan);
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
      return true;
    }

    // Check if we arrived anyway
    if (isCurrentPoseAtTarget(transformed_target_pose.pose, merged_options.goal_tolerance,
                               default_orientation_tolerance_)) {
      return true;
    }

    setRuntimeError("MoveIt execution failed for relative descent (Phase 2).");
    setState(ExecutorState::FAILED);
    return false;
  }

  return false;
}

void ExecutorNode::executePointGoal(const std::shared_ptr<PlayPointGoalHandle>& goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PlayPoint::Result>();

  if (!beginJob("", goal->point_name, 1)) {
    result->success = false;
    result->final_state = "FAILED";
    result->point_name = goal->point_name;
    result->error_message = "Executor is already busy.";
    result->final_status = makeStatusMessage();
    finishJob();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to abort goal: %s", e.what());
    }
    return;
  }

  StepOptions options;
  options.velocity_scale = goal->velocity_scale;
  options.acceleration_scale = goal->acceleration_scale;
  options.planning_time = goal->planning_time;
  options.goal_tolerance = goal->goal_tolerance;
  options.planner_id = goal->planner_id;
  options.base_only = goal->base_only;
  options.ref_point_name = goal->ref_point_name;

  const bool success = executePointInternal(goal->point_name, options, goal->plan_only, 1, 1);

  if (success) {
    {
      std::scoped_lock lock(runtime_mutex_);
      runtime_.completed_steps = 1;
      state_ = ExecutorState::SUCCEEDED;
    }
    fillResultCommon(*result, true, stateToString(ExecutorState::SUCCEEDED), goal->point_name);
    finishJob();
    try {
      goal_handle->succeed(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish succeed result: %s", e.what());
    }
  } else if (goal_handle->is_canceling() || makeStatusMessage().stop_requested ||
             makeStatusMessage().executor_state == "STOPPED") {
    fillResultCommon(*result, false, stateToString(ExecutorState::STOPPED), goal->point_name);
    finishJob();
    try {
      goal_handle->canceled(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish canceled result: %s", e.what());
    }
  } else {
    if (makeStatusMessage().executor_state != "FAULT_REFLEX" &&
        makeStatusMessage().executor_state != "FAULT_STOPPED") {
      setState(ExecutorState::FAILED);
    }
    fillResultCommon(*result, false, stateToString(state_), goal->point_name);
    finishJob();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish abort result: %s", e.what());
    }
  }
}

void ExecutorNode::executeSequenceGoal(const std::shared_ptr<PlaySequenceGoalHandle>& goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PlaySequence::Result>();

  SequenceData sequence;
  try {
    sequence = loadSequence(goal->sequence_name);
  } catch (const std::exception& error) {
    result->success = false;
    result->final_state = "FAILED";
    result->sequence_name = goal->sequence_name;
    result->error_message = error.what();
    result->final_status = makeStatusMessage();
    finishJob();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to abort goal: %s", e.what());
    }
    return;
  }

  if (!beginJob(sequence.name, "", sequence.steps.size())) {
    result->success = false;
    result->final_state = "FAILED";
    result->sequence_name = sequence.name;
    result->error_message = "Executor is already busy.";
    result->final_status = makeStatusMessage();
    finishJob();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to abort goal: %s", e.what());
    }
    return;
  }

  const std::size_t start_index = goal->start_step_index == 0 ? 1 : goal->start_step_index;
  if (start_index > sequence.steps.size()) {
    setRuntimeError("start_step_index is out of range.");
    setState(ExecutorState::FAILED);
    fillResultCommon(*result, false, stateToString(ExecutorState::FAILED), sequence.name);
    finishJob();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to abort goal: %s", e.what());
    }
    return;
  }

  bool success = true;
  for (std::size_t index = start_index; index <= sequence.steps.size(); ++index) {
    const auto& step = sequence.steps.at(index - 1);
    bool step_success = false;
    if (step.type == "gripper") {
      step_success = executeGripperStep(step, index, sequence.steps.size());
    } else {
      step_success = executePointInternal(step.point_name, step.options, goal->plan_only, index, sequence.steps.size());
    }
    publishFeedback(goal_handle);
    if (!step_success) {
      std::scoped_lock lock(runtime_mutex_);
      runtime_.failed_step_index = index;
      runtime_.failed_point_name = step.point_name;
      success = false;
      break;
    }

    {
      std::scoped_lock lock(runtime_mutex_);
      runtime_.completed_steps = index;
    }

    const bool must_wait = goal->force_wait_each_step || step.options.advance_mode == "wait";
    bool should_pause = false;
    {
      std::scoped_lock lock(runtime_mutex_);
      should_pause = runtime_.pause_requested;
      if (should_pause) {
        runtime_.pause_requested = false;
        state_ = ExecutorState::PAUSED;
      } else if (must_wait) {
        runtime_.waiting_for_continue = true;
        state_ = ExecutorState::WAITING_CONTINUE;
      }
    }

    if (must_wait || should_pause) {
      if (!waitForStepGateOrFault(step.point_name, index, sequence.steps.size())) {
        std::scoped_lock lock(runtime_mutex_);
        if (runtime_.failed_step_index == 0) {
          runtime_.failed_step_index = index;
          runtime_.failed_point_name = step.point_name;
        }
        success = false;
        break;
      }
    }
  }

  if (success) {
    setState(ExecutorState::SUCCEEDED);
    fillResultCommon(*result, true, stateToString(ExecutorState::SUCCEEDED), sequence.name);
    finishJob();
    try {
      goal_handle->succeed(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish succeed result: %s", e.what());
    }
  } else if (goal_handle->is_canceling() || makeStatusMessage().executor_state == "STOPPED") {
    fillResultCommon(*result, false, stateToString(ExecutorState::STOPPED), sequence.name);
    finishJob();
    try {
      goal_handle->canceled(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish canceled result: %s", e.what());
    }
  } else {
    if (makeStatusMessage().executor_state != "FAULT_REFLEX" &&
        makeStatusMessage().executor_state != "FAULT_STOPPED") {
      setState(ExecutorState::FAILED);
    }
    fillResultCommon(*result, false, stateToString(state_), sequence.name);
    finishJob();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish abort result: %s", e.what());
    }
  }
}

void ExecutorNode::executePickPlaceGoal(const std::shared_ptr<PickPlaceGoalHandle>& goal_handle) {
  const auto goal = goal_handle->get_goal();
  
  RCLCPP_INFO(get_logger(), "executePickPlaceGoal received: point_name='%s', mode='%s', velocity_scale=%.3f, "
                            "acceleration_scale=%.3f, gripper_force=%.3f, gripper_speed=%.3f, approach_height=%.3f, lift_off_height=%.3f, gripper_width=%.3f",
              goal->point_name.c_str(), goal->mode.c_str(), goal->velocity_scale, goal->acceleration_scale,
              goal->gripper_force, goal->gripper_speed, goal->approach_height, goal->lift_off_height, goal->gripper_width);

  auto result = std::make_shared<PickPlace::Result>();

  if (!beginJob("", goal->point_name, 1)) {
    result->success = false;
    result->final_state = "FAILED";
    result->point_name = goal->point_name;
    result->error_message = "Executor is already busy.";
    result->final_status = makeStatusMessage();
    finishJob();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to abort goal: %s", e.what());
    }
    return;
  }

  StepOptions options;
  options.velocity_scale = goal->velocity_scale;
  options.acceleration_scale = goal->acceleration_scale;
  options.planning_time = goal->planning_time;
  options.goal_tolerance = goal->goal_tolerance;
  options.approach_height = goal->approach_height;
  options.lift_off_height = goal->lift_off_height;
  options.gripper_width = goal->gripper_width;
  options.gripper_force = goal->gripper_force;
  options.gripper_speed = goal->gripper_speed;
  options.place_descent_force = goal->place_descent_force;
  options.planner_id = goal->planner_id;

  const bool success = executePickPlaceInternal(goal->point_name, goal->mode, options, goal->plan_only, 1, 1);

  if (success) {
    {
      std::scoped_lock lock(runtime_mutex_);
      runtime_.completed_steps = 1;
      state_ = ExecutorState::SUCCEEDED;
    }
    fillResultCommon(*result, true, stateToString(ExecutorState::SUCCEEDED), goal->point_name);
    finishJob();
    try {
      goal_handle->succeed(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish succeed result: %s", e.what());
    }
  } else if (goal_handle->is_canceling() || makeStatusMessage().stop_requested ||
             makeStatusMessage().executor_state == "STOPPED") {
    fillResultCommon(*result, false, stateToString(ExecutorState::STOPPED), goal->point_name);
    finishJob();
    try {
      goal_handle->canceled(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish canceled result: %s", e.what());
    }
  } else {
    if (makeStatusMessage().executor_state != "FAULT_REFLEX" &&
        makeStatusMessage().executor_state != "FAULT_STOPPED") {
      setState(ExecutorState::FAILED);
    }
    fillResultCommon(*result, false, stateToString(state_), goal->point_name);
    result->error_message = runtime_.last_error;
    finishJob();
    try {
      goal_handle->abort(result);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish abort result: %s", e.what());
    }
  }
}

bool ExecutorNode::executePickPlaceInternal(const std::string& point_name, const std::string& mode,
                                           const StepOptions& step_options, bool plan_only,
                                           std::size_t step_index, std::size_t total_steps) {
  // 1. Load point
  PointData point;
  try {
    point = loadPoint(point_name);
  } catch (const std::exception& error) {
    setRuntimeError(error.what());
    setState(ExecutorState::FAILED);
    return false;
  }

  // 2. Determine target joint positions and Cartesian pose
  const auto group_joint_names = move_group_->getJointNames();
  std::vector<double> target_positions;
  try {
    target_positions = reorderPointToGroup(point, group_joint_names);
  } catch (const std::exception& error) {
    setRuntimeError(error.what());
    setState(ExecutorState::FAILED);
    return false;
  }

  // Rack approach IK should use the dedicated taught seed configuration. The
  // rack point itself remains the Cartesian/joint target; this only selects
  // the IK branch and does not add an executed waypoint.
  std::vector<double> ik_seed_positions = target_positions;
  if (mode == "place" && point_name.rfind("storage_rack", 0) == 0 &&
      point_name != "storage_rack_seed") {
    try {
      const PointData seed_point = loadPoint("storage_rack_seed");
      ik_seed_positions = reorderPointToGroup(seed_point, group_joint_names);
      RCLCPP_INFO(get_logger(),
                  "Using taught storage_rack_seed configuration for approach IK of '%s'.",
                  point_name.c_str());
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(),
                  "storage_rack_seed unavailable; using target rack configuration as IK seed: %s",
                  error.what());
    }
  }

  // Use FK to find the target Cartesian pose in Base frame
  moveit::core::RobotState target_state(move_group_->getRobotModel());
  target_state.setJointGroupPositions(planning_group_, target_positions);
  target_state.update();
  const Eigen::Isometry3d target_ee_pose = target_state.getGlobalLinkTransform("panda_hand_tcp");
  geometry_msgs::msg::Pose target_pose = matrixToPose(target_ee_pose.matrix());



  const bool storage_rack_interaction =
      point_name.rfind("storage_rack", 0) == 0 && point_name != "storage_rack_seed";
  return executePickPlaceAtPose(target_pose, ik_seed_positions, mode, step_options,
                                storage_rack_interaction, plan_only, step_index, total_steps);
}

bool ExecutorNode::executePickPlaceAtPose(const geometry_msgs::msg::Pose& target_pose,
                                          const std::vector<double>& seed_joint_positions,
                                          const std::string& mode,
                                          const StepOptions& step_options,
                                          bool allow_storage_rack_hand_contact,
                                          bool plan_only,
                                          std::size_t step_index,
                                          std::size_t total_steps) {
  RuntimeOverrides no_overrides;
  const auto merged_options = mergeOptions(step_options, no_overrides);

  double approach_height = resolveOverride(step_options.approach_height, relative_approach_height_);
  double lift_off_height = step_options.lift_off_height;  // Negative means no lift-off
  std::vector<double> approach_joint_positions;

  // --- Phase 0: Setup (Open gripper for Pick) ---
  if (mode == "pick") {
    RCLCPP_INFO(get_logger(), "PickPlace Phase 0: Pre-opening gripper");
    SequenceStep setup_step;
    setup_step.gripper_action = "open";
    setup_step.gripper_speed = resolveOverride(step_options.gripper_speed, 0.1);
    setup_step.gripper_width = resolveOverride(step_options.gripper_width, 0.08); // Max width for pre-open

    if (!executeGripperStep(setup_step, step_index, total_steps)) {
        return false;
    }
  }

  // --- Phase 1: Approach (PTP to above target) ---
  Eigen::Quaterniond approach_q(target_pose.orientation.w, target_pose.orientation.x,
                                target_pose.orientation.y, target_pose.orientation.z);
  approach_q.normalize();
  Eigen::Vector3d local_z(0.0, 0.0, 1.0);
  Eigen::Vector3d global_z = approach_q * local_z;

  geometry_msgs::msg::Pose approach_pose = target_pose;
  approach_pose.position.x -= approach_height * global_z.x();
  approach_pose.position.y -= approach_height * global_z.y();
  approach_pose.position.z -= approach_height * global_z.z();

  // For PLACE mode: Add lift-off height to approach pose for extra clearance
  if (mode == "place" && lift_off_height > 0.0) {
    approach_pose.position.z += lift_off_height;
    RCLCPP_INFO(get_logger(), "PLACE: Approach pose elevated by lift-off height %.3fm", lift_off_height);
  }

  const std::string active_planner_id = plannerForPipeline(
      merged_options.planning_pipeline_id, step_options.planner_id, planner_id_, "PTP");
  RCLCPP_INFO(get_logger(), "PickPlace Phase 1: %s/%s Approach to z=%.3f (height=%.3f%s)",
              merged_options.planning_pipeline_id.empty() ? "default" : merged_options.planning_pipeline_id.c_str(),
              active_planner_id.c_str(), approach_pose.position.z, approach_height,
              (mode == "place" && lift_off_height > 0.0) ? " + lift-off" : "");

  setState(ExecutorState::PLANNING);
  
  // Seed IK with the provided seed_joint_positions to maintain configuration consistency
  moveit::core::RobotState approach_ik_state(move_group_->getRobotModel());
  approach_ik_state.setJointGroupPositions(planning_group_, seed_joint_positions);
  
  if (!approach_ik_state.setFromIK(move_group_->getRobotModel()->getJointModelGroup(planning_group_), approach_pose, "panda_hand_tcp")) {
    setRuntimeError("IK failed for Pick/Place approach point (Phase 1) using seed joints.");
    setState(ExecutorState::FAILED);
    return false;
  }
  
  move_group_->setStartStateToCurrentState();
  move_group_->setPlanningTime(merged_options.planning_time);
  move_group_->setMaxVelocityScalingFactor(ptp_velocity_scale_);
  move_group_->setMaxAccelerationScalingFactor(ptp_acceleration_scale_);
  move_group_->setGoalPositionTolerance(merged_options.goal_tolerance);
  move_group_->setGoalOrientationTolerance(default_orientation_tolerance_);

  if (!merged_options.planning_pipeline_id.empty()) {
    move_group_->setPlanningPipelineId(merged_options.planning_pipeline_id);
  }
  move_group_->setPlannerId(active_planner_id);
  move_group_->setEndEffectorLink("panda_hand_tcp");

  approach_ik_state.copyJointGroupPositions(planning_group_, approach_joint_positions);
  if (!move_group_->setJointValueTarget(approach_joint_positions)) {
    setRuntimeError("Failed to set joint value target for Pick/Place approach point (Phase 1).");
    setState(ExecutorState::FAILED);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
  if (move_group_->plan(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    setRuntimeError("MoveIt planning failed for Pick/Place approach (Phase 1).");
    setState(ExecutorState::FAILED);
    return false;
  }

  if (!plan_only) {
    setState(ExecutorState::EXECUTING);
    if (move_group_->execute(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        setRuntimeError("MoveIt execution failed for Pick/Place approach (Phase 1).");
        setState(ExecutorState::FAILED);
        return false;
    }
  }

  // --- Phase 1.5: Wrist Rotation (PTP joint-7-only, at approach height) ---
  // If wrist_offset_rad is set, rotate joint 7 in-place before descending.
  // Save original approach joints first so Phase 4.5 can un-rotate cleanly.
  std::vector<double> original_approach_joint_positions = approach_joint_positions;
  if (!plan_only && step_options.wrist_offset_rad != 0.0 && !approach_joint_positions.empty()) {
    RCLCPP_INFO(get_logger(), "PickPlace Phase 1.5: PTP wrist rotation of %.4f rad (%.1f deg) in-place at approach height.",
                step_options.wrist_offset_rad, step_options.wrist_offset_rad * 180.0 / M_PI);

    std::vector<double> rotated_joints = approach_joint_positions;
    double new_j7 = rotated_joints.back() + step_options.wrist_offset_rad;

    // Panda joint 7 hard limits: [-2.8973, +2.8973] rad.
    // If the offset puts us out of bounds, try the equivalent rotation in the other direction.
    // e.g. +pi out of bounds -> try -pi (same physical 180-deg rotation, opposite wind direction).
    const double j7_lower = -2.8973;
    const double j7_upper =  2.8973;
    if (new_j7 > j7_upper) {
      new_j7 -= 2.0 * M_PI;
      RCLCPP_INFO(get_logger(), "Phase 1.5: +offset exceeded j7 upper limit, using -direction (%.4f rad).", new_j7);
    } else if (new_j7 < j7_lower) {
      new_j7 += 2.0 * M_PI;
      RCLCPP_INFO(get_logger(), "Phase 1.5: +offset exceeded j7 lower limit, using +direction (%.4f rad).", new_j7);
    }
    rotated_joints.back() = new_j7;

    setState(ExecutorState::PLANNING);
    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningPipelineId("");
    move_group_->setPlannerId("PTP");
    move_group_->setPlanningTime(merged_options.planning_time);
    move_group_->setMaxVelocityScalingFactor(ptp_velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(ptp_acceleration_scale_);
    move_group_->setGoalJointTolerance(merged_options.goal_tolerance);

    if (!move_group_->setJointValueTarget(rotated_joints)) {
      setRuntimeError("Failed to set joint target for wrist rotation (Phase 1.5). "
                      "Computed j7=" + std::to_string(new_j7) + " rad may still be out of bounds.");
      setState(ExecutorState::FAILED);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan wrist_plan;
    if (move_group_->plan(wrist_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      setRuntimeError("MoveIt planning failed for wrist rotation (Phase 1.5).");
      setState(ExecutorState::FAILED);
      return false;
    }

    setState(ExecutorState::EXECUTING);
    if (move_group_->execute(wrist_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      setRuntimeError("MoveIt execution failed for wrist rotation (Phase 1.5).");
      setState(ExecutorState::FAILED);
      return false;
    }

    // Update approach_joint_positions so Phase 4 retract stays in the rotated pose
    approach_joint_positions = rotated_joints;
    RCLCPP_INFO(get_logger(), "PickPlace Phase 1.5: Wrist rotation complete. Joint 7 now at %.4f rad.", rotated_joints.back());
  }

  // --- Phase 1.8: Loosen Grip (PLACE only) ---
  if (mode == "place" && !plan_only && step_options.place_descent_force > 0.0) {
    RCLCPP_INFO(get_logger(), "PickPlace Phase 1.8: Stopping active gripper squeezing force (triggering compliance) before final LIN descent.");
    
    if (!gripper_stop_client_->wait_for_service(std::chrono::seconds(5))) {
      setRuntimeError("Gripper stop service not available at " + gripper_stop_service_name_);
      setState(ExecutorState::FAILED);
      return false;
    }

    auto stop_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto stop_future = gripper_stop_client_->async_send_request(stop_request);
    
    if (stop_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      setRuntimeError("Timed out calling gripper stop service.");
      setState(ExecutorState::FAILED);
      return false;
    }

    auto stop_response = stop_future.get();
    if (!stop_response->success) {
      setRuntimeError("Gripper stop service returned failure: " + stop_response->message);
      setState(ExecutorState::FAILED);
      return false;
    }
  }

  // Keep the PTP approach fully collision checked. Only the deliberate LIN
  // interaction inside a storage shelf receives a narrow hand-to-rack ACM
  // override, which remains active through the retract to approach height.
  std::unique_ptr<ScopedAllowedCollisions> storage_rack_collision_scope;
  if (allow_storage_rack_hand_contact && storage_rack_collision_override_enabled_) {
    const auto selected_rack_links = storageRackCollisionLinksForTarget(target_pose);
    if (selected_rack_links.empty()) {
      setRuntimeError("Could not select collision links for the target storage-rack shelf.");
      setState(ExecutorState::FAILED);
      return false;
    }
    storage_rack_collision_scope = std::make_unique<ScopedAllowedCollisions>(
        get_logger(), get_planning_scene_client_, apply_planning_scene_client_,
        [this](const std::string& error) {
          setRuntimeError(error);
          setState(ExecutorState::FAILED);
          startStorageRackCollisionCleanup();
        });
    if (!storage_rack_collision_scope->enable(
            storage_rack_collision_robot_links_,
            selected_rack_links)) {
      setRuntimeError("Could not enable the scoped storage-rack hand collision allowance.");
      setState(ExecutorState::FAILED);
      return false;
    }
  }

  if (storage_rack_collision_scope && !storage_rack_collision_scope->refresh()) {
    setRuntimeError("Could not refresh hand-to-rack collision allowance before descent.");
    setState(ExecutorState::FAILED);
    return false;
  }

  // --- Phase 2: Descent (LIN to target or target+lift_off for PLACE) ---
  geometry_msgs::msg::Pose phase2_target = target_pose;
  
  // For PLACE with lift-off: Phase 2 goes to target + lift_off_height
  if (mode == "place" && lift_off_height > 0.0) {
    phase2_target.position.z += lift_off_height;
    RCLCPP_INFO(get_logger(), "PickPlace Phase 2: LIN to lifted target z=%.3f (target + %.3fm lift-off)",
                phase2_target.position.z, lift_off_height);
  } else {
    RCLCPP_INFO(get_logger(), "PickPlace Phase 2: LIN Descent to target z=%.3f", target_pose.position.z);
  }
  
  setState(ExecutorState::PLANNING);
  if (plan_only) {
    moveit_msgs::msg::RobotState approach_state;
    approach_state.joint_state.name = approach_plan.trajectory_.joint_trajectory.joint_names;
    approach_state.joint_state.position = approach_plan.trajectory_.joint_trajectory.points.back().positions;
    move_group_->setStartState(approach_state);
  } else {
    move_group_->setStartStateToCurrentState();
  }

  // Descent remains a Cartesian linear move even if the approach used OMPL.
  move_group_->setPlanningPipelineId(kPilzPipelineId);
  move_group_->setPlannerId("LIN");
  move_group_->setMaxVelocityScalingFactor(merged_options.velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(merged_options.acceleration_scale);

  // If a wrist rotation was applied in Phase 1.5, we must also rotate the Cartesian
  // target_pose orientation so the LIN IK solver finds a solution with the rotated wrist,
  // not the original one (which would fight the Phase 1.5 move).
  geometry_msgs::msg::Pose effective_phase2_target = phase2_target;
  if (step_options.wrist_offset_rad != 0.0) {
    Eigen::Quaterniond q_orig(phase2_target.orientation.w, phase2_target.orientation.x,
                              phase2_target.orientation.y, phase2_target.orientation.z);
    Eigen::AngleAxisd wrist_rot(step_options.wrist_offset_rad, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_rotated = q_orig * wrist_rot;
    effective_phase2_target.orientation.x = q_rotated.x();
    effective_phase2_target.orientation.y = q_rotated.y();
    effective_phase2_target.orientation.z = q_rotated.z();
    effective_phase2_target.orientation.w = q_rotated.w();
    RCLCPP_INFO(get_logger(), "Phase 2: Using rotated Cartesian target orientation to match Phase 1.5 wrist rotation.");
  }

  if (!move_group_->setJointValueTarget(effective_phase2_target)) {
    setRuntimeError("IK failed for Pick/Place Phase 2 target.");
    setState(ExecutorState::FAILED);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan descent_plan;
  if (move_group_->plan(descent_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    setRuntimeError("MoveIt planning failed for Pick/Place Phase 2.");
    setState(ExecutorState::FAILED);
    return false;
  }

  if (!plan_only) {
    setState(ExecutorState::EXECUTING);
    if (move_group_->execute(descent_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        setRuntimeError("MoveIt execution failed for Pick/Place Phase 2.");
        setState(ExecutorState::FAILED);
        return false;
    }
  }

  // --- Phase 2.5: Final Descent (PLACE only, LIN down by lift_off_height to actual target) ---
  if (mode == "place" && lift_off_height > 0.0) {
    RCLCPP_INFO(get_logger(), "PickPlace Phase 2.5: LIN Final Descent %.3fm to target z=%.3f",
                lift_off_height, target_pose.position.z);
    
    setState(ExecutorState::PLANNING);
    if (storage_rack_collision_scope && !storage_rack_collision_scope->refresh()) {
      setRuntimeError("Could not refresh hand-to-rack collision allowance before final descent.");
      setState(ExecutorState::FAILED);
      return false;
    }
    if (plan_only) {
      moveit_msgs::msg::RobotState phase2_state;
      phase2_state.joint_state.name = descent_plan.trajectory_.joint_trajectory.joint_names;
      phase2_state.joint_state.position = descent_plan.trajectory_.joint_trajectory.points.back().positions;
      move_group_->setStartState(phase2_state);
    } else {
      move_group_->setStartStateToCurrentState();
    }

    move_group_->setPlanningPipelineId(kPilzPipelineId);
    move_group_->setPlannerId("LIN");
    move_group_->setMaxVelocityScalingFactor(merged_options.velocity_scale);
    move_group_->setMaxAccelerationScalingFactor(merged_options.acceleration_scale);

    geometry_msgs::msg::Pose effective_target_pose = target_pose;
    if (step_options.wrist_offset_rad != 0.0) {
      Eigen::Quaterniond q_orig(target_pose.orientation.w, target_pose.orientation.x,
                                target_pose.orientation.y, target_pose.orientation.z);
      Eigen::AngleAxisd wrist_rot(step_options.wrist_offset_rad, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond q_rotated = q_orig * wrist_rot;
      effective_target_pose.orientation.x = q_rotated.x();
      effective_target_pose.orientation.y = q_rotated.y();
      effective_target_pose.orientation.z = q_rotated.z();
      effective_target_pose.orientation.w = q_rotated.w();
    }

    if (!move_group_->setJointValueTarget(effective_target_pose)) {
      setRuntimeError("IK failed for PLACE final descent (Phase 2.5).");
      setState(ExecutorState::FAILED);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan final_descent_plan;
    if (move_group_->plan(final_descent_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      setRuntimeError("MoveIt planning failed for PLACE final descent (Phase 2.5).");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (!plan_only) {
      setState(ExecutorState::EXECUTING);
      if (move_group_->execute(final_descent_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        setRuntimeError("MoveIt execution failed for PLACE final descent (Phase 2.5).");
        setState(ExecutorState::FAILED);
        return false;
      }
    }
  }

  // --- Phase 3: Action ---
  if (!plan_only) {
    RCLCPP_INFO(get_logger(), "PickPlace Phase 3: Action '%s'", mode.c_str());
    SequenceStep gripper_step;
    gripper_step.gripper_action = (mode == "pick") ? "grasp" : "open";
    gripper_step.gripper_speed = resolveOverride(step_options.gripper_speed, 0.1);
    gripper_step.gripper_force = resolveOverride(step_options.gripper_force, 50.0);
    
    if (mode == "pick") {
        gripper_step.gripper_width = resolveOverride(step_options.gripper_width, 0.0);
    } else {
        gripper_step.gripper_width = resolveOverride(step_options.gripper_width, 0.08);
    }

    if (!executeGripperStep(gripper_step, step_index, total_steps)) {
        return false;
    }
  }

  // --- Phase 3.5: Lift-Off (PICK only, LIN straight up in global Z) ---
  if (mode == "pick" && lift_off_height > 0.0) {
    RCLCPP_INFO(get_logger(), "PickPlace Phase 3.5: PICK Lift-off %.3fm in global +Z", lift_off_height);
    
    setState(ExecutorState::PLANNING);
    if (plan_only) {
      moveit_msgs::msg::RobotState descent_state;
      descent_state.joint_state.name = descent_plan.trajectory_.joint_trajectory.joint_names;
      descent_state.joint_state.position = descent_plan.trajectory_.joint_trajectory.points.back().positions;
      move_group_->setStartState(descent_state);
    } else {
      move_group_->setStartStateToCurrentState();
    }

    // Get current pose and lift straight up in global Z
    geometry_msgs::msg::PoseStamped current_pose;
    if (plan_only) {
      // Use the target pose as reference for planning
      current_pose.pose = target_pose;
      if (step_options.wrist_offset_rad != 0.0) {
        Eigen::Quaterniond q_orig(target_pose.orientation.w, target_pose.orientation.x,
                                  target_pose.orientation.y, target_pose.orientation.z);
        Eigen::AngleAxisd wrist_rot(step_options.wrist_offset_rad, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q_rotated = q_orig * wrist_rot;
        current_pose.pose.orientation.x = q_rotated.x();
        current_pose.pose.orientation.y = q_rotated.y();
        current_pose.pose.orientation.z = q_rotated.z();
        current_pose.pose.orientation.w = q_rotated.w();
      }
    } else {
      current_pose = move_group_->getCurrentPose();
    }
    
    geometry_msgs::msg::Pose liftoff_pose = current_pose.pose;
    liftoff_pose.position.z += lift_off_height;  // Lift straight up in global Z

    move_group_->setPlanningPipelineId(kPilzPipelineId);
    move_group_->setPlannerId("LIN");
    move_group_->setMaxVelocityScalingFactor(merged_options.velocity_scale);
    move_group_->setMaxAccelerationScalingFactor(merged_options.acceleration_scale);

    if (!move_group_->setJointValueTarget(liftoff_pose)) {
      setRuntimeError("IK failed for PICK lift-off (Phase 3.5).");
      setState(ExecutorState::FAILED);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan liftoff_plan;
    if (move_group_->plan(liftoff_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      setRuntimeError("MoveIt planning failed for PICK lift-off (Phase 3.5).");
      setState(ExecutorState::FAILED);
      return false;
    }

    if (!plan_only) {
      setState(ExecutorState::EXECUTING);
      if (move_group_->execute(liftoff_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        setRuntimeError("MoveIt execution failed for PICK lift-off (Phase 3.5).");
        setState(ExecutorState::FAILED);
        return false;
      }
    }
  }

  // --- Phase 4: Retract (LIN back to approach position) ---
  RCLCPP_INFO(get_logger(), "PickPlace Phase 4: LIN Retract to approach position z=%.3f", approach_pose.position.z);
  
  setState(ExecutorState::PLANNING);

  if (storage_rack_collision_scope && !storage_rack_collision_scope->refresh()) {
    setRuntimeError("Could not refresh hand-to-rack collision allowance before retract.");
    setState(ExecutorState::FAILED);
    return false;
  }

  if (plan_only) {
     // If we are planning, we need to bridge the descent phase
     moveit_msgs::msg::RobotState descent_state;
     descent_state.joint_state.name = descent_plan.trajectory_.joint_trajectory.joint_names;
     descent_state.joint_state.position = descent_plan.trajectory_.joint_trajectory.points.back().positions;
     move_group_->setStartState(descent_state);
  } else {
    move_group_->setStartStateToCurrentState();
  }

  move_group_->setPlanningPipelineId(kPilzPipelineId);
  move_group_->setPlannerId("LIN");
  move_group_->setMaxVelocityScalingFactor(merged_options.velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(merged_options.acceleration_scale);

  if (!move_group_->setJointValueTarget(approach_joint_positions)) {
    setRuntimeError("Failed to set joint target for Pick/Place retract point (Phase 4).");
    setState(ExecutorState::FAILED);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan retract_plan;
  if (move_group_->plan(retract_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    setRuntimeError("MoveIt planning failed for Pick/Place retract (Phase 4).");
    setState(ExecutorState::FAILED);
    return false;
  }

  if (!plan_only) {
    setState(ExecutorState::EXECUTING);
    if (move_group_->execute(retract_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        setRuntimeError("MoveIt execution failed for Pick/Place retract (Phase 4).");
        setState(ExecutorState::FAILED);
        return false;
    }
    RCLCPP_INFO(get_logger(), "PickPlace Phase 4: Retract execution completed. Waiting 0.2s for joint settling...");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(get_logger(), "PickPlace Phase 4: Joint settling complete. Returning success.");
  }

  if (storage_rack_collision_scope && !storage_rack_collision_scope->restore()) {
    setRuntimeError("Failed to restore collision checking after the storage-rack LIN interaction.");
    setState(ExecutorState::FAILED);
    return false;
  }

  RCLCPP_INFO(get_logger(), "PickPlace execution completed successfully. Returning true.");
  return true;
}

void ExecutorNode::loadWorkpieceConfig() {
  workpiece_slot_coords_[0][0] = 5.0;  workpiece_slot_coords_[0][1] = 0.0;
  workpiece_slot_coords_[1][0] = 17.0; workpiece_slot_coords_[1][1] = 0.0;
  workpiece_slot_coords_[2][0] = 29.0; workpiece_slot_coords_[2][1] = 0.0;
  workpiece_slot_coords_[3][0] = 41.0; workpiece_slot_coords_[3][1] = 0.0;

  try {
      if (std::filesystem::exists(workpiece_config_path_)) {
          YAML::Node config = YAML::LoadFile(workpiece_config_path_);
          if (config["tasks"] && config["tasks"]["agv_relative_pick"] && config["tasks"]["agv_relative_pick"]["relative_pick_points"]) {
              auto rpp = config["tasks"]["agv_relative_pick"]["relative_pick_points"];
              for (int s = 1; s <= 4; ++s) {
                  std::string slot_key = "slot_" + std::to_string(s);
                  if (rpp[slot_key]) {
                      auto s_node = rpp[slot_key];
                      if (s_node["x"]) workpiece_slot_coords_[s-1][0] = s_node["x"].as<double>();
                      if (s_node["y"]) workpiece_slot_coords_[s-1][1] = s_node["y"].as<double>();
                  }
              }
              RCLCPP_INFO(get_logger(), "Loaded dynamic slot target coordinates from task_config.yaml!");
          }
      }
  } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Could not load dynamic slot coordinates, using defaults. Error: %s", e.what());
  }
}

void ExecutorNode::colorCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(workpiece_image_mutex_);
  try {
      workpiece_latest_color_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
      workpiece_latest_color_frame_id_ = msg->header.frame_id;
      workpiece_latest_color_stamp_ = rclcpp::Time(msg->header.stamp);
      workpiece_color_received_ = true;
  } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge color exception: %s", e.what());
  }
}

void ExecutorNode::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(workpiece_image_mutex_);
  try {
      workpiece_latest_depth_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
      workpiece_depth_received_ = true;
  } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge depth exception: %s", e.what());
  }
}

void ExecutorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(workpiece_image_mutex_);
  if (!workpiece_intrinsics_loaded_) {
      workpiece_intrinsics_.fx = msg->k[0];
      workpiece_intrinsics_.fy = msg->k[4];
      workpiece_intrinsics_.cx = msg->k[2];
      workpiece_intrinsics_.cy = msg->k[5];
      if (!msg->d.empty()) {
          workpiece_dist_coeffs_ = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
          for (std::size_t i = 0; i < msg->d.size(); ++i) {
              workpiece_dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d.at(i);
          }
      } else {
          workpiece_dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
      }
      workpiece_intrinsics_loaded_ = true;
      RCLCPP_INFO(get_logger(), "Dynamic camera intrinsics loaded: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                  workpiece_intrinsics_.fx, workpiece_intrinsics_.fy, workpiece_intrinsics_.cx, workpiece_intrinsics_.cy);
  }
}

Eigen::Vector4f ExecutorNode::extractTablePlane(const cv::Mat& depth_image, const cv::Mat& table_search_mask) {
  std::vector<Eigen::Vector3f> points;
  points.reserve(10000);
  for (int v = 0; v < depth_image.rows; v += 10) {
      const uint16_t* row = depth_image.ptr<uint16_t>(v);
      for (int u = 0; u < depth_image.cols; u += 10) {
          if (table_search_mask.at<uchar>(v, u) == 0) continue;
          uint16_t raw_d = row[u];
          if (raw_d == 0) continue;
          float Z = static_cast<float>(raw_d) * workpiece_intrinsics_.depth_scale;
          if (Z < 0.1f || Z > 1.5f) continue;
          float X = (static_cast<float>(u) - workpiece_intrinsics_.cx) * Z / workpiece_intrinsics_.fx;
          float Y = (static_cast<float>(v) - workpiece_intrinsics_.cy) * Z / workpiece_intrinsics_.fy;
          points.push_back(Eigen::Vector3f(X, Y, Z));
      }
  }
  if (points.size() < 100) return Eigen::Vector4f(0, 0, 0, 0);

  int best_inlier_count = 0;
  Eigen::Vector4f best_plane(0, 0, 0, 0);
  std::mt19937 rng(1337);
  std::uniform_int_distribution<int> int_dist(0, (int)points.size() - 1);
  for (int i = 0; i < 300; i++) {
      int idx1 = int_dist(rng);
      int idx2 = int_dist(rng);
      int idx3 = int_dist(rng);
      if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) continue;
      Eigen::Vector3f v1 = (points[idx2] - points[idx1]).normalized();
      Eigen::Vector3f v2 = (points[idx3] - points[idx1]).normalized();
      Eigen::Vector3f n = v1.cross(v2).normalized();
      if (n.norm() < 0.001f || std::abs(n.z()) < 0.7f) continue;
      float d = -n.dot(points[idx1]);
      int inliers = 0;
      for (const auto& pt : points) { if (std::abs(pt.dot(n) + d) < 0.015f) inliers++; }
      if (inliers > best_inlier_count) { best_inlier_count = inliers; best_plane = Eigen::Vector4f(n.x(), n.y(), n.z(), d); }
  }
  if (best_plane(2) > 0) best_plane = -best_plane;
  return best_plane;
}

bool ExecutorNode::detectWorkpieceSlotsFromLatestFrames(std::string& status_str) {
  cv::Mat color_mat, depth_mat;
  {
      std::lock_guard<std::mutex> lock(workpiece_image_mutex_);
      if (!workpiece_color_received_ || !workpiece_depth_received_ || !workpiece_intrinsics_loaded_ ||
          workpiece_latest_color_.empty() || workpiece_latest_depth_.empty()) {
          return false;
      }
      color_mat = workpiece_latest_color_.clone();
      depth_mat = workpiece_latest_depth_.clone();
  }

  float workspace_angle = 0.0f;
  cv::Point2f tag_center(0.0f, 0.0f);
  bool tag_found = false;
  float px_to_cm = 1.0f;

  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(color_mat, dictionary, corners, ids, parameters);
  if (!ids.empty()) {
      for (size_t i = 0; i < ids.size(); i++) {
          if (ids[i] == 0) {
              cv::Point2f unit_x = corners[i][1] - corners[i][0];
              workspace_angle = std::atan2(unit_x.y, unit_x.x) * 180.0f / CV_PI;
              tag_center = (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) * 0.25f;
              
              float s0 = cv::norm(corners[i][0] - corners[i][1]);
              float s1 = cv::norm(corners[i][1] - corners[i][2]);
              float s2 = cv::norm(corners[i][2] - corners[i][3]);
              float s3 = cv::norm(corners[i][3] - corners[i][0]);
              float avg_side = (s0 + s1 + s2 + s3) / 4.0f;
              if (avg_side > 6.0f) {
                  px_to_cm = 6.0f / avg_side;
              }
              tag_found = true;
              break;
          }
      }
  }

  if (!tag_found) {
      RCLCPP_DEBUG(get_logger(), "Workpiece slot detection: AprilTag 0 not found yet.");
      return false;
  }

  cv::Mat table_search_mask = cv::Mat::zeros(color_mat.size(), CV_8U);
  std::vector<cv::Point> table_poly(4);
  float theta = -workspace_angle * CV_PI / 180.0f;
  float tx[4] = { 3.20f, -37.17f, -35.57f, 4.39f };
  float ty[4] = { -3.75f, -2.96f, 46.94f, 44.98f };
  for (int k = 0; k < 4; ++k) {
      float local_x = tx[k] / px_to_cm;
      float local_y = ty[k] / px_to_cm;
      float rx = local_x * std::cos(theta) + local_y * std::sin(theta);
      float ry = -local_x * std::sin(theta) + local_y * std::cos(theta);
      table_poly[k] = cv::Point(static_cast<int>(std::round(tag_center.x + rx)), 
                                static_cast<int>(std::round(tag_center.y + ry)));
  }
  std::vector<std::vector<cv::Point>> polys = { table_poly };
  cv::fillPoly(table_search_mask, polys, 255);

  const std::vector<SlotBBox> slots = {
      { -11.52f, -1.05f, 3.59f, 16.97f },
      { -33.34f, -22.35f, 4.13f, 18.20f },
      { -32.51f, -21.21f, 22.73f, 36.07f },
      { -11.99f, -0.85f, 22.42f, 36.81f }
  };

  Eigen::Vector4f plane = extractTablePlane(depth_mat, table_search_mask);
  if (plane.norm() <= 0.001f) {
      RCLCPP_WARN(get_logger(), "Workpiece slot detection failed: Table plane could not be extracted!");
      return false;
  }

  float nx = plane(0), ny = plane(1), nz = plane(2), nd = plane(3);
  cv::Mat table_mask = cv::Mat::zeros(color_mat.size(), CV_8U);
  cv::Mat object_mask = cv::Mat::zeros(color_mat.size(), CV_8U);

  const int stride = 4;
  #pragma omp parallel for collapse(2) schedule(dynamic)
  for (int v = 0; v < color_mat.rows; v += stride) {
      for (int u = 0; u < color_mat.cols; u += stride) {
          if (table_search_mask.at<uchar>(v, u) == 0) continue;
          uint16_t raw_d = depth_mat.at<uint16_t>(v, u);
          if (raw_d == 0) continue;
          float Z = static_cast<float>(raw_d) * workpiece_intrinsics_.depth_scale;
          float X = (static_cast<float>(u) - workpiece_intrinsics_.cx) * Z / workpiece_intrinsics_.fx;
          float Y = (static_cast<float>(v) - workpiece_intrinsics_.cy) * Z / workpiece_intrinsics_.fy;
          float dist = X*nx + Y*ny + Z*nz + nd;
          
          if (std::abs(dist) < 0.015f) {
              for(int dy=0; dy<stride && v+dy<color_mat.rows; dy++) for(int dx=0; dx<stride && u+dx<color_mat.cols; dx++) table_mask.at<uchar>(v+dy, u+dx) = 255;
          } else if (dist > 0.015f && dist < 0.3f) {
              for(int dy=0; dy<stride && v+dy<color_mat.rows; dy++) for(int dx=0; dx<stride && u+dx<color_mat.cols; dx++) object_mask.at<uchar>(v+dy, u+dx) = 255;
          }
      }
  }

  cv::morphologyEx(table_mask, table_mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11)));
  std::vector<std::vector<cv::Point>> t_contours;
  cv::findContours(table_mask, t_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (t_contours.empty()) {
      RCLCPP_WARN(get_logger(), "Workpiece slot detection failed: No table contours found!");
      return false;
  }

  auto largest_t = std::max_element(t_contours.begin(), t_contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) { return cv::contourArea(a) < cv::contourArea(b); });
  cv::Mat final_table_mask = cv::Mat::zeros(color_mat.size(), CV_8U);
  cv::drawContours(final_table_mask, t_contours, (int)std::distance(t_contours.begin(), largest_t), 255, -1);

  cv::Mat o_mask;
  cv::bitwise_and(object_mask, final_table_mask, o_mask);
  cv::morphologyEx(o_mask, o_mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));

  std::vector<std::vector<cv::Point>> o_contours;
  cv::findContours(o_mask, o_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::sort(o_contours.begin(), o_contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) { return cv::contourArea(a) > cv::contourArea(b); });

  std::string slot_status[4] = {"EMPTY", "EMPTY", "EMPTY", "EMPTY"};

  for (size_t i = 0; i < o_contours.size(); i++) {
      if (cv::contourArea(o_contours[i]) < 1200) continue;

      cv::Rect bbox = cv::boundingRect(o_contours[i]);
      cv::Point2f obj_center(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
      
      cv::Point2f rel_v = obj_center - tag_center;
      float local_x = rel_v.x * std::cos(theta) - rel_v.y * std::sin(theta);
      float local_y = rel_v.x * std::sin(theta) + rel_v.y * std::cos(theta);
      float local_x_cm = local_x * px_to_cm;
      float local_y_cm = local_y * px_to_cm;

      int slot_idx = -1;
      for (int s = 0; s < 4; ++s) {
          if (local_x_cm >= slots[s].min_x && local_x_cm <= slots[s].max_x &&
              local_y_cm >= slots[s].min_y && local_y_cm <= slots[s].max_y) {
              slot_idx = s;
              break;
          }
      }

      if (slot_idx < 0 || slot_idx >= 4) continue;

      float norm_angle = workspace_angle;
      while (norm_angle > 45.0f) norm_angle -= 90.0f;
      while (norm_angle < -45.0f) norm_angle += 90.0f;

      int pad_w = bbox.width / 2 + 15;
      int pad_h = bbox.height / 2 + 15;
      int safe_x = std::max(0, static_cast<int>(obj_center.x) - pad_w);
      int safe_y = std::max(0, static_cast<int>(obj_center.y) - pad_h);
      int safe_w = std::min(color_mat.cols - safe_x, pad_w * 2);
      int safe_h = std::min(color_mat.rows - safe_y, pad_h * 2);

      cv::Mat roi = color_mat(cv::Rect(safe_x, safe_y, safe_w, safe_h));
      cv::Mat rotated_roi;
      cv::Point2f center(roi.cols / 2.0f, roi.rows / 2.0f);
      cv::Mat rot_mat = cv::getRotationMatrix2D(center, -norm_angle, 1.0);
      cv::warpAffine(roi, rotated_roi, rot_mat, roi.size());

      int patch_w = bbox.width + 10;
      int patch_h = bbox.height + 10;
      int px = std::max(0, static_cast<int>(center.x) - patch_w / 2);
      int py = std::max(0, static_cast<int>(center.y) - patch_h / 2);
      int pw = std::min(rotated_roi.cols - px, patch_w);
      int ph = std::min(rotated_roi.rows - py, patch_h);
      cv::Mat patch = rotated_roi(cv::Rect(px, py, pw, ph));

      cv::Mat gray, thresh;
      cv::cvtColor(patch, gray, cv::COLOR_BGR2GRAY);
      cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
      if (cv::countNonZero(thresh == 255) > cv::countNonZero(thresh == 0)) cv::bitwise_not(thresh, thresh);

      cv::Mat left_half = thresh(cv::Rect(0, 0, thresh.cols/2, thresh.rows));
      cv::Mat right_half = thresh(cv::Rect(thresh.cols/2, 0, thresh.cols/2, thresh.rows));
      std::string side = (cv::mean(left_half)[0] > cv::mean(right_half)[0]) ? "LEFT" : "RIGHT";

      slot_status[slot_idx] = side;
  }

  {
      std::lock_guard<std::mutex> lock(workpiece_status_mutex_);
      for (int s = 0; s < 4; ++s) {
          persistent_slot_status_[s] = slot_status[s];
      }
      latest_packaged_status_ = "1:" + persistent_slot_status_[0] + ";2:" + persistent_slot_status_[1] + ";3:" + persistent_slot_status_[2] + ";4:" + persistent_slot_status_[3];
      workpiece_tag_currently_detected_ = true;
      workpiece_last_detection_time_ = std::chrono::steady_clock::now();
      status_str = latest_packaged_status_;
  }

  return true;
}

bool ExecutorNode::moveToWorkpieceScanPose(std::string& target_point, std::string& error_message) {
  target_point = "orientation_pose";
  if (!std::filesystem::exists(storage_root_ / "points" / "orientation_pose.yaml") &&
      std::filesystem::exists(storage_root_ / "points" / "orientation_check.yaml")) {
      target_point = "orientation_check";
  }

  bool already_there = false;
  try {
      PointData point = loadPoint(target_point);
      const auto group_joint_names = move_group_->getJointNames();
      std::vector<double> target_positions = reorderPointToGroup(point, group_joint_names);

      std::map<std::string, double> current_positions;
      {
          std::scoped_lock lock(state_sample_mutex_);
          if (latest_joint_state_.valid) {
              for (std::size_t index = 0; index < latest_joint_state_.message.name.size(); ++index) {
                  const auto& joint_name = latest_joint_state_.message.name.at(index);
                  if (index < latest_joint_state_.message.position.size()) {
                      current_positions[joint_name] = latest_joint_state_.message.position.at(index);
                  }
              }
          }
      }

      if (!current_positions.empty()) {
          already_there = true;
          for (std::size_t index = 0; index < group_joint_names.size(); ++index) {
              const auto it = current_positions.find(group_joint_names.at(index));
              if (it == current_positions.end() || std::fabs(it->second - target_positions.at(index)) > 0.05) {
                  already_there = false;
                  break;
              }
          }
          if (already_there) {
              RCLCPP_INFO(get_logger(),
                          "Workpiece Check: Robot is already at '%s' (within 0.05 rad position-only).",
                          target_point.c_str());
          }
      }
  } catch (const std::exception& error) {
      RCLCPP_DEBUG(get_logger(),
                   "Workpiece Check: Could not pre-check target point '%s': %s",
                   target_point.c_str(), error.what());
  }

  if (!already_there) {
      StepOptions options;
      if (!executePointInternal(target_point, options, false, 1, 1)) {
          error_message = "Could not automatically move robot to '" + target_point + "'.";
          return false;
      }
  }

  return true;
}

bool ExecutorNode::estimateMarkerTransformCamera(const cv::Mat& color_mat,
                                                 const cv::Mat& camera_matrix,
                                                 const cv::Mat& dist_coeffs,
                                                 Eigen::Matrix4d& target_to_camera,
                                                 int& tag_id,
                                                 std::string& error_message) const {
  if (color_mat.empty() || camera_matrix.empty()) {
      error_message = "Missing color image or camera intrinsics.";
      return false;
  }

  cv::Mat gray_clean;
  cv::cvtColor(color_mat, gray_clean, cv::COLOR_BGR2GRAY);
  cv::Mat gray_detect;
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
  clahe->apply(gray_clean, gray_detect);

  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(gray_detect, dictionary, corners, ids, parameters);
  if (ids.empty()) {
      error_message = "No AprilTag detected.";
      return false;
  }

  int selected_index = 0;
  for (std::size_t i = 0; i < ids.size(); ++i) {
      if (ids.at(i) == 0) {
          selected_index = static_cast<int>(i);
          break;
      }
  }
  tag_id = ids.at(static_cast<std::size_t>(selected_index));

  std::vector<cv::Point2f> image_points = corners.at(static_cast<std::size_t>(selected_index));
  cv::cornerSubPix(gray_clean, image_points, cv::Size(5, 5), cv::Size(-1, -1),
                   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 1e-3));

  const float half_size = static_cast<float>(kAprilTagSizeM * 0.5);
  const std::vector<cv::Point3f> object_points = {
      {-half_size,  half_size, 0.0f},
      { half_size,  half_size, 0.0f},
      { half_size, -half_size, 0.0f},
      {-half_size, -half_size, 0.0f},
  };

  std::vector<int> solve_flags = {cv::SOLVEPNP_IPPE_SQUARE, cv::SOLVEPNP_ITERATIVE};
  bool found_solution = false;
  double best_error = std::numeric_limits<double>::infinity();
  cv::Mat best_rvec;
  cv::Mat best_tvec;

  for (int flag : solve_flags) {
      cv::Mat rvec;
      cv::Mat tvec;
      const bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs,
                                   rvec, tvec, false, flag);
      if (!ok) {
          continue;
      }

      std::vector<cv::Point2f> projected_points;
      cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

      double reprojection_error = 0.0;
      for (std::size_t i = 0; i < projected_points.size(); ++i) {
          reprojection_error += cv::norm(projected_points.at(i) - image_points.at(i));
      }
      reprojection_error /= static_cast<double>(projected_points.size());

      if (reprojection_error < best_error) {
          best_error = reprojection_error;
          best_rvec = rvec.clone();
          best_tvec = tvec.clone();
          found_solution = true;
      }
  }

  if (!found_solution) {
      std::vector<std::vector<cv::Point2f>> marker_corners = {image_points};
      std::vector<cv::Vec3d> rvecs;
      std::vector<cv::Vec3d> tvecs;
      cv::aruco::estimatePoseSingleMarkers(marker_corners, kAprilTagSizeM, camera_matrix,
                                           dist_coeffs, rvecs, tvecs);
      if (rvecs.empty() || tvecs.empty()) {
          error_message = "AprilTag pose estimation failed.";
          return false;
      }
      best_rvec = (cv::Mat_<double>(3, 1) << rvecs[0][0], rvecs[0][1], rvecs[0][2]);
      best_tvec = (cv::Mat_<double>(3, 1) << tvecs[0][0], tvecs[0][1], tvecs[0][2]);
  }

  cv::Mat rotation_cv;
  cv::Rodrigues(best_rvec, rotation_cv);

  target_to_camera = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
          target_to_camera(row, col) = rotation_cv.at<double>(row, col);
      }
      target_to_camera(row, 3) = best_tvec.at<double>(row, 0);
  }

  return true;
}

bool ExecutorNode::getCurrentBaseToEe(Eigen::Matrix4d& base_to_ee, std::string& error_message) {
  {
      std::scoped_lock lock(state_sample_mutex_);
      if (latest_robot_state_.valid &&
          (now() - latest_robot_state_.received_time).seconds() <= robot_state_timeout_sec_) {
          base_to_ee = Eigen::Matrix4d::Identity();
          for (int row = 0; row < 4; ++row) {
              for (int col = 0; col < 4; ++col) {
                  base_to_ee(row, col) = latest_robot_state_.message.o_t_ee[row + 4 * col];
              }
          }
          return true;
      }
  }

  try {
      const auto current_pose = move_group_->getCurrentPose(kDefaultEeFrame).pose;
      std::string pose_error;
      base_to_ee = poseToMatrix(current_pose, pose_error);
      if (!pose_error.empty()) {
          error_message = pose_error;
          return false;
      }
      return true;
  } catch (const std::exception& error) {
      error_message = std::string("Unable to determine current EE pose: ") + error.what();
      return false;
  }
}

bool ExecutorNode::captureHeadlessMarkerPose(geometry_msgs::msg::PoseStamped& marker_pose,
                                             std::string& error_message) {
  if (!calibration_loaded_) {
      error_message = "Hand-eye calibration is not loaded.";
      return false;
  }

  std::vector<Eigen::Matrix4d> target_to_camera_samples;
  target_to_camera_samples.reserve(kHeadlessMarkerSamples);
  int64_t last_image_stamp_ns = -1;
  int last_tag_id = -1;
  std::string last_detection_error;

  const auto start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
      if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(30)) {
          break;
      }

      cv::Mat color_mat;
      cv::Mat camera_matrix;
      cv::Mat dist_coeffs;
      int64_t image_stamp_ns = 0;
      {
          std::lock_guard<std::mutex> lock(workpiece_image_mutex_);
          if (!workpiece_color_received_ || !workpiece_intrinsics_loaded_ ||
              workpiece_latest_color_.empty()) {
              // Camera startup can lag behind motion completion.
              std::this_thread::sleep_for(std::chrono::milliseconds(20));
              continue;
          }

          color_mat = workpiece_latest_color_.clone();
          image_stamp_ns = workpiece_latest_color_stamp_.nanoseconds();
          camera_matrix = (cv::Mat_<double>(3, 3) <<
              workpiece_intrinsics_.fx, 0.0, workpiece_intrinsics_.cx,
              0.0, workpiece_intrinsics_.fy, workpiece_intrinsics_.cy,
              0.0, 0.0, 1.0);
          dist_coeffs = workpiece_dist_coeffs_.empty()
              ? cv::Mat::zeros(1, 5, CV_64F)
              : workpiece_dist_coeffs_.clone();
      }

      if (image_stamp_ns == last_image_stamp_ns) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
          continue;
      }
      last_image_stamp_ns = image_stamp_ns;

      Eigen::Matrix4d target_to_camera;
      int tag_id = -1;
      std::string detection_error;
      if (estimateMarkerTransformCamera(color_mat, camera_matrix, dist_coeffs,
                                        target_to_camera, tag_id, detection_error)) {
          target_to_camera_samples.push_back(target_to_camera);
          last_tag_id = tag_id;
          if (target_to_camera_samples.size() >= kHeadlessMarkerSamples) {
              break;
          }
      } else {
          last_detection_error = detection_error;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (target_to_camera_samples.size() < kHeadlessMarkerMinSamples) {
      error_message = "Only collected " + std::to_string(target_to_camera_samples.size()) +
                      " AprilTag sample(s). Last error: " + last_detection_error;
      return false;
  }

  Eigen::Matrix4d base_to_ee;
  if (!getCurrentBaseToEe(base_to_ee, error_message)) {
      return false;
  }

  const Eigen::Matrix4d target_to_camera = fuseMarkerTransforms(target_to_camera_samples);
  const Eigen::Matrix4d base_to_marker = base_to_ee * T_ee_camera_ * target_to_camera;

  marker_pose.header.stamp = now();
  marker_pose.header.frame_id = base_frame_;
  marker_pose.pose = matrixToPose(base_to_marker);

  RCLCPP_INFO(get_logger(),
              "Headless AprilTag scan locked tag %d using %zu sample(s): Base [%.4f, %.4f, %.4f]",
              last_tag_id, target_to_camera_samples.size(),
              marker_pose.pose.position.x, marker_pose.pose.position.y, marker_pose.pose.position.z);
  return true;
}

void ExecutorNode::cacheMarkerPose(const geometry_msgs::msg::PoseStamped& marker_pose) {
  {
      std::scoped_lock lock(runtime_mutex_);
      last_marker_pose_ = marker_pose;
      has_cached_marker_ = true;
      workpiece_latest_marker_pose_msg_ = marker_pose;
  }

  workpiece_pose_pub_->publish(marker_pose);
}

void ExecutorNode::handleTriggerCheck(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  RCLCPP_INFO(get_logger(), "Workpiece Check: Processing slot detection from latest camera frames...");

  std::string status_str;
  bool success = false;
  int scan_count = 0;
  const int max_scan_attempts = 10;

  while (rclcpp::ok() && scan_count < max_scan_attempts) {
      scan_count++;
      if (detectWorkpieceSlotsFromLatestFrames(status_str)) {
          success = true;
          RCLCPP_INFO(get_logger(), "Workpiece detection succeeded on scan attempt %d/%d", scan_count, max_scan_attempts);
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (!success) {
      RCLCPP_WARN(get_logger(), "Workpiece detection failed after %d scan attempts", scan_count);
  }

  if (success) {
      response->success = true;
      response->message = "Success: Workpiece Tray detected and stored. Status: " + status_str;
      RCLCPP_INFO(get_logger(), "Workpiece Tray successfully segmented! Status stored: %s", status_str.c_str());
  } else {
      response->success = false;
      response->message = "Failed: Could not detect tray marker or segment table.";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }
}

void ExecutorNode::handleClearSlot(int slot_idx, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  if (slot_idx < 0 || slot_idx >= 4) {
      response->success = false;
      response->message = "Error: Invalid slot index.";
      return;
  }
  {
      std::lock_guard<std::mutex> lock(workpiece_status_mutex_);
      persistent_slot_status_[slot_idx] = "EMPTY";
      latest_packaged_status_ = "1:" + persistent_slot_status_[0] + ";2:" + persistent_slot_status_[1] + ";3:" + persistent_slot_status_[2] + ";4:" + persistent_slot_status_[3];
  }
  response->success = true;
  response->message = "Slot " + std::to_string(slot_idx + 1) + " successfully declared EMPTY.";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  
  workpiece_latest_marker_pose_msg_.header.stamp = this->get_clock()->now();
  workpiece_pose_pub_->publish(workpiece_latest_marker_pose_msg_);
}

void ExecutorNode::handleClearAllSlots(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  {
      std::lock_guard<std::mutex> lock(workpiece_status_mutex_);
      for (int i = 0; i < 4; ++i) {
          persistent_slot_status_[i] = "EMPTY";
      }
      latest_packaged_status_ = "1:" + persistent_slot_status_[0] + ";2:" + persistent_slot_status_[1] + ";3:" + persistent_slot_status_[2] + ";4:" + persistent_slot_status_[3];
  }
  response->success = true;
  response->message = "All slots successfully declared EMPTY.";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  
  workpiece_latest_marker_pose_msg_.header.stamp = this->get_clock()->now();
  workpiece_pose_pub_->publish(workpiece_latest_marker_pose_msg_);
}

void ExecutorNode::handlePlaceToAvgareaToConveryerblet(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  
  if (!beginJob("", "place_to_avgarea_to_converyerblet", 1)) {
    response->success = false;
    response->message = "Failed: Executor is already busy.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Return immediately from service, run motion on a detached thread
  // (same pattern as action handlers) so subscriptions keep processing.
  response->success = true;
  response->message = "Accepted: executing direct place at c1 in background.";

  std::thread([this]() {
    StepOptions place_options;
    place_options.velocity_scale = pick_place_velocity_scale_;
    place_options.acceleration_scale = pick_place_acceleration_scale_;
    place_options.place_descent_force = 10.0;  // Enable Phase 1.8 gripper loosening (compliance) before LIN descent

    // Place at "c1".
    // If orientation is LEFT, executePickPlaceAtPose will perform a Phase 1.5 in-place
    // PTP wrist rotation (joint 7 +PI) at approach height before the LIN descent.
    {
      std::string workpiece_orientation = "LEFT";
      get_parameter("workpiece_orientation", workpiece_orientation);
      std::string orient_upper = workpiece_orientation;
      std::transform(orient_upper.begin(), orient_upper.end(), orient_upper.begin(), ::toupper);
      if (orient_upper == "LEFT") {
        place_options.wrist_offset_rad = M_PI;
        RCLCPP_INFO(get_logger(), "[place_to_avgarea_to_converyerblet] Place at c1 with Phase 1.5 wrist rotation +PI (LEFT-facing product).");
      } else {
        RCLCPP_INFO(get_logger(), "[place_to_avgarea_to_converyerblet] Place at c1 standard orientation (RIGHT-facing product).");
      }
    }
    if (!executePickPlaceInternal("c1", "place", place_options, false, 1, 1)) {
      RCLCPP_ERROR(get_logger(), "[place_to_avgarea_to_converyerblet] Failed placing at c1: %s", runtime_.last_error.c_str());
      finishJob();
      return;
    }

    {
      std::scoped_lock lock(runtime_mutex_);
      state_ = ExecutorState::SUCCEEDED;
    }
    RCLCPP_INFO(get_logger(), "[place_to_avgarea_to_converyerblet] Done.");
    finishJob();
  }).detach();
}

}  // namespace franka_teach_executor
