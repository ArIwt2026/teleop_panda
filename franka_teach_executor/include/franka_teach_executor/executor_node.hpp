#pragma once

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <franka_msgs/action/error_recovery.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <franka_msgs/msg/franka_robot_state.hpp>
#include <franka_teach_msgs/action/execute_relative_point.hpp>
#include <franka_teach_msgs/action/go_to_camera_pose.hpp>
#include <franka_teach_msgs/action/pick_place.hpp>
#include <franka_teach_msgs/action/play_point.hpp>
#include <franka_teach_msgs/action/play_sequence.hpp>
#include <franka_teach_msgs/msg/executor_status.hpp>
#include <franka_teach_msgs/srv/get_executor_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace franka_teach_executor {

struct PointData {
  std::string name;
  std::string type = "point"; // "point" (joint-based) or "cartesian_relative"
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  
  // For Cartesian points
  std::string relative_motion_mode;
  std::optional<Eigen::Vector3d> relative_position;
  std::optional<geometry_msgs::msg::Quaternion> reference_orientation;
  double marker_yaw_at_teach_rad = 0.0;
  std::string frame_id;
};

struct StepOptions {
  std::string advance_mode = "auto";
  std::string planner_id = "";
  std::string planning_pipeline_id = "";
  double velocity_scale = -1.0;
  double acceleration_scale = -1.0;
  double planning_time = -1.0;
  double goal_tolerance = -1.0;
  double approach_height = -1.0;
  double lift_off_height = -1.0;  // Vertical lift in global Z (PICK: after grasp, PLACE: extra clearance)
  double gripper_width = -1.0;
  double gripper_force = -1.0;
  double gripper_speed = -1.0;
  double place_descent_force = -1.0;
  bool base_only = false;
  bool skip_observation_pose = false;
  std::string ref_point_name = "";
  // When non-zero, this offset (radians) is added to the last joint (joint 7 / wrist)
  // before setJointValueTarget() is called. Use for PTP wrist pre-rotation.
  double wrist_offset_rad = 0.0;
};

struct SequenceStep {
  uint32_t index = 0;
  std::string type;  // "move_to_point" (default) or "gripper"
  std::string point_name;
  std::string point_file;
  StepOptions options;
  // Gripper-specific fields (only used when type == "gripper")
  std::string gripper_action;  // "grasp" or "open"
  double gripper_width = 0.08;
  double gripper_speed = 0.1;
  double gripper_force = 50.0;
  double gripper_epsilon_inner = 0.005;
  double gripper_epsilon_outer = 0.005;
};

struct SequenceData {
  std::string name;
  std::vector<SequenceStep> steps;
};

enum class ExecutorState {
  IDLE,
  PLANNING,
  EXECUTING,
  WAITING_CONTINUE,
  PAUSE_REQUESTED,
  PAUSED,
  FAULT_REFLEX,
  FAULT_STOPPED,
  RECOVERING,
  STOPPING,
  SUCCEEDED,
  FAILED,
  STOPPED,
};

class ExecutorNode : public rclcpp::Node {
 public:
  using GoToCameraPose = franka_teach_msgs::action::GoToCameraPose;
  using ExecuteRelativePoint = franka_teach_msgs::action::ExecuteRelativePoint;
  using PlayPoint = franka_teach_msgs::action::PlayPoint;
  using PlaySequence = franka_teach_msgs::action::PlaySequence;
  using PickPlace = franka_teach_msgs::action::PickPlace;
  using GoToCameraPoseGoalHandle = rclcpp_action::ServerGoalHandle<GoToCameraPose>;
  using ExecuteRelativePointGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteRelativePoint>;
  using PlayPointGoalHandle = rclcpp_action::ServerGoalHandle<PlayPoint>;
  using PlaySequenceGoalHandle = rclcpp_action::ServerGoalHandle<PlaySequence>;
  using PickPlaceGoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;
  using ErrorRecovery = franka_msgs::action::ErrorRecovery;
  using GripperGrasp = franka_msgs::action::Grasp;
  using GripperMove = franka_msgs::action::Move;

  ExecutorNode();

  void initialize();
  bool isNearPoint(const std::string& point_name, double tolerance_rad = 0.5) const;

 private:
  struct RuntimeContext {
    std::string sequence_name;
    std::string point_name;
    std::size_t total_steps = 0;
    std::size_t completed_steps = 0;
    std::size_t active_step_index = 0;
    std::size_t failed_step_index = 0;
    std::string failed_point_name;
    std::string last_error;
    bool plan_only = false;
    bool pause_requested = false;
    bool continue_requested = false;
    bool stop_requested = false;
    bool recover_requested = false;
    bool waiting_for_continue = false;
    bool fault_reflex = false;
    bool fault_stopped = false;
    bool recover_required = false;
    bool busy = false;
  };

  struct LatestJointState {
    sensor_msgs::msg::JointState message;
    rclcpp::Time received_time;
    bool valid = false;
  };

  struct LatestRobotState {
    franka_msgs::msg::FrankaRobotState message;
    rclcpp::Time received_time;
    bool valid = false;
  };

 struct RuntimeOverrides {
    double velocity_scale = -1.0;
    double acceleration_scale = -1.0;
    double planning_time = -1.0;
    double goal_tolerance = -1.0;
  };

  enum class StepExecutionResult {
    SUCCESS,
    RETRY,
    STOPPED,
    FAILED,
  };

  rclcpp_action::Server<GoToCameraPose>::SharedPtr go_to_camera_pose_server_;
  rclcpp_action::Server<ExecuteRelativePoint>::SharedPtr execute_relative_point_server_;
  rclcpp_action::Server<PlayPoint>::SharedPtr play_point_server_;
  rclcpp_action::Server<PlaySequence>::SharedPtr play_sequence_server_;
  rclcpp_action::Server<PickPlace>::SharedPtr pick_place_server_;
  rclcpp_action::Client<ErrorRecovery>::SharedPtr error_recovery_client_;
  rclcpp_action::Client<GripperGrasp>::SharedPtr gripper_grasp_client_;
  rclcpp_action::Client<GripperMove>::SharedPtr gripper_move_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_stop_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr continue_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_service_;
  rclcpp::Service<franka_teach_msgs::srv::GetExecutorStatus>::SharedPtr status_service_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr robot_state_subscription_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex runtime_mutex_;
  std::condition_variable runtime_cv_;
  RuntimeContext runtime_;
  ExecutorState state_ = ExecutorState::IDLE;

  mutable std::mutex state_sample_mutex_;
  LatestJointState latest_joint_state_;
  LatestRobotState latest_robot_state_;

  std::filesystem::path storage_root_;
  std::string planning_group_;
  std::string joint_state_topic_;
  std::string robot_state_topic_;
  std::string base_frame_;
  std::string camera_frame_;
  std::string calibration_result_path_;
  std::string error_recovery_action_name_;
  bool enable_robot_state_monitoring_ = true;
  double joint_state_timeout_sec_ = 1.0;
  double robot_state_timeout_sec_ = 1.0;
  double default_velocity_scale_ = 0.2;
  double default_acceleration_scale_ = 0.2;
  double default_planning_time_ = 5.0;
  double default_goal_tolerance_ = 0.01;
  double default_orientation_tolerance_ = 0.001;
  bool use_fake_hardware_ = false;
  
  // Global motion profile parameters
  double ptp_velocity_scale_ = 0.6;
  double ptp_acceleration_scale_ = 0.5;
  double lin_velocity_scale_ = 0.3;
  double lin_acceleration_scale_ = 0.2;
  double pick_place_velocity_scale_ = 0.2;
  double pick_place_acceleration_scale_ = 0.2;
  bool storage_rack_collision_override_enabled_ = true;
  std::vector<std::string> storage_rack_collision_robot_links_;
  std::vector<std::string> storage_rack_collision_environment_links_;
  std::atomic<bool> storage_rack_collision_safety_ready_{false};
  std::atomic<bool> storage_rack_collision_cleanup_running_{false};
  std::string gripper_grasp_action_name_;
  std::string gripper_move_action_name_;
  std::string gripper_stop_service_name_;
  std::string planning_pipeline_id_;
  std::string planner_id_;
  double relative_approach_height_ = 0.30;
  Eigen::Matrix4d T_ee_camera_ = Eigen::Matrix4d::Identity();
  bool calibration_loaded_ = false;
  geometry_msgs::msg::PoseStamped last_marker_pose_;
  bool has_cached_marker_ = false;

  rclcpp_action::GoalResponse handleGoToCameraPoseGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const GoToCameraPose::Goal> goal);
  rclcpp_action::CancelResponse handleGoToCameraPoseCancel(
      const std::shared_ptr<GoToCameraPoseGoalHandle>& goal_handle);
  void handleGoToCameraPoseAccepted(
      const std::shared_ptr<GoToCameraPoseGoalHandle>& goal_handle);

  rclcpp_action::GoalResponse handleExecuteRelativePointGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ExecuteRelativePoint::Goal> goal);
  rclcpp_action::CancelResponse handleExecuteRelativePointCancel(
      const std::shared_ptr<ExecuteRelativePointGoalHandle>& goal_handle);
  void handleExecuteRelativePointAccepted(
      const std::shared_ptr<ExecuteRelativePointGoalHandle>& goal_handle);

  rclcpp_action::GoalResponse handlePlayPointGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const PlayPoint::Goal> goal);
  rclcpp_action::CancelResponse handlePlayPointCancel(
      const std::shared_ptr<PlayPointGoalHandle>& goal_handle);
  void handlePlayPointAccepted(const std::shared_ptr<PlayPointGoalHandle>& goal_handle);

  rclcpp_action::GoalResponse handlePlaySequenceGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const PlaySequence::Goal> goal);
  rclcpp_action::CancelResponse handlePlaySequenceCancel(
      const std::shared_ptr<PlaySequenceGoalHandle>& goal_handle);
  void handlePlaySequenceAccepted(const std::shared_ptr<PlaySequenceGoalHandle>& goal_handle);

  rclcpp_action::GoalResponse handlePickPlaceGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const PickPlace::Goal> goal);
  rclcpp_action::CancelResponse handlePickPlaceCancel(
      const std::shared_ptr<PickPlaceGoalHandle>& goal_handle);
  void handlePickPlaceAccepted(const std::shared_ptr<PickPlaceGoalHandle>& goal_handle);

  void executeCameraPoseGoal(const std::shared_ptr<GoToCameraPoseGoalHandle>& goal_handle);
  void executeRelativePointGoal(const std::shared_ptr<ExecuteRelativePointGoalHandle>& goal_handle);
  void executePointGoal(const std::shared_ptr<PlayPointGoalHandle>& goal_handle);
  void executeSequenceGoal(const std::shared_ptr<PlaySequenceGoalHandle>& goal_handle);
  void executePickPlaceGoal(const std::shared_ptr<PickPlaceGoalHandle>& goal_handle);

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void robotStateCallback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg);

  void handlePause(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleContinue(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleStop(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleRecover(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleStatus(
      const std::shared_ptr<franka_teach_msgs::srv::GetExecutorStatus::Request>& request,
      std::shared_ptr<franka_teach_msgs::srv::GetExecutorStatus::Response> response);

  std::filesystem::path pointPath(const std::string& point_name) const;
  std::filesystem::path sequencePath(const std::string& sequence_name) const;
  std::filesystem::path detectStorageRoot() const;
  PointData loadPoint(const std::string& point_name) const;
  SequenceData loadSequence(const std::string& sequence_name) const;
  StepOptions mergeOptions(const StepOptions& step_options, const RuntimeOverrides& overrides) const;
  std::vector<double> reorderPointToGroup(
      const PointData& point,
      const std::vector<std::string>& group_joint_names) const;

  bool isRobotStateFresh() const;
  bool isJointStateFresh() const;
  bool isCurrentStepAtTarget(const std::vector<std::string>& joint_names,
                             const std::vector<double>& target_positions,
                             double tolerance) const;
  bool robotModeIsReflex() const;
  bool robotModeIsStopped() const;
  bool stopConditionCleared() const;
  bool runRecoveryAction(std::string& error_message);
  bool transformCameraPoseToBase(const geometry_msgs::msg::PoseStamped& camera_pose,
                                 geometry_msgs::msg::PoseStamped& base_pose,
                                 std::string& error_message) const;
  bool isCurrentPoseAtTarget(const geometry_msgs::msg::Pose& target_pose,
                             double position_tolerance,
                             double orientation_tolerance) const;
  void loadCalibration();

  std::string stateToString(ExecutorState state) const;
  franka_teach_msgs::msg::ExecutorStatus makeStatusMessage() const;
  void setState(ExecutorState state);
  void resetRuntime();
  void setRuntimeError(const std::string& error_message);
  bool beginJob(const std::string& sequence_name, const std::string& point_name, std::size_t total_steps);
  void finishJob();

  template <typename GoalHandleT>
  void publishFeedback(const std::shared_ptr<GoalHandleT>& goal_handle) {
    try {
      if constexpr (std::is_same_v<GoalHandleT, GoToCameraPoseGoalHandle>) {
        auto feedback = std::make_shared<GoToCameraPose::Feedback>();
        feedback->status = makeStatusMessage();
        goal_handle->publish_feedback(feedback);
      } else if constexpr (std::is_same_v<GoalHandleT, ExecuteRelativePointGoalHandle>) {
        auto feedback = std::make_shared<ExecuteRelativePoint::Feedback>();
        feedback->status = makeStatusMessage();
        goal_handle->publish_feedback(feedback);
      } else if constexpr (std::is_same_v<GoalHandleT, PlayPointGoalHandle>) {
        auto feedback = std::make_shared<PlayPoint::Feedback>();
        feedback->status = makeStatusMessage();
        goal_handle->publish_feedback(feedback);
      } else if constexpr (std::is_same_v<GoalHandleT, PickPlaceGoalHandle>) {
        auto feedback = std::make_shared<PickPlace::Feedback>();
        feedback->status = makeStatusMessage();
        goal_handle->publish_feedback(feedback);
      } else {
        auto feedback = std::make_shared<PlaySequence::Feedback>();
        feedback->status = makeStatusMessage();
        goal_handle->publish_feedback(feedback);
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to publish feedback: %s", e.what());
    }
  }


  template <typename ResultT>
  void fillResultCommon(ResultT& result, bool success, const std::string& final_state,
                        const std::string& target_name) {
    const auto status = makeStatusMessage();
    result.success = success;
    result.final_state = final_state;
    result.completed_steps = status.completed_steps;
    result.failed_step_index = status.failed_step_index;
    result.failed_point_name = status.failed_point_name;
    result.error_message = status.last_error;
    result.final_status = status;
    if constexpr (std::is_same_v<ResultT, GoToCameraPose::Result>) {
      result.target_frame = target_name;
    } else if constexpr (std::is_same_v<ResultT, ExecuteRelativePoint::Result>) {
      // Just common fields
    } else if constexpr (std::is_same_v<ResultT, PlayPoint::Result>) {
      result.point_name = target_name;
    } else if constexpr (std::is_same_v<ResultT, PickPlace::Result>) {
      result.point_name = target_name;
    } else {
      result.sequence_name = target_name;
    }
  }

  bool executeCameraPoseInternal(const geometry_msgs::msg::PoseStamped& target_pose_camera,
                                 const StepOptions& step_options,
                                 bool plan_only,
                                 std::size_t step_index,
                                 std::size_t total_steps,
                                 geometry_msgs::msg::PoseStamped& transformed_target_pose);
  bool executeRelativePointInternal(const std::string& point_name,
                                    const geometry_msgs::msg::PoseStamped& marker_pose,
                                    const StepOptions& step_options,
                                    const std::string& mode,
                                    bool plan_only,
                                    std::size_t step_index,
                                    std::size_t total_steps,
                                    geometry_msgs::msg::PoseStamped& transformed_target_pose);
  bool executePointInternal(const std::string& point_name, const StepOptions& step_options,
                            bool plan_only, std::size_t step_index, std::size_t total_steps);
  bool executePickPlaceInternal(const std::string& point_name, const std::string& mode,
                                const StepOptions& step_options, bool plan_only,
                                std::size_t step_index, std::size_t total_steps);

  /**
   * @brief Core logic for the 4-phase Pick/Place macro at a specific Cartesian pose.
   * Phases: 1. Approach (PTP), 2. Descent (LIN), 3. Action (Gripper), 4. Retract (LIN)
   */
  bool executePickPlaceAtPose(const geometry_msgs::msg::Pose& target_pose,
                              const std::vector<double>& seed_joint_positions,
                              const std::string& mode,
                              const StepOptions& step_options,
                              bool allow_storage_rack_hand_contact,
                              bool plan_only,
                              std::size_t step_index,
                              std::size_t total_steps);
  void validateStorageRackCollisionLinks() const;
  void startStorageRackCollisionCleanup();
  std::vector<std::string> storageRackCollisionLinksForTarget(
      const geometry_msgs::msg::Pose& target_pose) const;
  bool executeGripperStep(const SequenceStep& step, std::size_t step_index, std::size_t total_steps);
  bool waitForStepGateOrFault(const std::string& point_name, std::size_t step_index, std::size_t total_steps);
  StepExecutionResult waitForStoppedConditionClear(const std::string& error_message);
  StepExecutionResult waitForReflexRecovery();

  // --- Workpiece Detector Integration ---
  struct CameraIntrinsics {
      float fx = 918.0f;
      float fy = 918.0f;
      float cx = 640.0f;
      float cy = 360.0f;
      float depth_scale = 0.001f;
  };
  struct SlotBBox {
      float min_x;
      float max_x;
      float min_y;
      float max_y;
  };

  void loadWorkpieceConfig();
  void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  Eigen::Vector4f extractTablePlane(const cv::Mat& depth_image, const cv::Mat& table_search_mask);
  bool moveToWorkpieceScanPose(std::string& target_point, std::string& error_message);
  bool detectWorkpieceSlotsFromLatestFrames(std::string& status_str);
  bool estimateMarkerTransformCamera(const cv::Mat& color_mat,
                                     const cv::Mat& camera_matrix,
                                     const cv::Mat& dist_coeffs,
                                     Eigen::Matrix4d& target_to_camera,
                                     int& tag_id,
                                     std::string& error_message) const;
  bool getCurrentBaseToEe(Eigen::Matrix4d& base_to_ee, std::string& error_message);
  bool captureHeadlessMarkerPose(geometry_msgs::msg::PoseStamped& marker_pose,
                                 std::string& error_message);
  void cacheMarkerPose(const geometry_msgs::msg::PoseStamped& marker_pose);
  void handleTriggerCheck(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleClearSlot(int slot_idx, const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleClearAllSlots(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handlePlaceToAvgareaToConveryerblet(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::string workpiece_config_path_;
  double workpiece_slot_coords_[4][2];
  std::string latest_packaged_status_ = "1:EMPTY;2:EMPTY;3:EMPTY;4:EMPTY";
  std::string persistent_slot_status_[4] = {"EMPTY", "EMPTY", "EMPTY", "EMPTY"};
  bool workpiece_tag_currently_detected_ = false;
  std::chrono::steady_clock::time_point workpiece_last_detection_time_;
  std::mutex workpiece_status_mutex_;

  cv::Mat workpiece_latest_color_;
  cv::Mat workpiece_latest_depth_;
  cv::Mat workpiece_dist_coeffs_;
  std::string workpiece_latest_color_frame_id_;
  rclcpp::Time workpiece_latest_color_stamp_;
  geometry_msgs::msg::PoseStamped workpiece_latest_marker_pose_msg_;
  bool workpiece_color_received_ = false;
  bool workpiece_depth_received_ = false;
  bool workpiece_intrinsics_loaded_ = false;
  CameraIntrinsics workpiece_intrinsics_;
  std::mutex workpiece_image_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr workpiece_color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr workpiece_depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr workpiece_camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr workpiece_pose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr workpiece_trigger_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr workpiece_clear_slot_services_[4];
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr workpiece_clear_all_slots_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr place_to_avgarea_to_converyerblet_service_;
};

}  // namespace franka_teach_executor
