#include "franka_teach_executor/scoped_allowed_collisions.hpp"

#include <future>
#include <utility>

#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/msg/planning_scene_components.hpp>

namespace franka_teach_executor {

namespace {

bool pairIsAllowed(const collision_detection::AllowedCollisionMatrix& matrix,
                   const std::string& first,
                   const std::string& second) {
  collision_detection::AllowedCollision::Type type =
      collision_detection::AllowedCollision::NEVER;
  return matrix.getAllowedCollision(first, second, type) &&
         type == collision_detection::AllowedCollision::ALWAYS;
}

}  // namespace

ScopedAllowedCollisions::ScopedAllowedCollisions(
    rclcpp::Logger logger,
    GetClient::SharedPtr get_client,
    ApplyClient::SharedPtr apply_client,
    RestoreFailureCallback restore_failure_callback)
    : logger_(std::move(logger)),
      get_client_(std::move(get_client)),
      apply_client_(std::move(apply_client)),
      restore_failure_callback_(std::move(restore_failure_callback)) {}

ScopedAllowedCollisions::~ScopedAllowedCollisions() {
  if (!active_) {
    return;
  }

  if (!restore()) {
    const std::string message =
        "Failed to restore the planning-scene Allowed Collision Matrix.";
    RCLCPP_FATAL(logger_, "%s Further robot motion must remain blocked.", message.c_str());
    if (restore_failure_callback_) {
      restore_failure_callback_(message);
    }
  }
}

bool ScopedAllowedCollisions::enable(
    const std::vector<std::string>& robot_links,
    const std::vector<std::string>& environment_links,
    std::chrono::seconds timeout) {
  if (active_) {
    RCLCPP_ERROR(logger_, "Refusing to enable an already-active collision override.");
    return false;
  }
  if (robot_links.empty() || environment_links.empty()) {
    RCLCPP_ERROR(logger_, "Collision override link lists must not be empty.");
    return false;
  }
  if (!get_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(logger_, "Planning-scene query service is unavailable.");
    return false;
  }

  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto future = get_client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(logger_, "Timed out reading the Allowed Collision Matrix.");
    return false;
  }

  const auto current_message = future.get()->scene.allowed_collision_matrix;
  collision_detection::AllowedCollisionMatrix modified_matrix(current_message);

  original_pair_states_.clear();
  robot_links_ = robot_links;
  environment_links_ = environment_links;
  original_pair_states_.reserve(robot_links.size() * environment_links.size());
  for (const auto& robot_link : robot_links) {
    for (const auto& environment_link : environment_links) {
      original_pair_states_.push_back(
          {robot_link, environment_link,
           pairIsAllowed(modified_matrix, robot_link, environment_link)});
    }
  }
  modified_matrix.setEntry(robot_links, environment_links, true);

  moveit_msgs::msg::AllowedCollisionMatrix modified_message;
  modified_matrix.getMessage(modified_message);

  bool request_sent = false;
  if (!applyMatrix(modified_message, timeout, request_sent)) {
    // If the request was submitted, its result is uncertain and restoration
    // is mandatory. If it was never submitted, no override can be active.
    active_ = request_sent;
    return false;
  }
  active_ = true;

  if (!verifyPairs(false, timeout)) {
    RCLCPP_ERROR(logger_, "Storage-rack collision allowance verification failed.");
    return false;
  }

  RCLCPP_WARN(logger_,
              "Temporarily allowing %zu hand-to-rack collision pairs for the storage LIN interaction.",
              robot_links.size() * environment_links.size());
  return true;
}

bool ScopedAllowedCollisions::refresh(std::chrono::seconds timeout) {
  if (!active_ || robot_links_.empty() || environment_links_.empty()) {
    return false;
  }
  moveit_msgs::msg::AllowedCollisionMatrix current_message;
  if (!readMatrix(current_message, timeout)) {
    return false;
  }
  collision_detection::AllowedCollisionMatrix current_matrix(current_message);
  current_matrix.setEntry(robot_links_, environment_links_, true);
  moveit_msgs::msg::AllowedCollisionMatrix updated_message;
  current_matrix.getMessage(updated_message);
  bool request_sent = false;
  if (!applyMatrix(updated_message, timeout, request_sent)) {
    return false;
  }
  return verifyPairs(false, timeout);
}

bool ScopedAllowedCollisions::restore(std::chrono::seconds timeout) {
  if (!active_) {
    return true;
  }
  moveit_msgs::msg::AllowedCollisionMatrix current_message;
  if (!readMatrix(current_message, timeout)) {
    return false;
  }

  // Merge only the pairs owned by this scope into the newest ACM. This avoids
  // overwriting unrelated planning-scene changes made while the scope was active.
  collision_detection::AllowedCollisionMatrix current_matrix(current_message);
  for (const auto& pair : original_pair_states_) {
    current_matrix.setEntry(pair.robot_link, pair.environment_link, pair.allowed);
  }

  moveit_msgs::msg::AllowedCollisionMatrix restored_message;
  current_matrix.getMessage(restored_message);
  bool request_sent = false;
  if (!applyMatrix(restored_message, timeout, request_sent)) {
    return false;
  }
  if (!verifyPairs(true, timeout)) {
    RCLCPP_ERROR(logger_, "Restored collision-pair verification failed.");
    return false;
  }

  active_ = false;
  RCLCPP_INFO(logger_, "Restored the original Allowed Collision Matrix.");
  return true;
}

bool ScopedAllowedCollisions::readMatrix(
    moveit_msgs::msg::AllowedCollisionMatrix& matrix,
    std::chrono::seconds timeout) {
  if (!get_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(logger_, "Planning-scene query service is unavailable.");
    return false;
  }

  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto future = get_client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(logger_, "Timed out reading the Allowed Collision Matrix.");
    return false;
  }
  matrix = future.get()->scene.allowed_collision_matrix;
  return true;
}

bool ScopedAllowedCollisions::applyMatrix(
    const moveit_msgs::msg::AllowedCollisionMatrix& matrix,
    std::chrono::seconds timeout,
    bool& request_sent) {
  request_sent = false;
  if (!apply_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(logger_, "Planning-scene apply service is unavailable.");
    return false;
  }

  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene.is_diff = true;
  request->scene.allowed_collision_matrix = matrix;
  auto future = apply_client_->async_send_request(request);
  request_sent = true;
  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(logger_, "Timed out applying the Allowed Collision Matrix.");
    return false;
  }
  if (!future.get()->success) {
    RCLCPP_ERROR(logger_, "MoveIt rejected the Allowed Collision Matrix update.");
    return false;
  }
  return true;
}

bool ScopedAllowedCollisions::verifyPairs(bool use_original_values,
                                          std::chrono::seconds timeout) {
  moveit_msgs::msg::AllowedCollisionMatrix current_message;
  if (!readMatrix(current_message, timeout)) {
    return false;
  }
  const collision_detection::AllowedCollisionMatrix current_matrix(current_message);
  for (const auto& pair : original_pair_states_) {
    const bool expected = use_original_values ? pair.allowed : true;
    if (pairIsAllowed(current_matrix, pair.robot_link, pair.environment_link) != expected) {
      RCLCPP_ERROR(logger_, "Unexpected ACM value for '%s' <-> '%s'.",
                   pair.robot_link.c_str(), pair.environment_link.c_str());
      return false;
    }
  }
  return true;
}

bool ScopedAllowedCollisions::setPairsAllowed(
    rclcpp::Logger logger,
    GetClient::SharedPtr get_client,
    ApplyClient::SharedPtr apply_client,
    const std::vector<std::string>& robot_links,
    const std::vector<std::string>& environment_links,
    bool allowed,
    std::chrono::seconds timeout) {
  ScopedAllowedCollisions helper(logger, std::move(get_client), std::move(apply_client));
  moveit_msgs::msg::AllowedCollisionMatrix current_message;
  if (!helper.readMatrix(current_message, timeout)) {
    return false;
  }

  collision_detection::AllowedCollisionMatrix current_matrix(current_message);
  current_matrix.setEntry(robot_links, environment_links, allowed);
  moveit_msgs::msg::AllowedCollisionMatrix updated_message;
  current_matrix.getMessage(updated_message);
  bool request_sent = false;
  if (!helper.applyMatrix(updated_message, timeout, request_sent)) {
    return false;
  }

  moveit_msgs::msg::AllowedCollisionMatrix verification_message;
  if (!helper.readMatrix(verification_message, timeout)) {
    return false;
  }
  const collision_detection::AllowedCollisionMatrix verification_matrix(verification_message);
  for (const auto& robot_link : robot_links) {
    for (const auto& environment_link : environment_links) {
      if (pairIsAllowed(verification_matrix, robot_link, environment_link) != allowed) {
        RCLCPP_ERROR(logger, "Failed to verify ACM value for '%s' <-> '%s'.",
                     robot_link.c_str(), environment_link.c_str());
        return false;
      }
    }
  }
  return true;
}

}  // namespace franka_teach_executor
