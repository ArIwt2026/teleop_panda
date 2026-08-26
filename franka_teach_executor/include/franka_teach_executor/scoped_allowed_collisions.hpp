#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_teach_executor {

/**
 * Temporarily permits selected collision pairs in MoveIt's planning scene.
 *
 * The complete previous Allowed Collision Matrix is restored either by an
 * explicit restore() call or, as a best-effort fallback, by the destructor.
 */
class ScopedAllowedCollisions {
public:
  using ApplyClient = rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>;
  using GetClient = rclcpp::Client<moveit_msgs::srv::GetPlanningScene>;
  using RestoreFailureCallback = std::function<void(const std::string&)>;

  ScopedAllowedCollisions(
      rclcpp::Logger logger,
      GetClient::SharedPtr get_client,
      ApplyClient::SharedPtr apply_client,
      RestoreFailureCallback restore_failure_callback = {});
  ~ScopedAllowedCollisions();

  ScopedAllowedCollisions(const ScopedAllowedCollisions&) = delete;
  ScopedAllowedCollisions& operator=(const ScopedAllowedCollisions&) = delete;

  bool enable(const std::vector<std::string>& robot_links,
              const std::vector<std::string>& environment_links,
              std::chrono::seconds timeout = std::chrono::seconds(5));
  bool restore(std::chrono::seconds timeout = std::chrono::seconds(5));
  bool refresh(std::chrono::seconds timeout = std::chrono::seconds(5));
  bool active() const { return active_; }

  static bool setPairsAllowed(
      rclcpp::Logger logger,
      GetClient::SharedPtr get_client,
      ApplyClient::SharedPtr apply_client,
      const std::vector<std::string>& robot_links,
      const std::vector<std::string>& environment_links,
      bool allowed,
      std::chrono::seconds timeout = std::chrono::seconds(5));

private:
  struct OriginalPairState {
    std::string robot_link;
    std::string environment_link;
    bool allowed = false;
  };

  bool readMatrix(moveit_msgs::msg::AllowedCollisionMatrix& matrix,
                  std::chrono::seconds timeout);
  bool applyMatrix(const moveit_msgs::msg::AllowedCollisionMatrix& matrix,
                   std::chrono::seconds timeout,
                   bool& request_sent);
  bool verifyPairs(bool use_original_values, std::chrono::seconds timeout);

  rclcpp::Logger logger_;
  GetClient::SharedPtr get_client_;
  ApplyClient::SharedPtr apply_client_;
  RestoreFailureCallback restore_failure_callback_;
  std::vector<OriginalPairState> original_pair_states_;
  std::vector<std::string> robot_links_;
  std::vector<std::string> environment_links_;
  bool active_ = false;
};

}  // namespace franka_teach_executor
