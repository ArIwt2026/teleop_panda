#include "franka_teach_executor/executor_node.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<franka_teach_executor::ExecutorNode>();
    node->initialize();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& ex) {
    // Process-group shutdown can invalidate the ROS context while initialize()
    // is still waiting for MoveIt. That is an expected shutdown path, not an
    // executor startup failure.
    if (!rclcpp::ok()) {
      return 0;
    }
    auto logger = rclcpp::get_logger("franka_teach_executor");
    RCLCPP_FATAL(
        logger,
        "Executor startup failed: %s. Start your normal MoveIt/controller stack first, and "
        "launch the executor with matching robot model arguments such as "
        "`use_fake_hardware:=true robot_ip:=dont-care` for fake hardware.",
        ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
