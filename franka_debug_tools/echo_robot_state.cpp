#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./echo_robot_state <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    std::cout << "Connecting to Robot at " << argv[1] << "..." << std::endl;
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    
    std::cout << "Connecting to Gripper..." << std::endl;
    franka::Gripper gripper(argv[1]);

    std::cout << "Connected! Press Ctrl+C to stop." << std::endl;
    std::cout << std::fixed << std::setprecision(4);

    while (true) {
      // Get Robot State
      franka::RobotState state = robot.readOnce();
      
      // Get Gripper State
      franka::GripperState gripper_state = gripper.readOnce();

      std::cout << "\rArm Position: ";
      for (double q : state.q) { std::cout << std::setw(8) << q << " "; }
      
      std::cout << " | Gripper Width: " << std::setw(8) << gripper_state.width 
                << " (Max: " << gripper_state.max_width << ")"
                << " | Grasped: " << (gripper_state.is_grasped ? "YES" : "NO ")
                << std::flush;

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  } catch (franka::Exception const& e) {
    std::cout << "\nError: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
