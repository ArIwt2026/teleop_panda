// Direct libfranka zero-torque hand-guiding test.
// The robot's internal controller adds gravity and friction compensation.

#include <array>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>\n";
    return 1;
  }

  try {
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);

    std::cout << "Starting direct zero-torque hand-guiding.\n"
              << "Release the user stop only when ready. Press Ctrl-C to stop.\n";

    robot.control([](const franka::RobotState&, franka::Duration) -> franka::Torques {
      // Torques are specified without gravity and friction. libfranka/Control
      // compensates those internally, leaving the arm free to guide.
      return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    });
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka error: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
