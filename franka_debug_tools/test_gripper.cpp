// Franka Gripper Direct Test via libfranka
// Tests: connection, state read, homing, move, grasp, stop
// Usage: ./test_gripper <robot-hostname> [command]
// Commands: state, home, open, close, grasp, move <width_mm>, full_test

#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <csignal>
#include <atomic>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>

static std::atomic<bool> g_running{true};

void signal_handler(int /*sig*/) {
  g_running = false;
}

// ──────────────────────────────────────────────────────────────
// Helper: Print gripper state
// ──────────────────────────────────────────────────────────────
void print_gripper_state(const franka::GripperState& state) {
  std::cout << "\n╔══════════════════════════════════════╗\n";
  std::cout << "║       GRIPPER STATE                   ║\n";
  std::cout << "╠══════════════════════════════════════╣\n";
  std::cout << "║  Width:       " << std::fixed << std::setprecision(4) 
            << state.width << " m"
            << "  (" << (state.width * 1000.0) << " mm)" << std::endl;
  std::cout << "║  Max Width:   " << state.max_width << " m"
            << "  (" << (state.max_width * 1000.0) << " mm)" << std::endl;
  std::cout << "║  Is Grasped:  " << (state.is_grasped ? "YES" : "NO") << std::endl;
  std::cout << "║  Temperature: " << state.temperature << " °C" << std::endl;
  std::cout << "║  Time:        " << state.time.toMSec() << " ms" << std::endl;
  std::cout << "╠══════════════════════════════════════╣\n";
  std::cout << "║  Raw JSON: " << state << std::endl;
  std::cout << "╚══════════════════════════════════════╝\n" << std::endl;
}

// ──────────────────────────────────────────────────────────────
// Test 1: Just read state (no motion)
// ──────────────────────────────────────────────────────────────
int cmd_state(const std::string& ip) {
  std::cout << "[STATE] Connecting to gripper at " << ip << "..." << std::endl;
  franka::Gripper gripper(ip);
  std::cout << "[STATE] Connected. Reading state..." << std::endl;

  auto state = gripper.readOnce();
  print_gripper_state(state);

  // Also read robot state briefly
  std::cout << "[STATE] Connecting to robot arm..." << std::endl;
  franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
  auto rstate = robot.readOnce();
  
  std::cout << "[STATE] Robot mode: ";
  switch (rstate.robot_mode) {
    case franka::RobotMode::kOther:                std::cout << "Other"; break;
    case franka::RobotMode::kIdle:                 std::cout << "Idle (READY)"; break;
    case franka::RobotMode::kMove:                 std::cout << "Move"; break;
    case franka::RobotMode::kGuiding:              std::cout << "Guiding"; break;
    case franka::RobotMode::kReflex:               std::cout << "Reflex (ERROR)"; break;
    case franka::RobotMode::kUserStopped:          std::cout << "User Stopped"; break;
    case franka::RobotMode::kAutomaticErrorRecovery: std::cout << "Auto Error Recovery"; break;
  }
  std::cout << std::endl;

  std::cout << "[STATE] Joint positions: [";
  for (size_t i = 0; i < 7; i++) {
    std::cout << std::fixed << std::setprecision(3) << rstate.q[i];
    if (i < 6) std::cout << ", ";
  }
  std::cout << "]" << std::endl;

  // Check for errors
  bool has_errors = false;
  auto errors = rstate.current_errors;
  // Check a few key error flags
  if (errors.joint_position_limits_violation) { std::cout << "  ERROR: joint_position_limits_violation\n"; has_errors = true; }
  if (errors.cartesian_position_limits_violation) { std::cout << "  ERROR: cartesian_position_limits_violation\n"; has_errors = true; }
  if (errors.self_collision_avoidance_violation) { std::cout << "  ERROR: self_collision_avoidance_violation\n"; has_errors = true; }
  if (errors.joint_reflex) { std::cout << "  ERROR: joint_reflex\n"; has_errors = true; }
  if (errors.cartesian_reflex) { std::cout << "  ERROR: cartesian_reflex\n"; has_errors = true; }
  if (errors.tau_j_range_violation) { std::cout << "  ERROR: tau_j_range_violation\n"; has_errors = true; }
  if (errors.communication_constraints_violation) { std::cout << "  ERROR: communication_constraints_violation\n"; has_errors = true; }
  if (errors.power_limit_violation) { std::cout << "  ERROR: power_limit_violation\n"; has_errors = true; }
  
  if (!has_errors) {
    std::cout << "[STATE] No active errors." << std::endl;
  }

  return 0;
}

// ──────────────────────────────────────────────────────────────
// Test 2: Homing (calibrate gripper)
// ──────────────────────────────────────────────────────────────
int cmd_home(const std::string& ip) {
  std::cout << "[HOME] Connecting to gripper at " << ip << "..." << std::endl;
  franka::Gripper gripper(ip);
  std::cout << "[HOME] Connected. Performing homing..." << std::endl;

  bool success = gripper.homing();
  
  if (success) {
    std::cout << "[HOME] ✓ Homing succeeded!" << std::endl;
    auto state = gripper.readOnce();
    print_gripper_state(state);
  } else {
    std::cout << "[HOME] ✗ Homing FAILED!" << std::endl;
    return 1;
  }
  return 0;
}

// ──────────────────────────────────────────────────────────────
// Test 3: Open gripper fully
// ──────────────────────────────────────────────────────────────
int cmd_open(const std::string& ip) {
  std::cout << "[OPEN] Connecting to gripper at " << ip << "..." << std::endl;
  franka::Gripper gripper(ip);

  auto state = gripper.readOnce();
  double max_w = state.max_width;
  std::cout << "[OPEN] Max width: " << max_w << " m. Opening fully..." << std::endl;

  bool success = gripper.move(max_w, 0.05);  // 50 mm/s
  
  if (success) {
    std::cout << "[OPEN] ✓ Gripper opened successfully!" << std::endl;
    state = gripper.readOnce();
    print_gripper_state(state);
  } else {
    std::cout << "[OPEN] ✗ Open FAILED!" << std::endl;
    return 1;
  }
  return 0;
}

// ──────────────────────────────────────────────────────────────
// Test 4: Close gripper (grasp with low force)
// ──────────────────────────────────────────────────────────────
int cmd_close(const std::string& ip) {
  std::cout << "[CLOSE] Connecting to gripper at " << ip << "..." << std::endl;
  franka::Gripper gripper(ip);
  std::cout << "[CLOSE] Closing gripper (grasp width=0, force=10N)..." << std::endl;

  // Use grasp with 0 width and moderate force/epsilon
  bool success = gripper.grasp(0.0, 0.05, 10.0, 0.08, 0.08);
  
  auto state = gripper.readOnce();
  if (success) {
    std::cout << "[CLOSE] ✓ Grasp succeeded! is_grasped=" << state.is_grasped << std::endl;
  } else {
    std::cout << "[CLOSE] Grasp returned false (no object grasped), width=" 
              << state.width << " m" << std::endl;
    std::cout << "[CLOSE] This is normal if no object is between the fingers." << std::endl;
  }
  print_gripper_state(state);
  return 0;
}

// ──────────────────────────────────────────────────────────────
// Test 5: Grasp at a specific width with configurable force
// ──────────────────────────────────────────────────────────────
int cmd_grasp(const std::string& ip) {
  std::cout << "[GRASP] Connecting to gripper at " << ip << "..." << std::endl;
  franka::Gripper gripper(ip);

  double width, force;
  std::cout << "Enter grasp width in mm (e.g. 20): ";
  std::cin >> width;
  std::cout << "Enter grasp force in N (e.g. 40): ";
  std::cin >> force;

  double width_m = width / 1000.0;
  std::cout << "[GRASP] Grasping: width=" << width_m << " m, speed=0.05 m/s, force=" 
            << force << " N..." << std::endl;

  bool success = gripper.grasp(width_m, 0.05, force, 0.005, 0.005);
  
  auto state = gripper.readOnce();
  if (success) {
    std::cout << "[GRASP] ✓ Grasped! is_grasped=" << state.is_grasped << std::endl;
  } else {
    std::cout << "[GRASP] ✗ Grasp returned false (object lost or not grasped)" << std::endl;
  }
  print_gripper_state(state);
  return 0;
}

// ──────────────────────────────────────────────────────────────
// Test 6: Move to specific width
// ──────────────────────────────────────────────────────────────
int cmd_move(const std::string& ip, double width_mm) {
  std::cout << "[MOVE] Connecting to gripper at " << ip << "..." << std::endl;
  franka::Gripper gripper(ip);

  double width_m = width_mm / 1000.0;
  
  auto state = gripper.readOnce();
  if (width_m > state.max_width) {
    std::cout << "[MOVE] ✗ Requested width " << width_m << " m exceeds max " 
              << state.max_width << " m" << std::endl;
    return 1;
  }

  std::cout << "[MOVE] Moving to width " << width_m << " m (" << width_mm << " mm) at 0.05 m/s..." 
            << std::endl;

  bool success = gripper.move(width_m, 0.05);
  
  state = gripper.readOnce();
  if (success) {
    std::cout << "[MOVE] ✓ Move succeeded!" << std::endl;
  } else {
    std::cout << "[MOVE] ✗ Move FAILED!" << std::endl;
  }
  print_gripper_state(state);
  return 0;
}

// ──────────────────────────────────────────────────────────────
// Test 7: Full automated test sequence
// ──────────────────────────────────────────────────────────────
int cmd_full_test(const std::string& ip) {
  std::cout << "\n====================================================\n";
  std::cout << "  FRANKA GRIPPER FULL TEST SEQUENCE\n";
  std::cout << "  Robot: " << ip << "\n";
  std::cout << "====================================================\n\n";

  franka::Gripper gripper(ip);

  // Step 1: Read initial state
  std::cout << "─── Step 1/6: Read Initial State ───\n";
  auto state = gripper.readOnce();
  print_gripper_state(state);

  // Step 2: Homing
  std::cout << "─── Step 2/6: Homing ───\n";
  std::cout << "Performing homing (fingers will fully open then close)..." << std::endl;
  bool home_ok = gripper.homing();
  if (!home_ok) {
    std::cout << "✗ HOMING FAILED — aborting test.\n";
    return 1;
  }
  std::cout << "✓ Homing done.\n";
  state = gripper.readOnce();
  print_gripper_state(state);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Step 3: Move to half width
  std::cout << "─── Step 3/6: Move to Half Width ───\n";
  double half_w = state.max_width / 2.0;
  std::cout << "Moving to " << half_w * 1000.0 << " mm..." << std::endl;
  bool move_ok = gripper.move(half_w, 0.05);
  if (!move_ok) {
    std::cout << "✗ MOVE FAILED!\n";
    return 1;
  }
  std::cout << "✓ Move to half done.\n";
  state = gripper.readOnce();
  print_gripper_state(state);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Step 4: Close fully
  std::cout << "─── Step 4/6: Close Fully ───\n";
  std::cout << "Closing gripper..." << std::endl;
  bool close_ok = gripper.move(0.0, 0.05);
  // Note: move(0) may fail if fingers physically can't reach 0
  std::cout << (close_ok ? "✓" : "~") << " Close done (may not reach exact 0).\n";
  state = gripper.readOnce();
  print_gripper_state(state);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Step 5: Open fully
  std::cout << "─── Step 5/6: Open Fully ───\n";
  std::cout << "Opening gripper to max width..." << std::endl;
  bool open_ok = gripper.move(state.max_width, 0.05);
  if (!open_ok) {
    std::cout << "✗ OPEN FAILED!\n";
    return 1;
  }
  std::cout << "✓ Fully open.\n";
  state = gripper.readOnce();
  print_gripper_state(state);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Step 6: Test grasp (simulated, no object)
  std::cout << "─── Step 6/6: Grasp Test (no object) ───\n";
  std::cout << "Attempting grasp at 20mm with 30N force..." << std::endl;
  bool grasp_ok = gripper.grasp(0.02, 0.05, 30.0, 0.01, 0.01);
  state = gripper.readOnce();
  std::cout << (grasp_ok ? "✓ Grasp succeeded (unexpected without object)" 
                         : "Expected: Grasp returned false (no object)") << std::endl;
  print_gripper_state(state);

  // Re-open
  std::cout << "Re-opening gripper..." << std::endl;
  gripper.move(state.max_width, 0.05);
  state = gripper.readOnce();
  print_gripper_state(state);

  std::cout << "\n====================================================\n";
  std::cout << "  FULL TEST COMPLETE\n";
  std::cout << "====================================================\n\n";
  return 0;
}

// ──────────────────────────────────────────────────────────────
// Test: Continuous monitoring
// ──────────────────────────────────────────────────────────────
int cmd_monitor(const std::string& ip) {
  std::cout << "[MONITOR] Connecting to gripper at " << ip << "..." << std::endl;
  franka::Gripper gripper(ip);
  
  std::cout << "[MONITOR] Continuously reading gripper state. Press Ctrl+C to stop.\n" << std::endl;
  
  signal(SIGINT, signal_handler);
  
  while (g_running) {
    auto state = gripper.readOnce();
    std::cout << "\rWidth: " << std::fixed << std::setprecision(4) << state.width 
              << " m (" << std::setprecision(1) << state.width * 1000.0 << " mm)"
              << " | Max: " << std::setprecision(4) << state.max_width << " m"
              << " | Grasped: " << (state.is_grasped ? "YES" : "NO ")
              << " | Temp: " << std::setprecision(1) << state.temperature << "°C"
              << "    " << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "\n[MONITOR] Stopped." << std::endl;
  return 0;
}

// ──────────────────────────────────────────────────────────────
// Main
// ──────────────────────────────────────────────────────────────
void print_usage(const char* prog) {
  std::cout << "\nUsage: " << prog << " <robot-ip> <command> [args]\n\n"
            << "Commands:\n"
            << "  state       - Read and display gripper + robot state\n"
            << "  home        - Perform gripper homing (calibration)\n"
            << "  open        - Open gripper fully\n"
            << "  close       - Close gripper (grasp with low force)\n"
            << "  grasp       - Interactive grasp (prompts for width/force)\n"
            << "  move <mm>   - Move gripper to specific width in mm\n"
            << "  monitor     - Continuously print gripper state\n"
            << "  full_test   - Run full automated test sequence\n"
            << "\nExamples:\n"
            << "  " << prog << " 172.16.0.2 state\n"
            << "  " << prog << " 172.16.0.2 home\n"
            << "  " << prog << " 172.16.0.2 move 40\n"
            << "  " << prog << " 172.16.0.2 full_test\n\n";
}

int main(int argc, char** argv) {
  if (argc < 3) {
    print_usage(argv[0]);
    return -1;
  }

  std::string ip = argv[1];
  std::string cmd = argv[2];

  try {
    if (cmd == "state") {
      return cmd_state(ip);
    } else if (cmd == "home") {
      return cmd_home(ip);
    } else if (cmd == "open") {
      return cmd_open(ip);
    } else if (cmd == "close") {
      return cmd_close(ip);
    } else if (cmd == "grasp") {
      return cmd_grasp(ip);
    } else if (cmd == "move") {
      if (argc < 4) {
        std::cerr << "Error: 'move' requires a width in mm. E.g.: " << argv[0] 
                  << " " << ip << " move 40" << std::endl;
        return -1;
      }
      return cmd_move(ip, std::stod(argv[3]));
    } else if (cmd == "monitor") {
      return cmd_monitor(ip);
    } else if (cmd == "full_test") {
      return cmd_full_test(ip);
    } else {
      std::cerr << "Unknown command: " << cmd << std::endl;
      print_usage(argv[0]);
      return -1;
    }
  } catch (const franka::Exception& e) {
    std::cerr << "\n╔══════════════════════════════════════╗\n";
    std::cerr << "║  FRANKA EXCEPTION                    ║\n";
    std::cerr << "╠══════════════════════════════════════╣\n";
    std::cerr << "║  " << e.what() << std::endl;
    std::cerr << "╚══════════════════════════════════════╝\n";
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
