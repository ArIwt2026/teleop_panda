#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from franka_msgs.msg import FrankaRobotState
from controller_manager_msgs.srv import ListControllers, SwitchController

class FrankaFaultManager(Node):
    def __init__(self):
        super().__init__('franka_fault_manager')
        self.get_logger().info("Initializing Franka Fault Recovery Manager...")

        # State tracking variables
        self.was_faulted = False
        self.active_controllers_to_restart = []
        self.robot_mode_map = {
            FrankaRobotState.ROBOT_MODE_OTHER: "OTHER",
            FrankaRobotState.ROBOT_MODE_IDLE: "IDLE",
            FrankaRobotState.ROBOT_MODE_MOVE: "MOVE",
            FrankaRobotState.ROBOT_MODE_GUIDING: "GUIDING",
            FrankaRobotState.ROBOT_MODE_REFLEX: "REFLEX (FAULTED)",
            FrankaRobotState.ROBOT_MODE_USER_STOPPED: "USER STOPPED",
            FrankaRobotState.ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY: "AUTOMATIC ERROR RECOVERY"
        }

        # Initialize ROS 2 Clients
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

        # Wait for services to be ready
        self.get_logger().info("Connecting to controller_manager services...")
        while not self.list_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                sys.exit(0)
            self.get_logger().info("Waiting for '/controller_manager/list_controllers' service...")
        while not self.switch_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                sys.exit(0)
            self.get_logger().info("Waiting for '/controller_manager/switch_controller' service...")
        self.get_logger().info("Connected to controller_manager services.")

        # Subscribe to robot state topic
        self.state_sub = self.create_subscription(
            FrankaRobotState,
            '/franka_robot_state_broadcaster/robot_state',
            self.robot_state_callback,
            10
        )
        self.get_logger().info("Subscribed to '/franka_robot_state_broadcaster/robot_state'. Monitoring started.")
        print("\n\033[96m=== Franka Fault Manager is running. Press Ctrl+C to exit. ===\033[0m\n")

    def robot_state_callback(self, msg):
        # 1. Parse active error flags dynamically
        active_errors = []
        if hasattr(msg, 'current_errors'):
            for slot in msg.current_errors.__slots__:
                val = getattr(msg.current_errors, slot)
                if isinstance(val, bool) and val:
                    clean_name = slot.lstrip('_')
                    active_errors.append(clean_name)

        # 2. Determine if the robot is in a faulted/stopped mode or has error flags
        is_faulted = (
            msg.robot_mode in [FrankaRobotState.ROBOT_MODE_REFLEX, FrankaRobotState.ROBOT_MODE_USER_STOPPED]
            or len(active_errors) > 0
        )

        mode_str = self.robot_mode_map.get(msg.robot_mode, f"UNKNOWN ({msg.robot_mode})")

        # 3. Transition: Not Faulted -> Faulted
        if is_faulted and not self.was_faulted:
            self.was_faulted = True
            
            # Print high-visibility colorized red banner for error reporting
            print("\n\033[91m" + "!" * 80)
            print(" [FAULT DETECTED] ROBOT ENTERED REFLEX / FAULT STATE!")
            print(f"  Robot Mode:    {mode_str}")
            print(f"  Active Errors: {active_errors if active_errors else 'None'}")
            print("!" * 80 + "\033[0m\n")
            
            # Query active controllers and immediately stop/deactivate them
            self.query_and_deactivate_controllers()

        # 4. Transition: Faulted -> Idle (Fault has been cleared in Desk UI)
        elif not is_faulted and msg.robot_mode == FrankaRobotState.ROBOT_MODE_IDLE and self.was_faulted:
            self.was_faulted = False
            print("\n\033[92m" + "=" * 80)
            print(" [FAULT CLEARED] Robot state is now IDLE.")
            print("=" * 80 + "\033[0m\n")
            
            # Reactivate the previously deactivated controllers
            self.reactivate_controllers()

    def query_and_deactivate_controllers(self):
        req = ListControllers.Request()
        future = self.list_client.call_async(req)
        future.add_done_callback(self.list_and_deactivate_callback)

    def list_and_deactivate_callback(self, future):
        try:
            response = future.result()
            self.active_controllers_to_restart = []
            for c in response.controller:
                if c.state == 'active':
                    # Exclude read-only broadcasters
                    if 'broadcaster' not in c.name and 'state' not in c.name:
                        self.active_controllers_to_restart.append(c.name)
            
            if self.active_controllers_to_restart:
                self.get_logger().info(
                    f"Deactivating active controllers to abort MoveIt/RViz actions: {self.active_controllers_to_restart}"
                )
                self.deactivate_controllers()
            else:
                self.get_logger().info("No active motion/control controllers found to deactivate.")
        except Exception as e:
            self.get_logger().error(f"Failed to list controllers for deactivation: {e}")

    def deactivate_controllers(self):
        req = SwitchController.Request()
        req.deactivate_controllers = self.active_controllers_to_restart
        req.activate_controllers = []
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        
        future = self.switch_client.call_async(req)
        future.add_done_callback(self.deactivate_done_callback)

    def deactivate_done_callback(self, future):
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info("Successfully deactivated controllers. Stale commands aborted in MoveIt.")
                self.get_logger().info("Waiting for the fault to be cleared in the Desk web interface...")
            else:
                self.get_logger().error("Failed to deactivate controllers.")
        except Exception as e:
            self.get_logger().error(f"Error calling SwitchController service for deactivation: {e}")

    def reactivate_controllers(self):
        if not self.active_controllers_to_restart:
            self.get_logger().info("No controllers recorded to reactivate. Robot is ready.")
            return

        self.get_logger().info(f"Reactivating controllers: {self.active_controllers_to_restart} ...")
        
        req = SwitchController.Request()
        req.deactivate_controllers = []
        req.activate_controllers = self.active_controllers_to_restart
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        
        future = self.switch_client.call_async(req)
        future.add_done_callback(self.reactivate_done_callback)

    def reactivate_done_callback(self, future):
        try:
            response = future.result()
            if response.ok:
                print("\n\033[92m" + "*" * 80)
                print(" SUCCESS: Controllers reactivated and aligned to current pose.")
                print(" Robot is ready for new commands in MoveIt/RViz!")
                print("*" * 80 + "\033[0m\n")
            else:
                self.get_logger().error("Failed to reactivate controllers.")
        except Exception as e:
            self.get_logger().error(f"Error calling SwitchController service for reactivation: {e}")
        finally:
            self.active_controllers_to_restart = []

def main(args=None):
    rclpy.init(args=args)
    node = FrankaFaultManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
