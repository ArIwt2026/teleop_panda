import rclpy
from rclpy.executors import MultiThreadedExecutor

from .coordinator_node import HandEyeCalibrationCoordinator


def main(args=None) -> int:
    rclpy.init(args=args)
    node = HandEyeCalibrationCoordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
