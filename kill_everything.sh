#!/bin/bash

echo "=================================================="
echo "      CLEANING UP ALL RUNNING SYSTEM COMPONENTS    "
echo "=================================================="

echo "1. Stopping ROS 2 daemon..."
ros2 daemon stop 2>/dev/null

echo "2. Releasing any processes holding USB/RealSense camera devices..."
# Terminate any process accessing video or media descriptors to free the hardware locks
for dev in /dev/video* /dev/media*; do
    if [ -e "$dev" ]; then
        echo "   Releasing $dev"
        fuser -k -9 "$dev" 2>/dev/null
    fi
done

echo "3. Terminating all ROS 2, Python, and controller processes..."
PROCESSES=(
    "system_monitor.py"
    "iwt_bandlaufsystem_node"
    "rviz2"
    "move_group"
    "franka_teach_executor"
    "ros2_control_node"
    "controller_manager"
    "realsense2_camera"
    "joint_state_publisher"
    "robot_state_publisher"
    "franka_gripper"
    "franka_fault_manager"
    "spawner"
    "robot_to_camera_bridge"
)

for proc in "${PROCESSES[@]}"; do
    pkill -9 -f "$proc" 2>/dev/null
done

# Terminate any other generic ROS node wrapper executables
pkill -9 -f "_node" 2>/dev/null

echo "4. Checking remaining processes..."
ps aux | grep -E "system_monitor|iwt_bandlauf|franka_teach|move_group|rviz2" | grep -v grep

echo "=================================================="
echo " Cleanup complete! You can start fresh now.       "
echo "=================================================="
