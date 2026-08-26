from __future__ import annotations

import argparse
import copy
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from franka_msgs.action import Grasp, Homing, Move
from franka_msgs.msg import FrankaRobotState
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from .storage import (
    append_gripper_step_to_sequence,
    append_step_to_sequence,
    initialize_sequence,
    list_named_yaml_files,
    load_yaml,
    point_file_path,
    resolve_storage_root,
    sanitize_name,
    save_point,
    sequence_file_path,
)


from .listener import (
    LiveStateListener, 
    Snapshot, 
    Spinner, 
    ROBOT_MODE_NAMES, 
    GUIDING_MODES,
    DEFAULT_GRIPPER_MAX_WIDTH,
    DEFAULT_GRIPPER_SPEED,
    DEFAULT_GRIPPER_FORCE
)
from .executor_client import TeachExecutorClient

DEFAULT_GRIPPER_GRASP_THRESHOLD = 0.005


def pose_to_matrix(pose) -> np.ndarray:
    from scipy.spatial.transform import Rotation as R

    transform = np.identity(4, dtype=np.float64)
    transform[0, 3] = pose.position.x
    transform[1, 3] = pose.position.y
    transform[2, 3] = pose.position.z
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    transform[:3, :3] = R.from_quat(quaternion).as_matrix()
    return transform


def matrix_to_dict_pose(transform: np.ndarray) -> dict[str, dict[str, float]]:
    from scipy.spatial.transform import Rotation as R

    quaternion = R.from_matrix(transform[:3, :3]).as_quat()
    return {
        "position": {
            "x": float(transform[0, 3]),
            "y": float(transform[1, 3]),
            "z": float(transform[2, 3]),
        },
        "orientation": {
            "x": float(quaternion[0]),
            "y": float(quaternion[1]),
            "z": float(quaternion[2]),
            "w": float(quaternion[3]),
        },
    }


def rotation_matrix_to_dict_orientation(rotation: np.ndarray) -> dict[str, float]:
    from scipy.spatial.transform import Rotation as R

    quaternion = R.from_matrix(rotation).as_quat()
    return {
        "x": float(quaternion[0]),
        "y": float(quaternion[1]),
        "z": float(quaternion[2]),
        "w": float(quaternion[3]),
    }


def yaw_from_rotation_matrix(rotation: np.ndarray) -> float:
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def rotation_about_z(yaw: float) -> np.ndarray:
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    return np.array(
        [
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def common_parser(parent: argparse.ArgumentParser) -> None:
    parent.add_argument(
        "--overwrite-points",
        action="store_true",
        help="Overwrite individual point files if they already exist during sequence capture.",
    )
    parent.add_argument(
        "--storage-dir",
        default=None,
        help="Directory where points/ and sequences/ will be stored.",
    )

    ros_group = parent.add_argument_group("ROS/Hardware Advanced (usually not needed)")
    ros_group.add_argument(
        "--joint-states-topic",
        default="/franka/joint_states",
        help="JointState topic to capture from.",
    )
    ros_group.add_argument(
        "--robot-state-topic",
        default=None,
        help="Optional FrankaRobotState topic for showing or enforcing guiding mode.",
    )
    ros_group.add_argument(
        "--gripper-joint-states-topic",
        default="/panda_gripper/joint_states",
        help="Gripper JointState topic for reading finger width.",
    )
    ros_group.add_argument(
        "--gripper-grasp-action",
        default="/panda_gripper/grasp",
        help="Gripper Grasp action server name.",
    )
    ros_group.add_argument(
        "--gripper-move-action",
        default="/panda_gripper/move",
        help="Gripper Move action server name.",
    )
    parent.add_argument(
        "--max-age-sec",
        type=float,
        default=1.0,
        help="Maximum allowed age of the latest joint state when capturing.",
    )
    parent.add_argument(
        "--wait-timeout-sec",
        type=float,
        default=10.0,
        help="How long to wait for the first JointState message.",
    )
    parent.add_argument(
        "--require-guiding",
        action="store_true",
        help="Require Franka robot mode GUIDING/USER_STOPPED before capture.",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Teach Franka joint-state points and sequences.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture_point = subparsers.add_parser("capture-point", help="Capture one named point.")
    common_parser(capture_point)
    capture_point.add_argument("name", help="Point name.")
    capture_point.add_argument(
        "--overwrite", action="store_true", help="Overwrite the point file if it already exists."
    )

    capture_sequence = subparsers.add_parser(
        "capture-sequence",
        help="Interactively capture an ordered sequence of discrete points.",
    )
    common_parser(capture_sequence)
    capture_sequence.add_argument("name", help="Sequence name.")
    capture_sequence.add_argument(
        "--overwrite", action="store_true", help="Overwrite the sequence file if it already exists."
    )

    append_current = subparsers.add_parser(
        "append-current",
        help="Capture the current joint state as a point and append it to a sequence.",
    )
    common_parser(append_current)
    append_current.add_argument("sequence_name", help="Sequence name.")
    append_current.add_argument("point_name", help="Point name to save and append.")
    append_current.add_argument(
        "--overwrite-point",
        action="store_true",
        help="Overwrite the point file if it already exists.",
    )

    list_points = subparsers.add_parser("list-points", help="List saved points.")
    list_points.add_argument("--storage-dir", default=None, help="Storage directory.")

    list_sequences = subparsers.add_parser("list-sequences", help="List saved sequences.")
    list_sequences.add_argument("--storage-dir", default=None, help="Storage directory.")

    show_point = subparsers.add_parser("show-point", help="Show a saved point YAML payload.")
    show_point.add_argument("name", help="Point name.")
    show_point.add_argument("--storage-dir", default=None, help="Storage directory.")

    show_sequence = subparsers.add_parser("show-sequence", help="Show a saved sequence YAML payload.")
    show_sequence.add_argument("name", help="Sequence name.")
    show_sequence.add_argument("--storage-dir", default=None, help="Storage directory.")
    
    capture_relative = subparsers.add_parser("capture-relative", help="Capture a point relative to an AprilTag marker.")
    common_parser(capture_relative)
    capture_relative.add_argument("name", help="Relative point name.")
    capture_relative.add_argument("--tag-size", type=float, default=0.06, help="AprilTag size in meters.")

    return parser


def print_record(record: dict[str, Any]) -> None:
    for key, value in record.items():
        print(f"{key}: {value}")


def capture_one(listener: LiveStateListener, args: argparse.Namespace) -> Snapshot:
    print(f"Waiting for joint states on {args.joint_states_topic} ...")
    snapshot = listener.capture_snapshot(
        timeout_sec=args.wait_timeout_sec,
        max_age_sec=args.max_age_sec,
        require_guiding=args.require_guiding,
    )
    if snapshot.robot_mode is not None:
        print(f"Captured with robot mode: {snapshot.robot_mode}")
    return snapshot


def run_capture_point(listener: LiveStateListener, args: argparse.Namespace, storage_root: Path) -> int:
    snapshot = capture_one(listener, args)
    point_path = save_point(storage_root, args.name, snapshot.payload, overwrite=args.overwrite)
    print(f"Saved point '{sanitize_name(args.name)}' to {point_path}")
    return 0


def run_capture_relative(listener: LiveStateListener, args: argparse.Namespace, storage_root: Path) -> int:
    # --- STEP 0: MOVE TO OBSERVATION POSE ---
    print("\n--- STEP 0: MOVE TO OBSERVATION POSE ---")
    print("Commanding robot to 'observation_pose' via executor...")
    try:
        client = TeachExecutorClient()
        result = client.play_point("observation_pose", plan_only=False)
        if not result.accepted or not result.result.success:
            print("  ✗ Failed to move to observation_pose automatically. Please check if 'observation_pose' exists.", file=sys.stderr)
        else:
            print("  ✓ Robot moved to observation_pose successfully.")
    except Exception as e:
        print(f"  ✗ Error invoking executor: {e}", file=sys.stderr)
    finally:
        try:
            client.destroy_node()
        except:
            pass

    listener.tag_size = args.tag_size
    print("\n--- STEP 1: MARKER SCAN ---")
    print("Opening video window... Ensure the AprilTag is visible.")
    print("-> Press SPACE in the video window once the tag is detected to LOCK it.")
    try:
        marker_pose = listener.detect_marker_pose_base(timeout_sec=120.0, show_window=True)
        print(f"  ✓ Marker locked at Base: [{marker_pose.pose.position.x:.3f}, {marker_pose.pose.position.y:.3f}, {marker_pose.pose.position.z:.3f}]")
    except InterruptedError:
        print("  ✗ Scan cancelled by user.")
        return 1
    except Exception as e:
        print(f"  ✗ Marker detection failed: {e}")
        return 1

    print("\n--- STEP 2: TEACH POINT ---")
    input("Now physically move the robot to the desired TEACH position, then press Enter...")
    try:
        ee_pose = listener.get_ee_pose_base()
        print(f"  ✓ Captured EE pose at Base: [{ee_pose.pose.position.x:.3f}, {ee_pose.pose.position.y:.3f}, {ee_pose.pose.position.z:.3f}]")
    except Exception as e:
        print(f"  ✗ EE capture failed: {e}")
        return 1

    # Save the point in a leveled marker frame:
    # - marker translation is followed normally
    # - only marker yaw is reused later
    # - marker roll/pitch are ignored during replay
    try:
        T_base_marker = pose_to_matrix(marker_pose.pose)
        T_base_ee = pose_to_matrix(ee_pose.pose)
        marker_yaw = yaw_from_rotation_matrix(T_base_marker[:3, :3])

        T_base_marker_yaw_only = np.identity(4, dtype=np.float64)
        T_base_marker_yaw_only[:3, :3] = rotation_about_z(marker_yaw)
        T_base_marker_yaw_only[:3, 3] = T_base_marker[:3, 3]

        T_marker_yaw_only_ee = np.linalg.inv(T_base_marker_yaw_only) @ T_base_ee
        relative_position = T_marker_yaw_only_ee[:3, 3]

        relative_pose_payload = {
            "type": "cartesian_relative",
            "frame_id": "marker",
            "relative_motion_mode": "marker_yaw_only",
            "relative_position_frame": "marker_yaw_only_frame",
            "relative_position": {
                "x": float(relative_position[0]),
                "y": float(relative_position[1]),
                "z": float(relative_position[2]),
            },
            "marker_yaw_at_teach_rad": float(marker_yaw),
            "reference_orientation": rotation_matrix_to_dict_orientation(T_base_ee[:3, :3]),
            "marker_pose_at_teach": matrix_to_dict_pose(T_base_marker),
            "marker_pose_at_teach_yaw_only": matrix_to_dict_pose(T_base_marker_yaw_only),
            "ee_pose_at_teach": matrix_to_dict_pose(T_base_ee),
            "calibration_convention": "base_to_marker = base_to_ee @ camera_to_gripper @ marker_to_camera",
        }
        
        # Also include joint positions for safety/reference
        snapshot = listener.capture_snapshot(timeout_sec=2.0, max_age_sec=5.0, require_guiding=False)
        relative_pose_payload.update(snapshot.payload)

        point_path = save_point(storage_root, args.name, relative_pose_payload, overwrite=True)
        print(f"\n  ✓ Success! Saved RELATIVE point '{args.name}' to {point_path}")
        print("  This point will follow marker translation and yaw only during execution.")
        
    except Exception as e:
        print(f"  ✗ Calculation/Save failed: {e}")
        return 1

    # Optional Robustness Check
    print("\n--- STEP 3: ROBUSTNESS CHECK (Optional) ---")
    check = input("Would you like to move the arm back to SCAN position to verify the marker hasn't moved? (y/n): ")
    if check.lower() == 'y':
        try:
             marker_now = listener.detect_marker_pose_base(timeout_sec=30.0, show_window=True)
             dist = np.linalg.norm(np.array([marker_now.pose.position.x - marker_pose.pose.position.x,
                                             marker_now.pose.position.y - marker_pose.pose.position.y,
                                             marker_now.pose.position.z - marker_pose.pose.position.z]))
             if dist < 0.005: # 5mm
                 print(f"  ✓ Verified! Marker moved only {dist*1000:.1f}mm. Teaching is solid.")
             else:
                 print(f"  ⚠ WARNING: Marker moved {dist*1000:.1f}mm during teaching. Consider re-teaching!")
        except Exception:
             print("  ✗ Verification failed (marker not found).")

    return 0


def run_append_current(listener: LiveStateListener, args: argparse.Namespace, storage_root: Path) -> int:
    snapshot = capture_one(listener, args)
    point_path = save_point(
        storage_root, args.point_name, snapshot.payload, overwrite=args.overwrite_point
    )
    sequence_path = append_step_to_sequence(
        storage_root,
        sequence_name=args.sequence_name,
        point_name=args.point_name,
        joint_states_topic=args.joint_states_topic,
    )
    print(f"Saved point '{sanitize_name(args.point_name)}' to {point_path}")
    print(f"Updated sequence '{sanitize_name(args.sequence_name)}' at {sequence_path}")
    return 0


def run_capture_sequence(listener: LiveStateListener, args: argparse.Namespace, storage_root: Path) -> int:
    sequence_name = sanitize_name(args.name)
    sequence_path = initialize_sequence(storage_root, sequence_name, overwrite=args.overwrite)
    print(f"Created sequence '{sequence_name}' at {sequence_path}")
    print("Move the robot by hand, then press Enter to capture the next point.")
    print("Commands:  'open'  = open gripper,  'close' = close/grasp gripper")
    print("           'home'  = calibrate/home gripper")
    print("           'done'  = finish,  or type a custom point name")

    # If sequence is overwritten, default to overwriting points too for a clean re-teach
    overwrite_points = args.overwrite or args.overwrite_points
    if overwrite_points:
        print("  (Note: Existing point files will be overwritten during this session)")

    index = 1
    while True:
        suggested_name = f"{sequence_name}_{index:03d}"
        user_input = input(f"\nStep [{suggested_name}]> ").strip()
        if user_input.lower() in {"done", "quit", "q"}:
            break

        # --- Gripper HOME ---
        if user_input.lower() == "home":
            try:
                success = listener.command_gripper_home()
                if success:
                    print("  ✓ Gripper HOMED successfully")
                    # We don't record homing in the sequence by default, 
                    # but we could if the user wanted it. 
                else:
                    print("  ✗ Gripper homing failed (check hardware)")
            except (RuntimeError, TimeoutError) as err:
                print(f"  ✗ Gripper homing error: {err}")
            continue

        # --- Gripper OPEN ---
        if user_input.lower() == "open":
            try:
                final_width = listener.command_gripper_open()
                append_gripper_step_to_sequence(
                    storage_root,
                    sequence_name=sequence_name,
                    gripper_action="open",
                    gripper_width=final_width,
                )
                print(f"  ✓ Gripper OPENED (width={final_width:.4f}m) — recorded in sequence")
            except (RuntimeError, TimeoutError) as err:
                print(f"  ✗ Gripper open failed: {err}")
                continue
            index += 1
            continue

        # --- Gripper CLOSE ---
        if user_input.lower().startswith("close"):
            parts = user_input.split()
            force = DEFAULT_GRIPPER_FORCE
            if len(parts) > 1:
                try:
                    force = float(parts[1])
                except ValueError:
                    print(f"  ✗ Invalid force value: {parts[1]}")
                    continue

            try:
                final_width = listener.command_gripper_close(force=force)
                append_gripper_step_to_sequence(
                    storage_root,
                    sequence_name=sequence_name,
                    gripper_action="grasp",
                    gripper_width=final_width,
                    gripper_force=force,
                )
                print(f"  ✓ Gripper CLOSED (width={final_width:.4f}m, force={force:.1f}N) — recorded in sequence")
            except (RuntimeError, TimeoutError) as err:
                print(f"  ✗ Gripper close failed: {err}")
                continue
            index += 1
            continue

        # --- Normal arm point capture ---
        point_name = sanitize_name(user_input) if user_input else suggested_name
        snapshot = capture_one(listener, args)
        save_point(storage_root, point_name, snapshot.payload, overwrite=overwrite_points)
        append_step_to_sequence(
            storage_root,
            sequence_name=sequence_name,
            point_name=point_name,
            joint_states_topic=args.joint_states_topic,
        )
        print(f"  ✓ Added point '{point_name}' to sequence '{sequence_name}'")
        index += 1

    sequence_record = load_yaml(sequence_file_path(storage_root, sequence_name))
    steps = sequence_record.get("steps", [])
    print(f"\nFinished sequence '{sequence_name}' with {len(steps)} step(s).")
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    storage_root = resolve_storage_root(getattr(args, "storage_dir", None))

    if args.command == "list-points":
        for name in list_named_yaml_files(storage_root / "points"):
            print(name)
        return 0

    if args.command == "list-sequences":
        for name in list_named_yaml_files(storage_root / "sequences"):
            print(name)
        return 0

    if args.command == "show-point":
        print_record(load_yaml(point_file_path(storage_root, args.name)))
        return 0

    if args.command == "show-sequence":
        print_record(load_yaml(sequence_file_path(storage_root, args.name)))
        return 0

    rclpy.init(args=None)
    gripper_topic = getattr(args, "gripper_joint_states_topic", None)
    gripper_grasp_action = getattr(args, "gripper_grasp_action", "/panda_gripper/grasp")
    gripper_move_action = getattr(args, "gripper_move_action", "/panda_gripper/move")
    listener = LiveStateListener(
        args.joint_states_topic,
        args.robot_state_topic,
        gripper_topic,
        gripper_grasp_action,
        gripper_move_action,
    )
    spinner = Spinner(listener)
    spinner.start()

    try:
        if args.command == "capture-point":
            return run_capture_point(listener, args, storage_root)
        if args.command == "capture-sequence":
            return run_capture_sequence(listener, args, storage_root)
        if args.command == "append-current":
            return run_append_current(listener, args, storage_root)
        if args.command == "capture-relative":
            return run_capture_relative(listener, args, storage_root)
        parser.error(f"Unsupported command: {args.command}")
    except (FileExistsError, RuntimeError, TimeoutError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    finally:
        spinner.stop()
        listener.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
