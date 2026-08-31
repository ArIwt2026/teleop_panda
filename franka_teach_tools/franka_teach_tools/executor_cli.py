from __future__ import annotations

import argparse
import sys

import rclpy
from geometry_msgs.msg import PoseStamped

from .executor_client import TeachExecutorClient
from .listener import LiveStateListener, Spinner


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CLI client for franka_teach_executor.")
    parser.add_argument(
        "--executor-namespace",
        default="/franka_teach_executor",
        help="Executor node namespace or absolute name prefix.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    play_point = subparsers.add_parser("play-point", help="Plan or execute one saved point.")
    play_point.add_argument("name", help="Point name.")
    play_point.add_argument("--plan-only", action="store_true", help="Only plan, do not execute.")
    play_point.add_argument("--velocity-scale", type=float, default=-1.0)
    play_point.add_argument("--acceleration-scale", type=float, default=-1.0)
    play_point.add_argument("--planning-time", type=float, default=-1.0)
    play_point.add_argument("--goal-tolerance", type=float, default=-1.0)
    play_point.add_argument("--planner-id", type=str, default="PTP", choices=["PTP", "LIN"], help="Planner ID to use (PTP or LIN).")
    play_point.add_argument("--base-only", action="store_true", help="Rotate only panda_joint1, keeping all other joints identical to current configuration.")
    play_point.add_argument("--ref-point", type=str, default="", help="Golden Reference point name to copy joints 2-7 from.")
    
    play_relative = subparsers.add_parser(
        "play-relative",
        help="Plan or execute a point relative to a live AprilTag. Defaults to planning only.",
    )
    play_relative.add_argument("name", help="Relative point name.")
    play_relative.add_argument("--plan-only", action="store_true", help="Only plan, do not execute.")
    play_relative.add_argument(
        "--execute",
        action="store_true",
        help="Actually execute after planning. Without this flag the command stays in plan-only mode.",
    )
    play_relative.add_argument("--tag-size", type=float, default=0.06, help="AprilTag size in meters.")
    play_relative.add_argument("--mode", default="move", choices=["move", "pick", "place"], help="Motion mode.")
    play_relative.add_argument("--approach-height", type=float, default=-1.0)
    play_relative.add_argument("--gripper-width", type=float, default=-1.0)
    play_relative.add_argument("--gripper-force", type=float, default=-1.0)
    play_relative.add_argument("--gripper-speed", type=float, default=-1.0)
    play_relative.add_argument("--velocity-scale", type=float, default=-1.0)
    play_relative.add_argument("--acceleration-scale", type=float, default=-1.0)
    play_relative.add_argument("--planner-id", type=str, default="PTP", choices=["PTP", "LIN"], help="Planner ID to use (PTP or LIN).")
    play_relative.add_argument("--skip-observation-pose", action="store_true", help="Skip observation pose and marker scan, reuse last cached marker pose from C++ executor memory.")
    play_relative.add_argument("--pipeline", type=str, default="", help="Planning pipeline ID to use (e.g. pilz).")

    scan_relative = subparsers.add_parser(
        "scan-relative",
        help="Scan and cache the live AprilTag pose in C++ executor memory for later pick/place.",
    )
    scan_relative.add_argument("--tag-size", type=float, default=0.06, help="AprilTag size in meters.")

    go_camera_pose = subparsers.add_parser(
        "go-camera-pose",
        help="Plan or execute one camera-frame pose target directly through the executor.",
    )
    go_camera_pose.add_argument("--frame", default="panda_hand_camera_link", help="Camera frame id.")
    go_camera_pose.add_argument("--x", type=float, required=True)
    go_camera_pose.add_argument("--y", type=float, required=True)
    go_camera_pose.add_argument("--z", type=float, required=True)
    go_camera_pose.add_argument("--qx", type=float, default=0.0)
    go_camera_pose.add_argument("--qy", type=float, default=0.0)
    go_camera_pose.add_argument("--qz", type=float, default=0.0)
    go_camera_pose.add_argument("--qw", type=float, default=1.0)
    go_camera_pose.add_argument("--plan-only", action="store_true", help="Only plan, do not execute.")
    go_camera_pose.add_argument("--velocity-scale", type=float, default=-1.0)
    go_camera_pose.add_argument("--acceleration-scale", type=float, default=-1.0)
    go_camera_pose.add_argument("--planning-time", type=float, default=-1.0)
    go_camera_pose.add_argument("--goal-tolerance", type=float, default=-1.0)

    play_sequence = subparsers.add_parser("play-sequence", help="Plan or execute a saved sequence.")
    play_sequence.add_argument("name", help="Sequence name.")
    play_sequence.add_argument("--plan-only", action="store_true", help="Only plan, do not execute.")
    play_sequence.add_argument("--start-step-index", type=int, default=0)
    play_sequence.add_argument(
        "--force-wait-each-step",
        action="store_true",
        help="Force the executor to wait for continue after every step.",
    )

    play_pick = subparsers.add_parser("play-pick", help="Execute the pick macro (Approach -> Descent -> Grasp -> Lift-off -> Retract).")
    play_pick.add_argument("name", help="Point name.")
    play_pick.add_argument("--approach-height", type=float, default=-1.0, help="Retraction height along tool axis (m)")
    play_pick.add_argument("--lift-off-height", type=float, default=-1.0, help="Vertical lift in global Z after grasp (m)")
    play_pick.add_argument("--gripper-width", type=float, default=-1.0)
    play_pick.add_argument("--gripper-force", type=float, default=-1.0)
    play_pick.add_argument("--gripper-speed", type=float, default=-1.0)
    play_pick.add_argument("--plan-only", action="store_true")
    play_pick.add_argument("--velocity-scale", type=float, default=-1.0)
    play_pick.add_argument("--acceleration-scale", type=float, default=-1.0)
    play_pick.add_argument("--planner-id", type=str, default="PTP", choices=["PTP", "LIN"], help="Planner ID to use for Phase 1 (Approach) - PTP or LIN.")

    play_place = subparsers.add_parser("play-place", help="Execute the place macro (Approach+Lift -> Descent to Lift -> Final Descent -> Open -> Retract).")
    play_place.add_argument("name", help="Point name.")
    play_place.add_argument("--approach-height", type=float, default=-1.0, help="Retraction height along tool axis (m)")
    play_place.add_argument("--lift-off-height", type=float, default=-1.0, help="Extra vertical clearance in global Z (m)")
    play_place.add_argument("--gripper-width", type=float, default=-1.0)
    play_place.add_argument("--gripper-force", type=float, default=-1.0)
    play_place.add_argument("--gripper-speed", type=float, default=-1.0)
    play_place.add_argument("--place-descent-force", type=float, default=-1.0, help="Optional force to use when descending to place an object. 0.0 or negative means do not loosen grip.")
    play_place.add_argument("--plan-only", action="store_true")
    play_place.add_argument("--velocity-scale", type=float, default=-1.0)
    play_place.add_argument("--acceleration-scale", type=float, default=-1.0)
    play_place.add_argument("--planner-id", type=str, default="PTP", choices=["PTP", "LIN"], help="Planner ID to use for Phase 1 (Approach) - PTP or LIN.")

    step_sequence = subparsers.add_parser("step-sequence", help="Step through a sequence interactively (manual mode).")
    step_sequence.add_argument("name", help="Sequence name.")
    step_sequence.add_argument("--start-step-index", type=int, default=0)
    step_sequence.add_argument("--plan-only", action="store_true", help="Only plan, do not execute.")

    subparsers.add_parser("pause", help="Request a soft pause.")
    subparsers.add_parser("continue", help="Continue from pause or wait state.")
    subparsers.add_parser("stop", help="Request a hard stop.")
    subparsers.add_parser("recover", help="Request reflex recovery.")
    subparsers.add_parser("status", help="Show executor status.")

    subparsers.add_parser(
        "scan-workpieces",
        help="Command robot to automatically move to orientation_pose and run OpenCV workpiece slot checking.",
    )

    clear_slot = subparsers.add_parser(
        "clear-slot",
        help="Update memory in C++ executor to set a specific slot's status to EMPTY.",
    )
    clear_slot.add_argument("slot_id", type=int, choices=[1, 2, 3, 4], help="Slot index to clear (1-4).")

    subparsers.add_parser(
        "clear-all-slots",
        help="Update memory in C++ executor to set all workpiece slots' statuses to EMPTY.",
    )

    subparsers.add_parser(
        "place-to-conveyor",
        help="Execute LIN move to standby then LIN move to c1 (place to AGV area to conveyor belt).",
    )

    return parser


def print_status(status) -> None:
    print(f"executor_state: {status.executor_state}")
    print(f"active_sequence_name: {status.active_sequence_name}")
    print(f"active_point_name: {status.active_point_name}")
    print(f"active_step_index: {status.active_step_index}")
    print(f"total_steps: {status.total_steps}")
    print(f"completed_steps: {status.completed_steps}")
    print(f"failed_step_index: {status.failed_step_index}")
    print(f"failed_point_name: {status.failed_point_name}")
    print(f"last_error: {status.last_error}")
    print(f"busy: {status.busy}")
    print(f"pause_requested: {status.pause_requested}")
    print(f"waiting_for_continue: {status.waiting_for_continue}")
    print(f"fault_reflex: {status.fault_reflex}")
    print(f"fault_stopped: {status.fault_stopped}")
    print(f"recover_required: {status.recover_required}")
    print(f"stop_requested: {status.stop_requested}")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    rclpy.init(args=None)
    client = TeachExecutorClient(executor_namespace=args.executor_namespace)

    try:
        if args.command == "play-point":
            result = client.play_point(
                args.name,
                plan_only=args.plan_only,
                velocity_scale=args.velocity_scale,
                acceleration_scale=args.acceleration_scale,
                planning_time=args.planning_time,
                goal_tolerance=args.goal_tolerance,
                planner_id=args.planner_id,
                base_only=args.base_only,
                ref_point_name=args.ref_point,
            )
            if not result.accepted:
                print("Action goal was rejected.", file=sys.stderr)
                return 1
            print(f"result_code: {result.result_code}")
            print(f"success: {result.result.success}")
            print(f"final_state: {result.result.final_state}")
            print(f"error_message: {result.result.error_message}")
            print_status(result.result.final_status)
            return 0 if result.result.success else 1

        if args.command in ["play-pick", "play-place"]:
            mode = "pick" if args.command == "play-pick" else "place"
            place_descent_force = getattr(args, "place_descent_force", -1.0)
            result = client.pick_place(
                args.name,
                mode=mode,
                approach_height=args.approach_height,
                lift_off_height=args.lift_off_height,
                gripper_width=args.gripper_width,
                gripper_force=args.gripper_force,
                gripper_speed=args.gripper_speed,
                place_descent_force=place_descent_force,
                plan_only=args.plan_only,
                velocity_scale=args.velocity_scale,
                acceleration_scale=args.acceleration_scale,
                planner_id=args.planner_id,
            )
            if not result.accepted:
                print("Action goal was rejected.", file=sys.stderr)
                return 1
            print(f"result_code: {result.result_code}")
            print(f"success: {result.result.success}")
            print(f"final_state: {result.result.final_state}")
            print(f"error_message: {result.result.error_message}")
            print_status(result.result.final_status)
            return 0 if result.result.success else 1

        if args.command == "play-relative":
            marker_pose = PoseStamped()
            is_place_mode = (args.mode == "place")
            skip_scan = args.skip_observation_pose or is_place_mode

            if not skip_scan:
                # 0. Automatically move to observation_pose
                print(f"\n--- STEP 0: MOVE TO OBSERVATION POSE ---")
                print("Commanding robot to 'observation_pose' via executor...")
                try:
                    res = client.play_point("observation_pose", plan_only=False)
                    if not res.accepted or not res.result.success:
                        print("  ✗ Failed to move to observation_pose automatically.", file=sys.stderr)
                    else:
                        print("  ✓ Robot moved to observation_pose successfully.")
                except Exception as e:
                    print(f"  ✗ Error calling executor: {e}", file=sys.stderr)

                # 1. Scan for the marker first
                print(f"\n--- STEP 1: MARKER SCAN for '{args.name}' ---")
                print("Opening video window... Ensure the AprilTag is visible.")
                print("-> Press SPACE in the video window once the tag is detected to LOCK it.")
                
                listener = LiveStateListener(
                    joint_states_topic="/franka/joint_states",
                    robot_state_topic="/franka_robot_state_broadcaster/robot_state",
                )
                listener.tag_size = args.tag_size
                spinner = Spinner(listener)
                spinner.start()
                
                try:
                    marker_pose = listener.detect_marker_pose_base(show_window=True)
                    print(f"  ✓ Marker locked at Base: [{marker_pose.pose.position.x:.3f}, {marker_pose.pose.position.y:.3f}, {marker_pose.pose.position.z:.3f}]")
                except Exception as e:
                    print(f"  ✗ Marker scan failed: {e}", file=sys.stderr)
                    spinner.stop()
                    return 1
                
                spinner.stop()
            else:
                if is_place_mode:
                    print(f"\n--- PLACE MODE: SKIPPING SCAN (Camera is blocked, using cached marker) ---")
                else:
                    print(f"\n--- STEP 0 & 1: SKIPPED (Reusing cached pose in C++ executor memory) ---")

            # 2. Execute with the detected pose
            effective_plan_only = args.plan_only or not args.execute
            if effective_plan_only:
                print("  i Planning only. Pass --execute once the RViz ghost looks correct.")

            result = client.execute_relative_point(
                args.name,
                marker_pose=marker_pose,
                mode=args.mode,
                approach_height=args.approach_height,
                gripper_width=args.gripper_width,
                gripper_force=args.gripper_force,
                gripper_speed=args.gripper_speed,
                plan_only=effective_plan_only,
                velocity_scale=args.velocity_scale,
                acceleration_scale=args.acceleration_scale,
                skip_observation_pose=skip_scan,
                planner_id=args.planner_id,
                planning_pipeline_id=args.pipeline,
            )
            if not result.accepted:
                print("Action goal was rejected.", file=sys.stderr)
                return 1

            if not result.result.success:
                if "No cached AprilTag pose available" in result.result.error_message:
                    print(f"\n  ✗ Error: Cached AprilTag pose is not available in executor memory!", file=sys.stderr)
                    print(f"    Please perform a relative pick scan first to cache the AprilTag pose.\n", file=sys.stderr)

            print(f"result_code: {result.result_code}")
            print(f"success: {result.result.success}")
            print(f"final_state: {result.result.final_state}")
            print(f"error_message: {result.result.error_message}")
            print_status(result.result.final_status)
            return 0 if result.result.success else 1

        if args.command == "scan-relative":
            # 0. Automatically move to observation_pose
            print(f"\n--- STEP 0: MOVE TO OBSERVATION POSE ---")
            print("Commanding robot to 'observation_pose' via executor...")
            try:
                res = client.play_point("observation_pose", plan_only=False)
                if not res.accepted or not res.result.success:
                    print("  ✗ Failed to move to observation_pose automatically.", file=sys.stderr)
                else:
                    print("  ✓ Robot moved to observation_pose successfully.")
            except Exception as e:
                print(f"  ✗ Error calling executor: {e}", file=sys.stderr)

            # 1. Scan for the marker
            print(f"\n--- STEP 1: MARKER SCAN ---")
            print("Opening video window... Ensure the AprilTag is visible.")
            print("-> Press SPACE in the video window once the tag is detected to LOCK it.")
            
            listener = LiveStateListener(
                joint_states_topic="/franka/joint_states",
                robot_state_topic="/franka_robot_state_broadcaster/robot_state",
            )
            listener.tag_size = args.tag_size
            spinner = Spinner(listener)
            spinner.start()
            
            try:
                marker_pose = listener.detect_marker_pose_base(show_window=True)
                print(f"  ✓ Marker locked at Base: [{marker_pose.pose.position.x:.3f}, {marker_pose.pose.position.y:.3f}, {marker_pose.pose.position.z:.3f}]")
            except Exception as e:
                print(f"  ✗ Marker scan failed: {e}", file=sys.stderr)
                spinner.stop()
                return 1
            
            spinner.stop()

            # 2. Register/cache in C++ executor memory
            print(f"\n--- STEP 2: CACHING IN C++ EXECUTOR MEMORY ---")
            result = client.execute_relative_point(
                "scan_only",
                marker_pose=marker_pose,
                plan_only=True
            )
            if not result.accepted or not result.result.success:
                print("  ✗ Failed to register AprilTag pose in executor memory.", file=sys.stderr)
                return 1
            print("  ✓ AprilTag pose successfully scanned and cached in C++ executor memory!")
            return 0

        if args.command in ["play-sequence", "step-sequence"]:
            force_wait = is_step_mode or (hasattr(args, "force_wait_each_step") and args.force_wait_each_step)
            
            # For step-mode, we don't want to block in the client send_goal
            # because we need to interactively call "continue"
            goal_handle_future = client._play_sequence_client.send_goal_async(
                PlaySequence.Goal(
                    sequence_name=args.name,
                    plan_only=args.plan_only,
                    start_step_index=args.start_step_index,
                    force_wait_each_step=force_wait
                )
            )
            
            rclpy.spin_until_future_complete(client, goal_handle_future)
            goal_handle = goal_handle_future.result()
            
            if not goal_handle or not goal_handle.accepted:
                print("Action goal was rejected.", file=sys.stderr)
                return 1
                
            print(f"Goal accepted. Executing sequence '{args.name}'" + (" in STEP mode" if is_step_mode else ""))
            
            result_future = goal_handle.get_result_async()
            last_notified_step = -1
            
            import time
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(client, timeout_sec=0.1)
                
                if is_step_mode:
                    status = client.get_executor_status()
                    
                    if status.active_step_index != last_notified_step:
                        if status.active_step_index > 0:
                            print(f"\n[Step {status.active_step_index}/{status.total_steps}] "
                                  f"Active: {status.active_point_name or 'Gripper action'}")
                        last_notified_step = status.active_step_index
                    
                    if status.waiting_for_continue:
                        input(">> Robot paused. Press [ENTER] to continue to next step...")
                        client.continue_execution()
                        # Small sleep to let the state transition
                        time.sleep(0.5)

            rclpy.spin_until_future_complete(client, result_future)
            action_result = result_future.result()
            result_code = action_result.status
            result = action_result.result
            
            print(f"\nresult_code: {result_code}")
            print(f"success: {result.success}")
            print(f"final_state: {result.final_state}")
            if result.error_message:
                print(f"error_message: {result.error_message}")
            print_status(result.final_status)
            return 0 if result.success else 1

        if args.command == "go-camera-pose":
            target_pose = PoseStamped()
            target_pose.header.frame_id = args.frame
            target_pose.pose.position.x = args.x
            target_pose.pose.position.y = args.y
            target_pose.pose.position.z = args.z
            target_pose.pose.orientation.x = args.qx
            target_pose.pose.orientation.y = args.qy
            target_pose.pose.orientation.z = args.qz
            target_pose.pose.orientation.w = args.qw

            result = client.go_to_camera_pose(
                target_pose,
                plan_only=args.plan_only,
                velocity_scale=args.velocity_scale,
                acceleration_scale=args.acceleration_scale,
                planning_time=args.planning_time,
                goal_tolerance=args.goal_tolerance,
            )
            if not result.accepted:
                print("Action goal was rejected.", file=sys.stderr)
                return 1
            print(f"result_code: {result.result_code}")
            print(f"success: {result.result.success}")
            print(f"final_state: {result.result.final_state}")
            print(f"target_frame: {result.result.target_frame}")
            print(f"error_message: {result.result.error_message}")
            transformed = result.result.transformed_target_pose.pose.position
            print(
                "transformed_target_pose: "
                f"{transformed.x:.4f} {transformed.y:.4f} {transformed.z:.4f}"
            )
            print_status(result.result.final_status)
            return 0 if result.result.success else 1

        if args.command == "pause":
            response = client.pause_execution()
            print(response.message)
            return 0 if response.success else 1

        if args.command == "continue":
            response = client.continue_execution()
            print(response.message)
            return 0 if response.success else 1

        if args.command == "stop":
            response = client.stop_execution()
            print(response.message)
            return 0 if response.success else 1

        if args.command == "recover":
            response = client.recover_execution()
            print(response.message)
            return 0 if response.success else 1

        if args.command == "status":
            print_status(client.get_executor_status())
            return 0

        if args.command == "scan-workpieces":
            print("Commanding robot to 'orientation_pose' and initiating workpiece detection slot scanning...")
            response = client.scan_workpieces()
            print(response.message)
            return 0 if response.success else 1

        if args.command == "clear-slot":
            print(f"Clearing slot {args.slot_id} state...")
            response = client.clear_slot(args.slot_id)
            print(response.message)
            return 0 if response.success else 1

        if args.command == "clear-all-slots":
            print("Clearing all workpiece slots states...")
            response = client.clear_all_slots()
            print(response.message)
            return 0 if response.success else 1

        if args.command == "place-to-conveyor":
            print("Executing place to AGV area to conveyor belt (LIN to standby, LIN to c1)...")
            response = client.place_to_avgarea_to_converyerblet()
            if not response.success:
                print(response.message)
                return 1
            print(f"  {response.message}")
            # Poll executor status until the background job finishes
            import time
            while True:
                time.sleep(0.5)
                try:
                    status = client.get_executor_status()
                except Exception:
                    break
                state = status.executor_state
                point = status.active_point_name
                step = status.active_step_index
                total = status.total_steps
                if point:
                    print(f"  [{step}/{total}] {state}: {point}", end="\r")
                if state in ("SUCCEEDED", "FAILED", "STOPPED", "IDLE"):
                    print()
                    if state == "SUCCEEDED":
                        print("✓ Successfully moved LIN to standby then LIN to c1.")
                        return 0
                    else:
                        print(f"✗ Failed: {status.last_error}")
                        return 1
            return 0

        parser.error(f"Unsupported command: {args.command}")
    except RuntimeError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0
