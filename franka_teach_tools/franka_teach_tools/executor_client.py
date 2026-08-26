from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import rclpy
from franka_teach_msgs.action import ExecuteRelativePoint, GoToCameraPose, PlayPoint, PlaySequence, PickPlace
from franka_teach_msgs.srv import GetExecutorStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


def _normalize_namespace(namespace: str) -> str:
    stripped = namespace.strip()
    if not stripped:
        return "/franka_teach_executor"
    stripped = stripped.rstrip("/")
    if not stripped.startswith("/"):
        stripped = "/" + stripped
    return stripped


def _join_name(namespace: str, leaf: str) -> str:
    return f"{_normalize_namespace(namespace)}/{leaf.lstrip('/')}"


@dataclass
class ActionCallResult:
    accepted: bool
    result_code: int | None
    result: Any | None


class TeachExecutorClient(Node):
    def __init__(self, executor_namespace: str = "/franka_teach_executor") -> None:
        super().__init__("franka_teach_executor_client")
        self._executor_namespace = _normalize_namespace(executor_namespace)
        self._play_point_client = ActionClient(
            self, PlayPoint, _join_name(self._executor_namespace, "play_point")
        )
        self._go_to_camera_pose_client = ActionClient(
            self, GoToCameraPose, _join_name(self._executor_namespace, "go_to_camera_pose")
        )
        self._play_sequence_client = ActionClient(
            self, PlaySequence, _join_name(self._executor_namespace, "play_sequence")
        )
        self._execute_relative_point_client = ActionClient(
            self, ExecuteRelativePoint, _join_name(self._executor_namespace, "execute_relative_point")
        )
        self._pick_place_client = ActionClient(
            self, PickPlace, _join_name(self._executor_namespace, "pick_place")
        )
        self._pause_client = self.create_client(
            Trigger, _join_name(self._executor_namespace, "pause")
        )
        self._continue_client = self.create_client(
            Trigger, _join_name(self._executor_namespace, "continue")
        )
        self._stop_client = self.create_client(Trigger, _join_name(self._executor_namespace, "stop"))
        self._recover_client = self.create_client(
            Trigger, _join_name(self._executor_namespace, "recover")
        )
        self._status_client = self.create_client(
            GetExecutorStatus, _join_name(self._executor_namespace, "status")
        )
        self._workpiece_trigger_client = self.create_client(
            Trigger, "/workpiece_detector_node/trigger_check"
        )
        self._workpiece_clear_all_client = self.create_client(
            Trigger, "/workpiece_detector_node/clear_all_slots"
        )
        self._place_to_avgarea_client = self.create_client(
            Trigger, _join_name(self._executor_namespace, "place_to_avgarea_to_converyerblet")
        )

    def play_point(
        self,
        point_name: str,
        *,
        plan_only: bool = False,
        velocity_scale: float = -1.0,
        acceleration_scale: float = -1.0,
        planning_time: float = -1.0,
        goal_tolerance: float = -1.0,
        planner_id: str = "PTP",
        base_only: bool = False,
        ref_point_name: str = "",
        feedback_callback=None,
        wait_timeout_sec: float = 5.0,
    ) -> ActionCallResult:
        goal = PlayPoint.Goal()
        goal.point_name = point_name
        goal.plan_only = plan_only
        goal.velocity_scale = velocity_scale
        goal.acceleration_scale = acceleration_scale
        goal.planning_time = planning_time
        goal.goal_tolerance = goal_tolerance
        goal.planner_id = planner_id
        goal.base_only = base_only
        goal.ref_point_name = ref_point_name
        return self._send_action_goal(
            self._play_point_client,
            goal,
            feedback_callback=feedback_callback,
            wait_timeout_sec=wait_timeout_sec,
        )

    def play_sequence(
        self,
        sequence_name: str,
        *,
        plan_only: bool = False,
        start_step_index: int = 0,
        force_wait_each_step: bool = False,
        feedback_callback=None,
        wait_timeout_sec: float = 5.0,
    ) -> ActionCallResult:
        goal = PlaySequence.Goal()
        goal.sequence_name = sequence_name
        goal.plan_only = plan_only
        goal.start_step_index = start_step_index
        goal.force_wait_each_step = force_wait_each_step
        return self._send_action_goal(
            self._play_sequence_client,
            goal,
            feedback_callback=feedback_callback,
            wait_timeout_sec=wait_timeout_sec,
        )

    def go_to_camera_pose(
        self,
        target_pose: PoseStamped,
        *,
        plan_only: bool = False,
        velocity_scale: float = -1.0,
        acceleration_scale: float = -1.0,
        planning_time: float = -1.0,
        goal_tolerance: float = -1.0,
        feedback_callback=None,
        wait_timeout_sec: float = 5.0,
    ) -> ActionCallResult:
        goal = GoToCameraPose.Goal()
        goal.target_pose = target_pose
        goal.plan_only = plan_only
        goal.velocity_scale = velocity_scale
        goal.acceleration_scale = acceleration_scale
        goal.planning_time = planning_time
        goal.goal_tolerance = goal_tolerance
        return self._send_action_goal(
            self._go_to_camera_pose_client,
            goal,
            feedback_callback=feedback_callback,
            wait_timeout_sec=wait_timeout_sec,
        )

    def execute_relative_point(
        self,
        point_name: str,
        *,
        marker_pose: PoseStamped | None = None,
        mode: str = "move",
        approach_height: float = -1.0,
        gripper_width: float = -1.0,
        gripper_force: float = -1.0,
        gripper_speed: float = -1.0,
        plan_only: bool = False,
        velocity_scale: float = -1.0,
        acceleration_scale: float = -1.0,
        planning_time: float = -1.0,
        goal_tolerance: float = -1.0,
        skip_observation_pose: bool = False,
        planner_id: str = "PTP",
        planning_pipeline_id: str = "",
        feedback_callback=None,
        wait_timeout_sec: float = 5.0,
    ) -> ActionCallResult:
        """Call the execute_relative_point action on the executor."""
        from franka_teach_msgs.action import ExecuteRelativePoint
        goal = ExecuteRelativePoint.Goal()
        goal.point_name = point_name
        if marker_pose:
            goal.marker_pose = marker_pose
        goal.mode = mode
        goal.approach_height = approach_height
        goal.gripper_width = gripper_width
        goal.gripper_force = gripper_force
        goal.gripper_speed = gripper_speed
        goal.plan_only = plan_only
        goal.velocity_scale = velocity_scale
        goal.acceleration_scale = acceleration_scale
        goal.planning_time = planning_time
        goal.goal_tolerance = goal_tolerance
        goal.skip_observation_pose = skip_observation_pose
        goal.planner_id = planner_id
        goal.planning_pipeline_id = planning_pipeline_id
        
        return self._send_action_goal(
            self._execute_relative_point_client,
            goal,
            feedback_callback=feedback_callback,
            wait_timeout_sec=wait_timeout_sec,
        )

    def pick_place(
        self,
        point_name: str,
        mode: str,
        *,
        approach_height: float = -1.0,
        lift_off_height: float = -1.0,
        gripper_width: float = -1.0,
        gripper_force: float = -1.0,
        gripper_speed: float = -1.0,
        place_descent_force: float = -1.0,
        plan_only: bool = False,
        velocity_scale: float = -1.0,
        acceleration_scale: float = -1.0,
        planning_time: float = -1.0,
        goal_tolerance: float = -1.0,
        planner_id: str = "PTP",
        feedback_callback=None,
        wait_timeout_sec: float = 5.0,
    ) -> ActionCallResult:
        goal = PickPlace.Goal()
        goal.point_name = point_name
        goal.mode = mode
        goal.approach_height = approach_height
        goal.lift_off_height = lift_off_height
        goal.gripper_width = gripper_width
        goal.gripper_force = gripper_force
        goal.gripper_speed = gripper_speed
        goal.place_descent_force = place_descent_force
        goal.plan_only = plan_only
        goal.velocity_scale = velocity_scale
        goal.acceleration_scale = acceleration_scale
        goal.planning_time = planning_time
        goal.goal_tolerance = goal_tolerance
        goal.planner_id = planner_id
        return self._send_action_goal(
            self._pick_place_client,
            goal,
            feedback_callback=feedback_callback,
            wait_timeout_sec=wait_timeout_sec,
        )

    def pause_execution(self, wait_timeout_sec: float = 5.0):
        return self._call_trigger(self._pause_client, wait_timeout_sec=wait_timeout_sec)

    def continue_execution(self, wait_timeout_sec: float = 5.0):
        return self._call_trigger(self._continue_client, wait_timeout_sec=wait_timeout_sec)

    def stop_execution(self, wait_timeout_sec: float = 5.0):
        return self._call_trigger(self._stop_client, wait_timeout_sec=wait_timeout_sec)

    def recover_execution(self, wait_timeout_sec: float = 5.0):
        return self._call_trigger(self._recover_client, wait_timeout_sec=wait_timeout_sec)

    def scan_workpieces(self, wait_timeout_sec: float = 15.0):
        return self._call_trigger(self._workpiece_trigger_client, wait_timeout_sec=wait_timeout_sec)

    def clear_all_slots(self, wait_timeout_sec: float = 5.0):
        return self._call_trigger(self._workpiece_clear_all_client, wait_timeout_sec=wait_timeout_sec)

    def place_to_avgarea_to_converyerblet(self, wait_timeout_sec: float = 30.0):
        return self._call_trigger(self._place_to_avgarea_client, wait_timeout_sec=wait_timeout_sec)

    def clear_slot(self, slot_id: int, wait_timeout_sec: float = 5.0):
        client = self.create_client(Trigger, f"/workpiece_detector_node/clear_slot_{slot_id}")
        return self._call_trigger(client, wait_timeout_sec=wait_timeout_sec)

    def get_executor_status(self, wait_timeout_sec: float = 5.0):
        if not self._status_client.wait_for_service(timeout_sec=wait_timeout_sec):
            raise RuntimeError("Executor status service is not available.")
        future = self._status_client.call_async(GetExecutorStatus.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait_timeout_sec)
        if not future.done():
            raise RuntimeError("Timed out waiting for executor status.")
        response = future.result()
        if response is None:
            raise RuntimeError("Executor status service returned no response.")
        return response.status

    def _send_action_goal(
        self,
        client: ActionClient,
        goal_msg,
        *,
        feedback_callback=None,
        wait_timeout_sec: float = 5.0,
    ) -> ActionCallResult:
        if not client.wait_for_server(timeout_sec=wait_timeout_sec):
            raise RuntimeError("Executor action server is not available.")

        future = client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait_timeout_sec)
        if not future.done():
            raise RuntimeError("Timed out waiting for action goal response.")

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return ActionCallResult(accepted=False, result_code=None, result=None)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if not result_future.done():
            raise RuntimeError("Timed out waiting for action result.")

        wrapped_result = result_future.result()
        return ActionCallResult(
            accepted=True,
            result_code=wrapped_result.status,
            result=wrapped_result.result,
        )

    def _call_trigger(self, client, *, wait_timeout_sec: float = 5.0):
        if not client.wait_for_service(timeout_sec=wait_timeout_sec):
            raise RuntimeError("Executor trigger service is not available.")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait_timeout_sec)
        if not future.done():
            raise RuntimeError("Timed out waiting for trigger service response.")
        response = future.result()
        if response is None:
            raise RuntimeError("Trigger service returned no response.")
        return response


def executor_status_to_dict(status) -> dict[str, Any]:
    return {
        "executor_state": status.executor_state,
        "active_sequence_name": status.active_sequence_name,
        "active_point_name": status.active_point_name,
        "active_step_index": status.active_step_index,
        "total_steps": status.total_steps,
        "completed_steps": status.completed_steps,
        "failed_step_index": status.failed_step_index,
        "failed_point_name": status.failed_point_name,
        "last_error": status.last_error,
        "busy": status.busy,
        "pause_requested": status.pause_requested,
        "waiting_for_continue": status.waiting_for_continue,
        "fault_reflex": status.fault_reflex,
        "fault_stopped": status.fault_stopped,
        "recover_required": status.recover_required,
        "stop_requested": status.stop_requested,
    }
