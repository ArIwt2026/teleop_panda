from __future__ import annotations

import threading

from franka_teach_msgs.action import PlayPoint, PlaySequence
from franka_teach_msgs.srv import GetExecutorStatus
from rclpy.action import ActionClient
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


class ExecutorBridge:
    def __init__(self, node, executor_namespace: str, callback_group=None) -> None:
        self._node = node
        self._executor_namespace = _normalize_namespace(executor_namespace)
        self._play_sequence_client = ActionClient(
            node,
            PlaySequence,
            _join_name(self._executor_namespace, "play_sequence"),
            callback_group=callback_group,
        )
        self._play_point_client = ActionClient(
            node,
            PlayPoint,
            _join_name(self._executor_namespace, "play_point"),
            callback_group=callback_group,
        )
        self._continue_client = node.create_client(
            Trigger,
            _join_name(self._executor_namespace, "continue"),
            callback_group=callback_group,
        )
        self._stop_client = node.create_client(
            Trigger,
            _join_name(self._executor_namespace, "stop"),
            callback_group=callback_group,
        )
        self._status_client = node.create_client(
            GetExecutorStatus,
            _join_name(self._executor_namespace, "status"),
            callback_group=callback_group,
        )

    def _wait_for_future(self, future, timeout_sec: float, timeout_message: str):
        completion = threading.Event()

        def _notify(_future) -> None:
            completion.set()

        future.add_done_callback(_notify)
        if future.done():
            completion.set()
        if not completion.wait(timeout_sec):
            raise RuntimeError(timeout_message)
        return future.result()

    def wait_for_ready(self, timeout_sec: float = 10.0) -> None:
        if not self._play_point_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError("The taught-point executor action server is not available.")
        if not self._play_sequence_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError("The taught-sequence executor action server is not available.")
        if not self._continue_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("The executor continue service is not available.")
        if not self._stop_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("The executor stop service is not available.")
        if not self._status_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("The executor status service is not available.")

    def start_sequence(
        self,
        sequence_name: str,
        *,
        start_step_index: int = 0,
        force_wait_each_step: bool = True,
        timeout_sec: float = 10.0,
    ):
        goal = PlaySequence.Goal()
        goal.sequence_name = sequence_name
        goal.plan_only = False
        goal.start_step_index = start_step_index
        goal.force_wait_each_step = force_wait_each_step

        goal_future = self._play_sequence_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(
            goal_future,
            timeout_sec,
            "Timed out waiting for the executor to accept the sequence goal.",
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("The executor rejected the sequence goal.")
        return goal_handle, goal_handle.get_result_async()

    def run_point(
        self,
        point_name: str,
        *,
        plan_only: bool,
        velocity_scale: float = -1.0,
        acceleration_scale: float = -1.0,
        planning_time: float = -1.0,
        goal_tolerance: float = -1.0,
        accept_timeout_sec: float = 10.0,
        result_timeout_sec: float = 60.0,
    ):
        goal = PlayPoint.Goal()
        goal.point_name = point_name
        goal.plan_only = plan_only
        goal.velocity_scale = velocity_scale
        goal.acceleration_scale = acceleration_scale
        goal.planning_time = planning_time
        goal.goal_tolerance = goal_tolerance

        goal_future = self._play_point_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(
            goal_future,
            accept_timeout_sec,
            "Timed out waiting for the executor to accept the point goal.",
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("The executor rejected the point goal.")

        return self._wait_for_future(
            goal_handle.get_result_async(),
            result_timeout_sec,
            "Timed out waiting for the executor point result.",
        )

    def get_status(self, timeout_sec: float = 5.0):
        future = self._status_client.call_async(GetExecutorStatus.Request())
        response = self._wait_for_future(
            future,
            timeout_sec,
            "Timed out waiting for executor status.",
        )
        if response is None:
            raise RuntimeError("Executor status service returned no response.")
        return response.status

    def continue_execution(self, timeout_sec: float = 5.0):
        future = self._continue_client.call_async(Trigger.Request())
        response = self._wait_for_future(
            future,
            timeout_sec,
            "Timed out waiting for executor continue response.",
        )
        if response is None:
            raise RuntimeError("Executor continue service returned no response.")
        return response

    def stop_execution(self, timeout_sec: float = 5.0):
        future = self._stop_client.call_async(Trigger.Request())
        response = self._wait_for_future(
            future,
            timeout_sec,
            "Timed out waiting for executor stop response.",
        )
        if response is None:
            raise RuntimeError("Executor stop service returned no response.")
        return response
