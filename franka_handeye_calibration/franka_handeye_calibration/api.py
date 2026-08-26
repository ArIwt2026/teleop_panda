from __future__ import annotations

from typing import Any

import rclpy
from franka_handeye_msgs.action import RunCalibration
from rclpy.action import ActionClient
from rclpy.node import Node

from .handeye_solver import solve_calibration_run as solve_run_impl
from .handeye_solver import validate_calibration_run as validate_run_impl


def _normalize_namespace(namespace: str) -> str:
    stripped = namespace.strip()
    if not stripped:
        return "/franka_handeye_calibration"
    stripped = stripped.rstrip("/")
    if not stripped.startswith("/"):
        stripped = "/" + stripped
    return stripped


def _join_name(namespace: str, leaf: str) -> str:
    return f"{_normalize_namespace(namespace)}/{leaf.lstrip('/')}"


class HandEyeCalibrationClient(Node):
    def __init__(self, namespace: str = "/franka_handeye_calibration") -> None:
        super().__init__("franka_handeye_calibration_client")
        self._namespace = _normalize_namespace(namespace)
        self._run_client = ActionClient(
            self,
            RunCalibration,
            _join_name(self._namespace, "run_calibration"),
        )

    def run_job(
        self,
        job_name: str,
        *,
        job_config_path: str = "",
        solve_after_collection: bool = True,
        wait_timeout_sec: float = 10.0,
        feedback_callback=None,
    ):
        if not self._run_client.wait_for_server(timeout_sec=wait_timeout_sec):
            raise RuntimeError("The hand-eye calibration action server is not available.")

        goal = RunCalibration.Goal()
        goal.job_name = job_name
        goal.job_config_path = job_config_path
        goal.solve_after_collection = solve_after_collection

        goal_future = self._run_client.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=wait_timeout_sec)
        if not goal_future.done():
            raise RuntimeError("Timed out waiting for the calibration goal response.")

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("The calibration goal was rejected.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if not result_future.done():
            raise RuntimeError("Timed out waiting for the calibration result.")
        return result_future.result()


def run_calibration_job(
    job_name: str,
    *,
    job_config_path: str = "",
    solve_after_collection: bool = True,
    namespace: str = "/franka_handeye_calibration",
) -> Any:
    rclpy.init(args=None)
    client = HandEyeCalibrationClient(namespace=namespace)
    try:
        return client.run_job(
            job_name,
            job_config_path=job_config_path,
            solve_after_collection=solve_after_collection,
        )
    finally:
        client.destroy_node()
        rclpy.shutdown()


def collect_calibration_samples(
    job_name: str,
    *,
    job_config_path: str = "",
    namespace: str = "/franka_handeye_calibration",
) -> Any:
    return run_calibration_job(
        job_name,
        job_config_path=job_config_path,
        solve_after_collection=False,
        namespace=namespace,
    )


def solve_calibration_run(run_dir: str) -> dict[str, Any]:
    return solve_run_impl(run_dir)


def validate_calibration_run(run_dir: str) -> dict[str, Any]:
    return validate_run_impl(run_dir)
