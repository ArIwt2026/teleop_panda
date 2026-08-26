from __future__ import annotations

import threading

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK

from .transforms import pose_stamped_to_transform, residual_between_transforms, serialize_transform


def compute_fk_transform(
    client,
    joint_state,
    *,
    fk_link_name: str,
    frame_id: str,
    timeout_sec: float = 5.0,
):
    request = GetPositionFK.Request()
    request.header.frame_id = frame_id
    request.fk_link_names = [fk_link_name]
    robot_state = RobotState()
    robot_state.joint_state = joint_state
    request.robot_state = robot_state

    future = client.call_async(request)
    completion = threading.Event()

    def _notify(_future) -> None:
        completion.set()

    future.add_done_callback(_notify)
    if future.done():
        completion.set()

    if not completion.wait(timeout_sec):
        raise RuntimeError("Timed out waiting for MoveIt's /compute_fk service.")

    response = future.result()
    if response is None:
        raise RuntimeError("MoveIt's /compute_fk service returned no response.")
    if response.error_code.val != 1 or not response.pose_stamped:
        raise RuntimeError(
            f"MoveIt's /compute_fk returned error code {response.error_code.val} for link '{fk_link_name}'."
        )
    return pose_stamped_to_transform(response.pose_stamped[0])


def serialize_pose_pair(base_to_ee, fk_base_to_ee) -> tuple[dict, dict, dict]:
    residual = residual_between_transforms(base_to_ee, fk_base_to_ee)
    return (
        serialize_transform(base_to_ee),
        serialize_transform(fk_base_to_ee),
        {
            "translation_error_m": residual.translation_error_m,
            "rotation_error_deg": residual.rotation_error_deg,
        },
    )
