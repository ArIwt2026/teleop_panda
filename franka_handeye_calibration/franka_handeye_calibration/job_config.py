from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

DEFAULT_JOB_NAME = "d455_handeye"


@dataclass(frozen=True)
class BoardConfig:
    squares_x: int
    squares_y: int
    square_length_m: float
    marker_length_m: float
    dictionary_name: str


@dataclass(frozen=True)
class FreshnessConfig:
    image_max_age_sec: float = 1.0
    camera_info_max_age_sec: float = 1.0
    robot_state_max_age_sec: float = 1.0
    joint_state_max_age_sec: float = 1.0


@dataclass(frozen=True)
class CalibrationJobConfig:
    job_name: str
    sequence_name: str
    eye_in_hand: bool
    capture_mode: str
    image_topic: str
    camera_info_topic: str
    robot_state_topic: str
    joint_state_topic: str
    intrinsics_path: Path | None
    fk_service_name: str
    fk_link_name: str
    robot_pose_source: str
    robot_base_frame: str
    board: BoardConfig
    settle_time_sec: float
    min_corners: int
    required_min_samples: int
    max_retries_per_point: int
    skip_failed_points: bool
    retry_backoff_sec: float
    output_root: Path
    executor_namespace: str
    freshness: FreshnessConfig
    reject_on_fk_mismatch: bool
    max_fk_translation_error_m: float
    max_fk_rotation_error_deg: float
    skip_steps: list[int]


@dataclass(frozen=True)
class CameraIntrinsics:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    width: int | None
    height: int | None
    source_path: Path


def _workspace_root_from_share(package_name: str) -> Path | None:
    try:
        share_dir = Path(get_package_share_directory(package_name)).resolve()
    except PackageNotFoundError:
        return None

    for parent in share_dir.parents:
        if parent.name == "install" and parent.parent.exists():
            return parent.parent
    return None


def detect_workspace_root() -> Path:
    workspace_root = _workspace_root_from_share("franka_handeye_calibration")
    if workspace_root is not None:
        return workspace_root
    return Path.home() / ".ros" / "franka_handeye_calibration"


def detect_default_output_root() -> Path:
    return detect_workspace_root() / "calibration_data" / "handeye"


def resolve_job_config_path(job_name: str = "", job_config_path: str = "") -> Path:
    if job_config_path:
        return Path(job_config_path).expanduser().resolve()

    resolved_job_name = job_name or DEFAULT_JOB_NAME

    share_dir = Path(get_package_share_directory("franka_handeye_calibration"))
    return (share_dir / "config" / "jobs" / f"{resolved_job_name}.yaml").resolve()


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    if not isinstance(payload, dict):
        raise ValueError(f"Expected a mapping in {path}.")
    return payload


def load_job_config(job_name: str = "", job_config_path: str = "") -> tuple[CalibrationJobConfig, Path]:
    config_path = resolve_job_config_path(job_name, job_config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Calibration job config does not exist: {config_path}")

    payload = _load_yaml(config_path)
    board_payload = payload.get("board") or {}
    freshness_payload = payload.get("freshness") or {}
    workspace_root = detect_workspace_root()

    output_root = Path(payload.get("output_root", detect_default_output_root()))
    if not output_root.is_absolute():
        output_root = workspace_root / output_root

    intrinsics_path_value = str(payload.get("intrinsics_path", "")).strip()
    intrinsics_path = None
    if intrinsics_path_value:
        intrinsics_path = Path(intrinsics_path_value).expanduser()
        if not intrinsics_path.is_absolute():
            intrinsics_path = (workspace_root / intrinsics_path).resolve()

    config = CalibrationJobConfig(
        job_name=str(payload.get("job_name") or config_path.stem),
        sequence_name=str(payload["sequence_name"]),
        eye_in_hand=bool(payload.get("eye_in_hand", True)),
        capture_mode=str(payload.get("capture_mode", "charuco")).lower(),
        image_topic=str(payload["image_topic"]),
        camera_info_topic=str(payload["camera_info_topic"]),
        robot_state_topic=str(payload.get("robot_state_topic", "/fr3/robot_state")),
        joint_state_topic=str(payload.get("joint_state_topic", "/franka/joint_states")),
        intrinsics_path=intrinsics_path,
        fk_service_name=str(payload.get("fk_service_name", "/compute_fk")),
        fk_link_name=str(payload.get("fk_link_name", "fr3_hand_tcp")),
        robot_pose_source=str(payload.get("robot_pose_source", "robot_state")),
        robot_base_frame=str(payload.get("robot_base_frame", "fr3_link0")),
        board=BoardConfig(
            squares_x=int(board_payload.get("squares_x", 6)),
            squares_y=int(board_payload.get("squares_y", 8)),
            square_length_m=float(board_payload.get("square_length_m", 0.03)),
            marker_length_m=float(board_payload.get("marker_length_m", 0.021)),
            dictionary_name=str(board_payload.get("dictionary_name", "DICT_5X5_100")),
        ),
        settle_time_sec=float(payload.get("settle_time_sec", 0.5)),
        min_corners=int(payload.get("min_corners", 12)),
        required_min_samples=int(payload.get("required_min_samples", 8)),
        max_retries_per_point=int(payload.get("max_retries_per_point", 3)),
        skip_failed_points=bool(payload.get("skip_failed_points", False)),
        retry_backoff_sec=float(payload.get("retry_backoff_sec", 0.5)),
        output_root=output_root.resolve(),
        executor_namespace=str(payload.get("executor_namespace", "/franka_teach_executor")),
        freshness=FreshnessConfig(
            image_max_age_sec=float(freshness_payload.get("image_max_age_sec", 1.0)),
            camera_info_max_age_sec=float(freshness_payload.get("camera_info_max_age_sec", 1.0)),
            robot_state_max_age_sec=float(freshness_payload.get("robot_state_max_age_sec", 1.0)),
            joint_state_max_age_sec=float(freshness_payload.get("joint_state_max_age_sec", 1.0)),
        ),
        reject_on_fk_mismatch=bool(payload.get("reject_on_fk_mismatch", False)),
        max_fk_translation_error_m=float(payload.get("max_fk_translation_error_m", 0.02)),
        max_fk_rotation_error_deg=float(payload.get("max_fk_rotation_error_deg", 5.0)),
        skip_steps=list(payload.get("skip_steps", [])),
    )
    return config, config_path


def load_intrinsics(path: Path | None) -> CameraIntrinsics:
    if path is None:
        raise ValueError("No intrinsics path was configured for this calibration job.")
    if not path.exists():
        raise FileNotFoundError(f"Intrinsics file does not exist: {path}")

    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)

    width = None
    height = None
    dist_coeffs: list[float]
    fx: float
    fy: float
    cx: float
    cy: float

    if "camera_matrix" in payload:
        camera_matrix = np.asarray(payload["camera_matrix"], dtype=np.float64).reshape(3, 3)
        dist_coeffs = list(payload.get("dist_coeffs", []))
        width = payload.get("image_width")
        height = payload.get("image_height")
        return CameraIntrinsics(
            camera_matrix=camera_matrix,
            dist_coeffs=np.asarray(dist_coeffs, dtype=np.float64).reshape(-1),
            width=width,
            height=height,
            source_path=path,
        )

    if "rgb_intrinsics_corrected_charuco" in payload:
        rgb = payload["rgb_intrinsics_corrected_charuco"]
        fx = float(rgb["fx"])
        fy = float(rgb["fy"])
        cx = float(rgb["cx"])
        cy = float(rgb["cy"])
        dist_coeffs = list(rgb.get("dist_coeffs", []))
        width = rgb.get("width")
        height = rgb.get("height")
    elif "streams" in payload and "color" in payload["streams"]:
        color = payload["streams"]["color"]
        fx = float(color["fx"])
        fy = float(color["fy"])
        cx = float(color["cx"])
        cy = float(color["cy"])
        dist_coeffs = list(color.get("distortion_coeffs", []))
        width = color.get("width")
        height = color.get("height")
    else:
        raise ValueError(f"Unsupported intrinsics file format: {path}")

    camera_matrix = np.array(
        [
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    return CameraIntrinsics(
        camera_matrix=camera_matrix,
        dist_coeffs=np.asarray(dist_coeffs, dtype=np.float64).reshape(-1),
        width=width,
        height=height,
        source_path=path,
    )


def intrinsics_from_camera_info(camera_info: Any) -> CameraIntrinsics:
    camera_matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
    if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
        raise ValueError("CameraInfo contains an invalid camera matrix.")

    width = int(camera_info.width) if getattr(camera_info, "width", 0) else None
    height = int(camera_info.height) if getattr(camera_info, "height", 0) else None

    return CameraIntrinsics(
        camera_matrix=camera_matrix,
        dist_coeffs=np.asarray(list(camera_info.d), dtype=np.float64).reshape(-1),
        width=width,
        height=height,
        source_path=Path("<camera_info>"),
    )
