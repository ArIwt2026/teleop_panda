from __future__ import annotations

import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def sanitize_name(name: str) -> str:
    stripped = name.strip()
    if not stripped:
        raise ValueError("Name must not be empty.")
    sanitized = re.sub(r"[^A-Za-z0-9_-]+", "_", stripped)
    sanitized = sanitized.strip("_")
    if not sanitized:
        raise ValueError("Name must contain at least one letter or number.")
    return sanitized


def detect_default_storage_root() -> Path:
    env_value = Path.home() / ".ros" / "franka_teach_tools"
    try:
        share_dir = Path(get_package_share_directory("franka_teach_tools")).resolve()
    except PackageNotFoundError:
        return env_value

    for parent in share_dir.parents:
        if parent.name == "install" and parent.parent.exists():
            return parent.parent / "teach_data"

    return env_value


def resolve_storage_root(storage_dir: str | None) -> Path:
    if storage_dir:
        return Path(storage_dir).expanduser().resolve()
    return detect_default_storage_root()


def ensure_storage_tree(storage_root: Path) -> None:
    (storage_root / "points").mkdir(parents=True, exist_ok=True)
    (storage_root / "sequences").mkdir(parents=True, exist_ok=True)


def point_file_path(storage_root: Path, point_name: str) -> Path:
    return storage_root / "points" / f"{sanitize_name(point_name)}.yaml"


def sequence_file_path(storage_root: Path, sequence_name: str) -> Path:
    return storage_root / "sequences" / f"{sanitize_name(sequence_name)}.yaml"


def dump_yaml(path: Path, payload: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as output:
        yaml.safe_dump(payload, output, sort_keys=False)


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"Expected a mapping in {path}.")
    return loaded


def save_point(storage_root: Path, point_name: str, payload: dict[str, Any], overwrite: bool) -> Path:
    ensure_storage_tree(storage_root)
    point_name = sanitize_name(point_name)
    point_path = point_file_path(storage_root, point_name)
    if point_path.exists() and not overwrite:
        raise FileExistsError(f"Point '{point_name}' already exists at {point_path}.")

    record = {
        "name": point_name,
        "type": "point",
        "captured_at": utc_now_iso(),
        **payload,
    }
    dump_yaml(point_path, record)
    return point_path


def initialize_sequence(storage_root: Path, sequence_name: str, overwrite: bool) -> Path:
    ensure_storage_tree(storage_root)
    sequence_name = sanitize_name(sequence_name)
    sequence_path = sequence_file_path(storage_root, sequence_name)
    if sequence_path.exists() and not overwrite:
        raise FileExistsError(
            f"Sequence '{sequence_name}' already exists at {sequence_path}. Use a new name or overwrite."
        )

    dump_yaml(
        sequence_path,
        {
            "name": sequence_name,
            "type": "sequence",
            "created_at": utc_now_iso(),
            "updated_at": utc_now_iso(),
            "steps": [],
        },
    )
    return sequence_path


def append_step_to_sequence(
    storage_root: Path,
    sequence_name: str,
    point_name: str,
    joint_states_topic: str,
) -> Path:
    ensure_storage_tree(storage_root)
    sequence_name = sanitize_name(sequence_name)
    point_name = sanitize_name(point_name)
    sequence_path = sequence_file_path(storage_root, sequence_name)

    if not sequence_path.exists():
        initialize_sequence(storage_root, sequence_name, overwrite=False)

    record = load_yaml(sequence_path)
    steps = record.setdefault("steps", [])
    if not isinstance(steps, list):
        raise ValueError(f"Sequence file {sequence_path} is invalid.")

    steps.append(
        {
            "index": len(steps) + 1,
            "point_name": point_name,
            "point_file": f"points/{point_name}.yaml",
            "joint_states_topic": joint_states_topic,
            "added_at": utc_now_iso(),
        }
    )
    record["updated_at"] = utc_now_iso()
    dump_yaml(sequence_path, record)
    return sequence_path


def append_gripper_step_to_sequence(
    storage_root: Path,
    sequence_name: str,
    gripper_action: str,
    gripper_width: float,
    gripper_speed: float = 0.1,
    gripper_force: float = 50.0,
    gripper_epsilon_inner: float = 0.005,
    gripper_epsilon_outer: float = 0.005,
) -> Path:
    """Append a gripper action step (grasp or open) to a sequence."""
    ensure_storage_tree(storage_root)
    sequence_name = sanitize_name(sequence_name)
    sequence_path = sequence_file_path(storage_root, sequence_name)

    if not sequence_path.exists():
        initialize_sequence(storage_root, sequence_name, overwrite=False)

    record = load_yaml(sequence_path)
    steps = record.setdefault("steps", [])
    if not isinstance(steps, list):
        raise ValueError(f"Sequence file {sequence_path} is invalid.")

    steps.append(
        {
            "index": len(steps) + 1,
            "type": "gripper",
            "gripper_action": gripper_action,
            "gripper_width": gripper_width,
            "gripper_speed": gripper_speed,
            "gripper_force": gripper_force,
            "gripper_epsilon_inner": gripper_epsilon_inner,
            "gripper_epsilon_outer": gripper_epsilon_outer,
            "added_at": utc_now_iso(),
        }
    )
    record["updated_at"] = utc_now_iso()
    dump_yaml(sequence_path, record)
    return sequence_path


def list_named_yaml_files(directory: Path) -> list[str]:
    if not directory.exists():
        return []
    return sorted(path.stem for path in directory.glob("*.yaml"))
