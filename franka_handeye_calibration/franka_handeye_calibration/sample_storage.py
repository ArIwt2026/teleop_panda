from __future__ import annotations

import json
import os
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import yaml

from .job_config import CalibrationJobConfig


@dataclass(frozen=True)
class RunPaths:
    run_dir: Path
    images_dir: Path
    detections_dir: Path
    debug_dir: Path
    samples_jsonl: Path
    result_yaml: Path
    metrics_json: Path
    job_copy_yaml: Path


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def make_run_paths(job: CalibrationJobConfig) -> RunPaths:
    run_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    run_dir = job.output_root / job.job_name / run_id
    images_dir = run_dir / "images"
    detections_dir = run_dir / "detections"
    debug_dir = run_dir / "debug"
    for directory in (images_dir, detections_dir, debug_dir):
        directory.mkdir(parents=True, exist_ok=True)
    return RunPaths(
        run_dir=run_dir,
        images_dir=images_dir,
        detections_dir=detections_dir,
        debug_dir=debug_dir,
        samples_jsonl=run_dir / "samples.jsonl",
        result_yaml=run_dir / "result_handeye.yaml",
        metrics_json=run_dir / "result_metrics.json",
        job_copy_yaml=run_dir / "job_config.yaml",
    )


def save_job_config(run_paths: RunPaths, config_payload: dict[str, Any]) -> None:
    save_yaml(run_paths.job_copy_yaml, config_payload)


def save_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    _atomic_write_text(path, json.dumps(payload, indent=2))


def save_yaml(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    _atomic_write_text(path, yaml.safe_dump(payload, sort_keys=False))


def append_jsonl(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload) + "\n")


def save_image(path: Path, image_bgr) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), image_bgr)


def sample_file_stem(step_index: int, point_name: str, retry_count: int) -> str:
    safe_name = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in point_name)
    return f"step_{step_index:03d}_{safe_name}_try_{retry_count:02d}"


def _atomic_write_text(path: Path, text: str) -> None:
    parent = path.parent
    parent.mkdir(parents=True, exist_ok=True)
    # Use a unique suffix to prevent collisions between threads/processes
    unique_id = uuid.uuid4().hex[:8]
    tmp_path = parent / f".{path.name}.{unique_id}.tmp"
    try:
        with tmp_path.open("w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        tmp_path.replace(path)
    finally:
        # Cleanup in case replace failed before rename
        if tmp_path.exists():
            try:
                tmp_path.unlink()
            except OSError:
                pass
