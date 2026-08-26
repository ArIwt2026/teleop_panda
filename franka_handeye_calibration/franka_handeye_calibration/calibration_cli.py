from __future__ import annotations

import argparse
import json
import sys

import rclpy
from action_msgs.msg import GoalStatus

from .api import (
    HandEyeCalibrationClient,
    collect_calibration_samples,
    solve_calibration_run,
    validate_calibration_run,
)
from .job_config import DEFAULT_JOB_NAME


def _feedback_callback(feedback_msg) -> None:
    feedback = feedback_msg.feedback
    print(
        "[feedback] "
        f"state={feedback.state} "
        f"step={feedback.active_step_index}/{feedback.total_steps} "
        f"point={feedback.active_point_name} "
        f"valid={feedback.valid_samples} "
        f"attempted={feedback.attempted_samples} "
        f"retries={feedback.retry_count} "
        f"message={feedback.last_message}"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CLI for the Franka hand-eye calibration stack.")
    parser.add_argument(
        "--namespace",
        default="/franka_handeye_calibration",
        help="Calibration coordinator namespace or absolute name prefix.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    run_job = subparsers.add_parser("run-job", help="Collect samples and solve a calibration job.")
    run_job.add_argument("job_name", nargs="?", default=DEFAULT_JOB_NAME)
    run_job.add_argument("--job-config", default="")

    collect_job = subparsers.add_parser("collect-job", help="Collect samples only for a calibration job.")
    collect_job.add_argument("job_name", nargs="?", default=DEFAULT_JOB_NAME)
    collect_job.add_argument("--job-config", default="")

    solve_run = subparsers.add_parser("solve-run", help="Solve a saved calibration run directory.")
    solve_run.add_argument("run_dir")

    validate_run = subparsers.add_parser("validate-run", help="Validate a saved calibration run directory.")
    validate_run.add_argument("run_dir")

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        if args.command == "solve-run":
            print(json.dumps(solve_calibration_run(args.run_dir), indent=2))
            return 0

        if args.command == "validate-run":
            print(json.dumps(validate_calibration_run(args.run_dir), indent=2))
            return 0

        if args.command == "collect-job":
            wrapped_result = collect_calibration_samples(
                args.job_name,
                job_config_path=args.job_config,
                namespace=args.namespace,
            )
        else:
            rclpy.init(args=None)
            client = HandEyeCalibrationClient(namespace=args.namespace)
            try:
                wrapped_result = client.run_job(
                    args.job_name,
                    job_config_path=args.job_config,
                    solve_after_collection=True,
                    feedback_callback=_feedback_callback,
                )
            finally:
                client.destroy_node()
                rclpy.shutdown()

        result = wrapped_result.result
        print(f"goal_status: {wrapped_result.status}")
        print(f"success: {result.success}")
        print(f"final_state: {result.final_state}")
        print(f"job_name: {result.job_name}")
        print(f"run_directory: {result.run_directory}")
        print(f"attempted_samples: {result.attempted_samples}")
        print(f"valid_samples: {result.valid_samples}")
        print(f"chosen_method: {result.chosen_method}")
        print(f"transform_file: {result.transform_file}")
        print(f"error_message: {result.error_message}")
        return 0 if result.success and wrapped_result.status == GoalStatus.STATUS_SUCCEEDED else 1
    except RuntimeError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
