from __future__ import annotations

import copy
import threading
import textwrap
import time
import traceback
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from franka_handeye_msgs.action import RunCalibration
from franka_msgs.msg import FrankaRobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState

IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

from .charuco_pose_estimator import (
    create_charuco_board,
    detect_aruco_pose,
    detect_charuco_pose,
    get_dictionary,
    serialize_detection,
)
from .executor_bridge import ExecutorBridge
from .handeye_solver import (
    load_calibration_result,
    solve_calibration_run,
    solve_handeye_from_samples,
)
from .job_config import (
    DEFAULT_JOB_NAME,
    CalibrationJobConfig,
    detect_workspace_root,
    intrinsics_from_camera_info,
    load_job_config,
)
from .robot_pose_collector import compute_fk_transform, serialize_pose_pair
from .sample_storage import (
    append_jsonl,
    make_run_paths,
    sample_file_stem,
    save_job_config,
    save_json,
    save_yaml,
    utc_now_iso,
)
from .transforms import (
    franka_matrix_to_transform,
    invert_transform,
    matrix_to_quaternion,
    pose_stamped_to_transform,
    residual_between_transforms,
    serialize_transform,
)


@dataclass
class Snapshot:
    image: Image | None
    image_bgr: np.ndarray | None  # Pre-converted OpenCV image
    camera_info: CameraInfo | None
    intrinsics: any | None  # Pre-processed intrinsics
    joint_state: JointState
    robot_state: FrankaRobotState | None


@dataclass(frozen=True)
class VerificationPlan:
    point_name: str
    point_path: str
    created_mono: float
    fake_hardware: bool
    selected_charuco_id: int | None
    target_translation_xyz_m: tuple[float, float, float]


class HandEyeCalibrationCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("franka_handeye_calibration")

        self._executor_namespace = self.declare_parameter(
            "executor_namespace", "/franka_teach_executor"
        ).value
        self._default_job_name = self.declare_parameter("job_name", DEFAULT_JOB_NAME).value
        self._default_job_config_path = self.declare_parameter("job_config_path", "").value
        self._enable_preview = bool(self.declare_parameter("enable_preview", True).value)
        self._preview_window_name = self.declare_parameter(
            "preview_window_name", "franka_handeye_preview"
        ).value
        self._autostart = bool(self.declare_parameter("autostart", False).value)
        self._autostart_job_name = self.declare_parameter("autostart_job_name", "").value
        self._autostart_job_config_path = self.declare_parameter(
            "autostart_job_config_path", ""
        ).value
        self._autostart_solve_after_collection = bool(
            self.declare_parameter("autostart_solve_after_collection", True).value
        )
        self._autostart_delay_sec = float(
            self.declare_parameter("autostart_delay_sec", 2.0).value
        )
        self._use_fake_hardware = bool(self.declare_parameter("use_fake_hardware", False).value)
        self._planning_group = self.declare_parameter("planning_group", "panda_arm").value
        self._ik_service_name_param = self.declare_parameter("ik_service_name", "/compute_ik").value
        self._verification_velocity_scale = float(
            self.declare_parameter("verification_velocity_scale", 0.1).value
        )
        self._verification_acceleration_scale = float(
            self.declare_parameter("verification_acceleration_scale", 0.1).value
        )
        self._verification_planning_time = float(
            self.declare_parameter("verification_planning_time", 5.0).value
        )
        self._verification_goal_tolerance = float(
            self.declare_parameter("verification_goal_tolerance", 0.001).value
        )
        self._verification_camera_standoff_m = float(
            self.declare_parameter("verification_camera_standoff_m", 0.01).value
        )
        self._verification_max_translation_m = float(
            self.declare_parameter("verification_max_translation_m", 0.60).value
        )
        self._verification_plan_ttl_sec = float(
            self.declare_parameter("verification_plan_ttl_sec", 60.0).value
        )
        self._verification_workspace_min_x = float(
            self.declare_parameter("verification_workspace_min_x", -0.60).value
        )
        self._verification_workspace_max_x = float(
            self.declare_parameter("verification_workspace_max_x", 1.0).value
        )
        self._verification_workspace_min_y = float(
            self.declare_parameter("verification_workspace_min_y", -0.85).value
        )
        self._verification_workspace_max_y = float(
            self.declare_parameter("verification_workspace_max_y", 0.85).value
        )
        self._verification_workspace_min_z = float(
            self.declare_parameter("verification_workspace_min_z", -0.05).value
        )
        self._verification_workspace_max_z = float(
            self.declare_parameter("verification_workspace_max_z", 1.10).value
        )
        self._client_callback_group = ReentrantCallbackGroup()
        self._timer_callback_group = ReentrantCallbackGroup()
        self._action_callback_group = ReentrantCallbackGroup()

        self._cv_bridge = CvBridge()
        self._executor_bridge = ExecutorBridge(
            self,
            self._executor_namespace,
            callback_group=self._client_callback_group,
        )
        self._last_executor_namespace = self._executor_namespace
        self._lifecycle_lock = threading.Lock()
        self._is_shutting_down = False

        self._fk_client = None
        self._fk_service_name = ""
        self._ik_client = None
        self._ik_service_name = ""
        self._preview_fk_link_name = ""
        self._preview_base_frame = ""
        self._preview_robot_pose_source = ""
        self._preview_ee_cache_lines = ["EE xyz [m]: unavailable"]
        self._preview_ee_cache_time = 0.0
        self._preview_calibration_lines = ["Calibration: waiting for samples"]
        self._preview_verification_lines = ["Verify: plan a target before execute"]

        self._manual_translation_threshold_m = 0.02
        self._manual_rotation_threshold_deg = 5.0
        self._last_saved_base_to_ee: np.ndarray | None = None
        self._manual_collection_active = False

        self._current_valid_samples: list[dict[str, Any]] = []
        self._current_samples_job_name = ""
        self._latest_handeye_solution: dict[str, Any] | None = None
        self._latest_handeye_job_name = ""
        self._latest_handeye_source = ""
        self._teach_points_dir = detect_workspace_root() / "teach_data" / "points"
        self._rng = np.random.default_rng()

        self._run_lock = threading.Lock()
        self._run_active = False
        self._last_run_request: tuple[str, str, bool] | None = None
        self._pending_background_run: tuple[str, str, bool] | None = None
        self._current_run_paths = None

        # Live detection cache
        self._live_detect_config: tuple[str, str, any, any] | None = None
        self._live_detect_config_lock = threading.Lock()

        self._verification_lock = threading.Lock()
        self._verification_busy = False
        self._last_verification_plan: VerificationPlan | None = None

        self._latest_lock = threading.Lock()
        self._latest_image: tuple[Image, float] | None = None
        self._latest_camera_info: tuple[CameraInfo, float] | None = None
        self._latest_joint_state: tuple[JointState, float] | None = None
        self._latest_robot_state: tuple[FrankaRobotState, float] | None = None

        self._preview_lock = threading.Lock()
        self._preview_frame = None
        self._preview_status_lines = ["Idle"]
        self._preview_disabled = False
        self._preview_window_initialized = False
        self._preview_button_rects: dict[str, tuple[int, int, int, int]] = {}
        
        # Settle logic for auto-capture
        self._last_move_time = time.monotonic()
        self._stationary_start_time = time.monotonic()
        self._last_preview_ee_pose: np.ndarray | None = None
        self._manual_solve_requested = False

        self._image_subscription = None
        self._camera_info_subscription = None
        self._joint_state_subscription = None
        self._robot_state_subscription = None
        self._image_topic = ""
        self._camera_info_topic = ""
        self._joint_state_topic = ""
        self._robot_state_topic = ""
        self._last_detection_time = 0.0

        self._action_server = ActionServer(
            self,
            RunCalibration,
            "~/run_calibration",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_callback_group,
        )

        self._preview_timer = self.create_timer(
            0.1,
            self._preview_timer_callback,
            callback_group=self._timer_callback_group,
        )
        self._autostart_timer = self.create_timer(
            0.5,
            self._autostart_timer_callback,
            callback_group=self._timer_callback_group,
        )

        # Initialize subscriptions with default job config to enable live preview immediately
        try:
            initial_job, _ = load_job_config(self._default_job_name, self._default_job_config_path)
            self._configure_subscriptions(initial_job)
            # Seed the last run request so "Start Run" button works immediately
            self._last_run_request = (
                self._default_job_name,
                self._default_job_config_path,
                self._autostart_solve_after_collection,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize subscriptions with default job: {str(e)}")

    def destroy_node(self):
        with self._lifecycle_lock:
            self._is_shutting_down = True
            self.get_logger().info("Shutting down coordinator node...")
            # 1. Cancel timers to stop triggering new work
            if hasattr(self, "_preview_timer"):
                self._preview_timer.cancel()
            if hasattr(self, "_autostart_timer"):
                self._autostart_timer.cancel()

            # 2. Stop subscriptions explicitly to quell callbacks
            self._image_subscription = None
            self._camera_info_subscription = None
            self._joint_state_subscription = None
            self._robot_state_subscription = None

            # 3. Clean up OpenCV
            try:
                if self._enable_preview and not self._preview_disabled:
                    cv2.destroyWindow(self._preview_window_name)
            except Exception:  # noqa: BLE001
                pass
            
            # 4. Final destruction
            return super().destroy_node()

    def _goal_callback(self, _goal_request) -> GoalResponse:
        with self._run_lock:
            if self._run_active:
                self.get_logger().warn("Rejecting calibration goal because another run is already active.")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        request = goal_handle.request
        return self._run_job(
            goal_handle=goal_handle,
            job_name=request.job_name or self._default_job_name,
            job_config_path=request.job_config_path or self._default_job_config_path,
            solve_after_collection=request.solve_after_collection,
        )

    def _autostart_timer_callback(self) -> None:
        pending_run: tuple[str, str, bool] | None = None
        delay_sec = 0.0

        with self._run_lock:
            if self._run_active:
                return

            if self._pending_background_run is not None:
                pending_run = self._pending_background_run
                self._pending_background_run = None
            elif self._autostart and not self._autostart_started:
                if not (
                    self._autostart_job_name
                    or self._default_job_name
                    or self._autostart_job_config_path
                ):
                    return
                self._autostart_started = True
                pending_run = (
                    self._autostart_job_name or self._default_job_name,
                    self._autostart_job_config_path or self._default_job_config_path,
                    self._autostart_solve_after_collection,
                )
                delay_sec = self._autostart_delay_sec

        if pending_run is None:
            return

        threading.Thread(
            target=self._run_background_job,
            args=(*pending_run, delay_sec),
            daemon=True,
        ).start()

    def _run_background_job(
        self,
        job_name: str,
        job_config_path: str,
        solve_after_collection: bool,
        delay_sec: float = 0.0,
    ) -> None:
        if delay_sec > 0.0:
            self.get_logger().info(f"Delaying background job start by {delay_sec}s...")
            time.sleep(delay_sec)
        self.get_logger().info(
            f"Starting background calibration/debug run for job '{job_name or self._default_job_name}'."
        )
        try:
            result = self._run_job(
                goal_handle=None,
                job_name=job_name,
                job_config_path=job_config_path,
                solve_after_collection=solve_after_collection,
            )
            if result.error_message:
                self.get_logger().error(f"Background job finished with ERROR: {result.error_message}")
            else:
                self.get_logger().info(f"Background job finished with state: {result.final_state}")
        except Exception as e:
            self.get_logger().error(f"Background job failed with exception: {str(e)}")
            self.get_logger().error(traceback.format_exc())

    def _run_job(
        self,
        *,
        goal_handle: ServerGoalHandle | None,
        job_name: str,
        job_config_path: str,
        solve_after_collection: bool,
    ):
        result = RunCalibration.Result()
        result.success = False
        result.job_name = job_name
        result.final_state = "FAILED"
        result.run_directory = ""
        result.attempted_samples = 0
        result.valid_samples = 0
        result.chosen_method = ""
        result.transform_file = ""
        result.error_message = ""

        with self._run_lock:
            if self._run_active:
                result.error_message = "Another calibration run is already active."
                if goal_handle is not None:
                    goal_handle.abort()
                return result
            self._run_active = True
            self._last_run_request = (job_name, job_config_path, solve_after_collection)

        try:
            try:
                job, config_path = load_job_config(job_name, job_config_path)

                # Override for fake hardware if requested by launch/parameter
                if self._use_fake_hardware and job.robot_pose_source == "robot_state":
                    self.get_logger().info(
                        "Overriding robot_pose_source to 'fk' because use_fake_hardware is true."
                    )
                    job = replace(job, robot_pose_source="fk", reject_on_fk_mismatch=False)

                result.job_name = job.job_name
                dictionary, board = (
                    create_charuco_board(job.board) if job.capture_mode == "charuco" else (None, None)
                )
            except Exception as error:  # noqa: BLE001
                result.error_message = str(error)
                if goal_handle is not None:
                    goal_handle.abort()
                return result

            self.get_logger().info("Configuring subscriptions and services...")
            self._configure_subscriptions(job)
            self._configure_fk_client(job)
            with self._lifecycle_lock:
                if self._executor_bridge is None or job.executor_namespace != self._last_executor_namespace:
                    self._executor_bridge = ExecutorBridge(
                        self, job.executor_namespace, callback_group=self._timer_callback_group
                    )
                    self._last_executor_namespace = job.executor_namespace
            self.get_logger().info("Waiting for dependencies (MoveIt and Executor)...")
            self._update_preview_status(
                [f"Job: {job.job_name}", f"Sequence: {job.sequence_name}", "Waiting for dependencies..."]
            )

            try:
                self._executor_bridge.wait_for_ready(timeout_sec=20.0)
                self.get_logger().info("Executor bridge ready.")
                if self._fk_client is None or not self._fk_client.wait_for_service(timeout_sec=20.0):
                    raise RuntimeError("MoveIt's /compute_fk service is not available.")
                self.get_logger().info("MoveIt FK service ready.")
                
                self.get_logger().info("Waiting for initial data (camera/joints)...")
                self._wait_for_initial_data(job, timeout_sec=10.0)
                self.get_logger().info("Initial data received and fresh.")
            except Exception as error:  # noqa: BLE001
                self.get_logger().error(f"Dependency wait failed: {str(error)}")
                result.error_message = str(error)
                if goal_handle is not None:
                    goal_handle.abort()
                return result

            self.get_logger().info("Starting manual calibration session...")
            self._current_valid_samples = []
            self._current_samples_job_name = job.job_name
            self._set_preview_calibration_lines(
                [
                    "Calibration: waiting for samples",
                    f"Job: {job.job_name}",
                ]
            )
            run_paths = make_run_paths(job)
            with self._run_lock:
                self._current_run_paths = run_paths
            result.run_directory = str(run_paths.run_dir)
            with config_path.open("r", encoding="utf-8") as stream:
                save_job_config(run_paths, yaml.safe_load(stream) or {})

            # try:
            #     _sequence_goal_handle, sequence_result_future = self._executor_bridge.start_sequence(
            #         job.sequence_name,
            #         force_wait_each_step=True,
            #     )
            #     self.get_logger().info(f"Sequence goal sent to executor for '{job.sequence_name}'.")
            # except Exception as error:  # noqa: BLE001
            #     self.get_logger().error(f"Failed to start sequence: {str(error)}")
            #     result.error_message = str(error)
            #     if goal_handle is not None:
            #         goal_handle.abort()
            #     return result

            self.get_logger().info("Manual collection mode active. Move the robot and click 'Add Sample'.")
            
            while rclpy.ok():
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    result.final_state = "CANCELED"
                    result.error_message = "Calibration canceled by the action client."
                    goal_handle.canceled()
                    return result

                valid_count = len(self._current_valid_samples)
                
                # In manual mode, we just wait until the user decides to solve or stop.
                # We'll pulse feedback so the user knows we are alive.
                self._publish_feedback(
                    goal_handle,
                    state="MANUAL_WAITING",
                    active_point_name="Manual Guiding",
                    active_step_index=valid_count,
                    total_steps=job.required_min_samples,
                    attempted_samples=result.attempted_samples,
                    valid_samples=valid_count,
                    retry_count=0,
                    last_message="Waiting for manual samples...",
                )

                self._update_preview_status(
                    [
                        f"Job: {job.job_name}",
                        "Mode: MANUAL HAND-GUIDING",
                        f"Samples: {valid_count}/{job.required_min_samples}",
                        "Click 'Add Sample' at each pose",
                    ]
                )

                # Check if we should exit (manual stop signaled by _run_active = False)
                with self._run_lock:
                    if not self._run_active:
                        result.final_state = "STOPPED_BY_USER"
                        break
                
                time.sleep(0.5)

            # Finalize collection result
            result.valid_samples = len(self._current_valid_samples)

            if result.valid_samples < job.required_min_samples:
                result.error_message = (
                    f"Collected only {result.valid_samples} valid samples. "
                    f"Need at least {job.required_min_samples}."
                )
                if goal_handle is not None:
                    goal_handle.abort()
                return result

            if solve_after_collection:
                try:
                    self.get_logger().info(f"Running ChArUco solver on {result.valid_samples} samples...")
                    solve_payload = solve_calibration_run(run_paths.run_dir)
                    chosen = solve_payload["methods"][str(solve_payload["chosen_method"])]
                    self._store_latest_handeye_solution(
                        job.job_name,
                        chosen,
                        f"saved run {run_paths.run_dir}",
                    )
                    result.chosen_method = str(solve_payload["chosen_method"])
                    result.transform_file = str(solve_payload["result_handeye_yaml"])
                    result.final_state = "SUCCEEDED"
                except Exception as error:  # noqa: BLE001
                    self.get_logger().error(f"Collected samples, but solving failed: {error}")
                    result.error_message = f"Collected samples, but solving failed: {error}"
                    if goal_handle is not None:
                        goal_handle.abort()
                    return result
            else:
                result.final_state = "COLLECTED"

            result.success = True
            self._publish_feedback(
                goal_handle,
                state=result.final_state,
                active_point_name="",
                active_step_index=0,
                total_steps=0,
                attempted_samples=result.attempted_samples,
                valid_samples=result.valid_samples,
                retry_count=0,
                last_message="Manual calibration job completed.",
            )
            if goal_handle is not None:
                goal_handle.succeed()
            return result
        finally:
            with self._run_lock:
                self._run_active = False
                self._current_run_paths = None

    def _configure_subscriptions(self, job: CalibrationJobConfig) -> None:
        image_topic = job.image_topic if job.capture_mode == "charuco" else ""
        camera_info_topic = job.camera_info_topic if job.capture_mode == "charuco" else ""
        self._replace_subscription(
            subscription_attr="_image_subscription",
            topic_attr="_image_topic",
            desired_topic=image_topic,
            msg_type=Image,
            callback=self._image_callback,
            qos=IMAGE_QOS,
        )
        self._replace_subscription(
            subscription_attr="_camera_info_subscription",
            topic_attr="_camera_info_topic",
            desired_topic=camera_info_topic,
            msg_type=CameraInfo,
            callback=self._camera_info_callback,
            qos=IMAGE_QOS,
        )
        self._replace_subscription(
            subscription_attr="_joint_state_subscription",
            topic_attr="_joint_state_topic",
            desired_topic=job.joint_state_topic,
            msg_type=JointState,
            callback=self._joint_state_callback,
        )

        robot_state_topic = job.robot_state_topic if job.robot_state_topic else ""
        self._replace_subscription(
            subscription_attr="_robot_state_subscription",
            topic_attr="_robot_state_topic",
            desired_topic=robot_state_topic,
            msg_type=FrankaRobotState,
            callback=self._robot_state_callback,
        )

    def _configure_fk_client(self, job: CalibrationJobConfig) -> None:
        if self._fk_client is not None and self._fk_service_name != job.fk_service_name:
            self.destroy_client(self._fk_client)
            self._fk_client = None
            self._fk_service_name = ""

        if self._fk_client is None:
            self._fk_client = self.create_client(
                GetPositionFK,
                job.fk_service_name,
                callback_group=self._client_callback_group,
            )
            self._fk_service_name = job.fk_service_name

        self._preview_fk_link_name = job.fk_link_name
        self._preview_base_frame = job.robot_base_frame
        self._preview_robot_pose_source = job.robot_pose_source

    def _configure_ik_client(self) -> None:
        if self._ik_client is not None and self._ik_service_name != self._ik_service_name_param:
            self.destroy_client(self._ik_client)
            self._ik_client = None
            self._ik_service_name = ""

        if self._ik_client is None:
            self._ik_client = self.create_client(
                GetPositionIK,
                self._ik_service_name_param,
                callback_group=self._client_callback_group,
            )
            self._ik_service_name = self._ik_service_name_param

    def _prepare_verification_stack(self, job: CalibrationJobConfig, timeout_sec: float = 20.0) -> None:
        self._configure_subscriptions(job)
        self._configure_fk_client(job)
        self._configure_ik_client()
        with self._lifecycle_lock:
            if self._executor_bridge is None or job.executor_namespace != self._last_executor_namespace:
                self._executor_bridge = ExecutorBridge(
                    self, job.executor_namespace, callback_group=self._timer_callback_group
                )
                self._last_executor_namespace = job.executor_namespace
        self._executor_bridge.wait_for_ready(timeout_sec=timeout_sec)
        if self._fk_client is None or not self._fk_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("MoveIt's /compute_fk service is not available.")
        if self._ik_client is None or not self._ik_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("MoveIt's /compute_ik service is not available.")

    def _replace_subscription(
        self,
        *,
        subscription_attr: str,
        topic_attr: str,
        desired_topic: str,
        msg_type,
        callback,
        qos=10,
    ) -> None:
        with self._lifecycle_lock:
            if self._is_shutting_down:
                return

            current_topic = getattr(self, topic_attr)
            current_subscription = getattr(self, subscription_attr)

            # Recreate IF the topic has changed or if we don't have a subscription yet.
            # Avoid forced recreation due to QoSProfile type checking which causes
            # InvalidHandle errors when switching modes.
            should_recreate = current_topic != desired_topic or (
                desired_topic and current_subscription is None
            )

            if current_subscription is not None and should_recreate:
                self.get_logger().info(f"Replacing subscription for {subscription_attr} on {desired_topic}")
                self.destroy_subscription(current_subscription)
                setattr(self, subscription_attr, None)
                setattr(self, topic_attr, "")

            if desired_topic and getattr(self, subscription_attr) is None:
                setattr(
                    self,
                    subscription_attr,
                    self.create_subscription(
                        msg_type, desired_topic, callback, qos, callback_group=self._timer_callback_group
                    ),
                )
                setattr(self, topic_attr, desired_topic)

    def _image_callback(self, msg: Image) -> None:
        received_time = time.monotonic()
        with self._latest_lock:
            self._latest_image = (msg, received_time)
        if self._enable_preview and not self._preview_disabled:
            now = time.monotonic()
            if (now - self._last_detection_time) < 0.1: # Throttle to 10Hz
                return
            self._last_detection_time = now

            try:
                preview = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                
                # Live ChArUco axes for manual mode
                info_entry = self._latest_camera_info
                if info_entry is not None:
                    info_msg, _ = info_entry
                    try:
                        # Use default job config for board parameters
                        job, _ = load_job_config(self._default_job_name, self._default_job_config_path)
                        dict_charuco, board = create_charuco_board(job.board)
                        intrinsics = intrinsics_from_camera_info(info_msg)
                        
                        detect_res = detect_charuco_pose(
                            preview,
                            dictionary=dict_charuco,
                            board=board,
                            intrinsics=intrinsics,
                            min_corners=4, # Low threshold for live visual feedback
                        )
                        if detect_res.success:
                            cv2.drawFrameAxes(
                                preview, 
                                intrinsics.camera_matrix, 
                                intrinsics.dist_coeffs, 
                                detect_res.rvec, 
                                detect_res.tvec, 
                                0.1
                            )

                            # --- AUTO CAPTURE LOGIC ---
                            # If a job is active, automatically try to save the sample
                            with self._run_lock:
                                run_active = self._run_active
                            
                            if run_active:
                                try:
                                    curr_ee = self._current_base_to_ee(job=job)
                                    if curr_ee is not None:
                                        # 1. Track movement for settle detection
                                        if self._last_preview_ee_pose is not None:
                                            res = residual_between_transforms(self._last_preview_ee_pose, curr_ee)
                                            if res.translation_error_m > 0.005 or res.rotation_error_deg > 1.0:
                                                # Robot is moving
                                                self._last_move_time = time.monotonic()
                                                self._stationary_start_time = None
                                            else:
                                                # Robot is stationary
                                                if self._stationary_start_time is None:
                                                    self._stationary_start_time = time.monotonic()
                                        
                                        self._last_preview_ee_pose = curr_ee.copy()

                                        # 2. Check for significant move from last SAVED sample
                                        significant, reason = self._is_new_pose_significant(curr_ee)
                                        
                                        # 3. Check if settled (stationary for 1.0s)
                                        is_settled = False
                                        if self._stationary_start_time is not None:
                                            if (time.monotonic() - self._stationary_start_time) >= 1.0:
                                                is_settled = True

                                        if significant and is_settled:
                                            # We have board, we have pose, it's new, AND it's stable!
                                            record = {
                                                "base_to_ee": serialize_transform(curr_ee),
                                                "target_to_camera": serialize_transform(detect_res.target_to_camera),
                                            }
                                            self._register_valid_sample(record)
                                            self._last_saved_base_to_ee = curr_ee.copy()
                                            self.get_logger().info(f"Auto-captured stable sample! Reason: {reason}")
                                except Exception as e:
                                    self.get_logger().error(f"Auto-capture attempt failed: {e}")
                            # --------------------------
                    except Exception: # noqa: BLE001
                        pass
                        
            except Exception:  # noqa: BLE001
                return
            with self._preview_lock:
                self._preview_frame = preview

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        with self._latest_lock:
            self._latest_camera_info = (msg, time.monotonic())

    def _joint_state_callback(self, msg: JointState) -> None:
        with self._latest_lock:
            self._latest_joint_state = (msg, time.monotonic())

    def _robot_state_callback(self, msg: FrankaRobotState) -> None:
        with self._latest_lock:
            self._latest_robot_state = (msg, time.monotonic())

    def _wait_for_initial_data(self, job: CalibrationJobConfig, timeout_sec: float) -> None:
        """Wait until at least one valid message has been received on required topics."""
        start_time = time.monotonic()
        while time.monotonic() - start_time < timeout_sec:
            with self._latest_lock:
                has_joints = self._latest_joint_state is not None
                has_image = self._latest_image is not None or job.capture_mode != "charuco"
                has_info = self._latest_camera_info is not None or job.capture_mode != "charuco"
                has_robot = self._latest_robot_state is not None or job.robot_pose_source != "robot_state"

            if has_joints and has_image and has_info and has_robot:
                return
            time.sleep(0.5)

        missing = []
        with self._latest_lock:
            if self._latest_joint_state is None: missing.append("joint_states")
            if job.capture_mode == "charuco":
                if self._latest_image is None: missing.append("image")
                if self._latest_camera_info is None: missing.append("camera_info")
            if job.robot_pose_source == "robot_state" and self._latest_robot_state is None:
                missing.append("robot_state")
        
        raise RuntimeError(f"Timed out waiting for initial data. Missing: {', '.join(missing)}")

    def _snapshot(self, job: CalibrationJobConfig) -> Snapshot:
        with self._latest_lock:
            image_entry = copy.deepcopy(self._latest_image)
            camera_info_entry = copy.deepcopy(self._latest_camera_info)
            joint_state_entry = copy.deepcopy(self._latest_joint_state)
            robot_state_entry = copy.deepcopy(self._latest_robot_state)

        now_mono = time.monotonic()
        if joint_state_entry is None:
            raise RuntimeError(f"No joint state has been received yet on {job.joint_state_topic}.")
        image_msg = None
        camera_info_msg = None
        joint_state_msg, joint_state_time = joint_state_entry

        if job.capture_mode == "charuco":
            if image_entry is None:
                raise RuntimeError(f"No image has been received yet on {job.image_topic}.")
            if camera_info_entry is None:
                raise RuntimeError(f"No camera info has been received yet on {job.camera_info_topic}.")
            image_msg, image_time = image_entry
            camera_info_msg, camera_info_time = camera_info_entry
            if now_mono - image_time > job.freshness.image_max_age_sec:
                raise RuntimeError("Latest image is stale.")
            if now_mono - camera_info_time > job.freshness.camera_info_max_age_sec:
                raise RuntimeError("Latest camera info is stale.")
        image_msg, image_time = image_entry
        camera_info_msg, camera_info_time = camera_info_entry
        if now_mono - image_time > job.freshness.image_max_age_sec:
            raise RuntimeError("Latest image is stale.")
        if now_mono - camera_info_time > job.freshness.camera_info_max_age_sec:
            raise RuntimeError("Latest camera info is stale.")

        # Pre-process image/intrinsics for downstream consumers
        image_bgr = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        intrinsics = intrinsics_from_camera_info(camera_info_msg)

        robot_state_msg = None
        if job.robot_pose_source == "robot_state":
            if robot_state_entry is None:
                raise RuntimeError(f"No robot state has been received yet on {job.robot_state_topic}.")
            robot_state_msg, robot_state_time = robot_state_entry
            if now_mono - robot_state_time > job.freshness.robot_state_max_age_sec:
                raise RuntimeError("Latest robot state is stale.")

        return Snapshot(
            image=image_msg,
            image_bgr=image_bgr,
            camera_info=camera_info_msg,
            intrinsics=intrinsics,
            joint_state=joint_state_msg,
            robot_state=robot_state_msg,
        )

    def _wait_for_snapshot(self, job: CalibrationJobConfig, timeout_sec: float = 3.0) -> Snapshot:
        deadline = time.monotonic() + timeout_sec
        last_error = "Timed out waiting for fresh sensor data."
        while time.monotonic() < deadline:
            try:
                return self._snapshot(job)
            except RuntimeError as error:
                last_error = str(error)
                time.sleep(0.1)
        raise RuntimeError(last_error)

    def _capture_sample_for_waiting_step(
        self,
        *,
        job: CalibrationJobConfig,
        run_paths,
        dictionary,
        board,
        step_index: int,
        point_name: str,
        retry_count: int,
    ) -> tuple[bool, str]:
        time.sleep(job.settle_time_sec)
        snapshot = self._snapshot(job)

        if job.capture_mode == "mock":
            return self._capture_mock_sample_for_waiting_step(
                job=job,
                run_paths=run_paths,
                snapshot=snapshot,
                step_index=step_index,
                point_name=point_name,
                retry_count=retry_count,
            )

        image_bgr = self._cv_bridge.imgmsg_to_cv2(snapshot.image, desired_encoding="bgr8")
        intrinsics = intrinsics_from_camera_info(snapshot.camera_info)
        detection = detect_charuco_pose(
            image_bgr,
            dictionary=dictionary,
            board=board,
            intrinsics=intrinsics,
            min_corners=job.min_corners,
        )

        sample_stem = sample_file_stem(step_index, point_name, retry_count)
        detection_json_path = run_paths.detections_dir / f"{sample_stem}.json"
        record: dict[str, Any] = {
            "captured_at": utc_now_iso(),
            "step_index": step_index,
            "point_name": point_name,
            "attempt_index": retry_count,
            "valid_sample": False,
            "detection_path": str(detection_json_path),
            "camera_info": self._serialize_camera_info(snapshot.camera_info),
            "joint_state": self._serialize_joint_state(snapshot.joint_state),
            "intrinsics_path": str(intrinsics.source_path),
            "robot_pose_source": job.robot_pose_source,
        }

        detection_payload = serialize_detection(detection)
        save_json(detection_json_path, detection_payload)
        record["detection"] = detection_payload

        if not detection.success or detection.target_to_camera is None:
            record["invalid_reason"] = detection.error_message or "ChArUco detection failed."
            append_jsonl(run_paths.samples_jsonl, record)
            self._set_preview_frame(detection.preview_bgr)
            return False, record["invalid_reason"]

        frame_id = job.robot_base_frame
        if job.robot_pose_source == "robot_state":
            base_to_ee = franka_matrix_to_transform(snapshot.robot_state.o_t_ee)
            # Default frame_id to configured robot_base_frame since raw o_t_ee has no header
            frame_id = job.robot_base_frame 
            fk_base_to_ee = compute_fk_transform(
                self._fk_client,
                snapshot.joint_state,
                fk_link_name=job.fk_link_name,
                frame_id=frame_id,
                timeout_sec=5.0,
            )
            base_pose_payload, fk_pose_payload, fk_residual_payload = serialize_pose_pair(
                base_to_ee, fk_base_to_ee
            )
            record["base_to_ee"] = base_pose_payload
            record["fk_base_to_ee"] = fk_pose_payload
            record["fk_validation_residual"] = fk_residual_payload
            if job.reject_on_fk_mismatch and (
                fk_residual_payload["translation_error_m"] > job.max_fk_translation_error_m
                or fk_residual_payload["rotation_error_deg"] > job.max_fk_rotation_error_deg
            ):
                record["valid_sample"] = False
                record["invalid_reason"] = (
                    "FK validation mismatch exceeded threshold: "
                    f"{fk_residual_payload['translation_error_m']:.4f} m, "
                    f"{fk_residual_payload['rotation_error_deg']:.2f} deg."
                )
                append_jsonl(run_paths.samples_jsonl, record)
                self._set_preview_frame(detection.preview_bgr)
                return False, record["invalid_reason"]
        else:
            fk_base_to_ee = compute_fk_transform(
                self._fk_client,
                snapshot.joint_state,
                fk_link_name=job.fk_link_name,
                frame_id=frame_id,
                timeout_sec=5.0,
            )
            record["base_to_ee"] = serialize_transform(fk_base_to_ee)
            record["fk_base_to_ee"] = serialize_transform(fk_base_to_ee)
            record["fk_validation_residual"] = {
                "translation_error_m": 0.0,
                "rotation_error_deg": 0.0,
            }

        record["target_to_camera"] = serialize_transform(detection.target_to_camera)
        record["valid_sample"] = True
        append_jsonl(run_paths.samples_jsonl, record)
        self._register_valid_sample(record)
        self._set_preview_frame(detection.preview_bgr)
        return True, f"Saved valid sample for {point_name}."

    def _capture_mock_sample_for_waiting_step(
        self,
        *,
        job: CalibrationJobConfig,
        run_paths,
        snapshot: Snapshot,
        step_index: int,
        point_name: str,
        retry_count: int,
    ) -> tuple[bool, str]:
        preview = self._build_mock_preview(point_name=point_name, step_index=step_index, retry_count=retry_count)
        sample_stem = sample_file_stem(step_index, point_name, retry_count)
        detection_json_path = run_paths.detections_dir / f"{sample_stem}.json"
        
        base_to_ee = compute_fk_transform(
            self._fk_client,
            snapshot.joint_state,
            fk_link_name=job.fk_link_name,
            frame_id=job.robot_base_frame,
            timeout_sec=5.0,
        )
        mock_target_to_camera = self._mock_target_to_camera(step_index)

        record: dict[str, Any] = {
            "captured_at": utc_now_iso(),
            "step_index": step_index,
            "point_name": point_name,
            "attempt_index": retry_count,
            "valid_sample": True,
            "detection_path": str(detection_json_path),
            "camera_info": self._mock_camera_info(job),
            "joint_state": self._serialize_joint_state(snapshot.joint_state),
            "intrinsics_path": "",
            "robot_pose_source": "fk",
            "capture_mode": "mock",
            "base_to_ee": serialize_transform(base_to_ee),
            "fk_base_to_ee": serialize_transform(base_to_ee),
            "fk_validation_residual": {
                "translation_error_m": 0.0,
                "rotation_error_deg": 0.0,
            },
            "target_to_camera": serialize_transform(mock_target_to_camera),
        }
        detection_payload = {
            "success": True,
            "error_message": "",
            "mode": "mock",
            "target_to_camera": serialize_transform(mock_target_to_camera),
            "reprojection_error_px": 0.0,
        }
        save_json(detection_json_path, detection_payload)
        record["detection"] = detection_payload
        append_jsonl(run_paths.samples_jsonl, record)
        self._register_valid_sample(record)
        self._set_preview_frame(preview)
        return True, f"Saved mock sample for {point_name}."

    def _serialize_camera_info(self, camera_info: CameraInfo) -> dict[str, Any]:
        return {
            "header_frame_id": camera_info.header.frame_id,
            "height": int(camera_info.height),
            "width": int(camera_info.width),
            "distortion_model": camera_info.distortion_model,
            "d": list(camera_info.d),
            "k": list(camera_info.k),
            "r": list(camera_info.r),
            "p": list(camera_info.p),
        }

    def _mock_camera_info(self, job: CalibrationJobConfig) -> dict[str, Any]:
        return {
            "header_frame_id": "mock_camera",
            "height": 480,
            "width": 640,
            "distortion_model": "plumb_bob",
            "d": [],
            "k": [1.0, 0.0, 320.0, 0.0, 1.0, 240.0, 0.0, 0.0, 1.0],
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "p": [1.0, 0.0, 320.0, 0.0, 0.0, 1.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            "mode": job.capture_mode,
        }

    def _serialize_joint_state(self, joint_state: JointState) -> dict[str, Any]:
        return {
            "name": list(joint_state.name),
            "position": list(joint_state.position),
            "velocity": list(joint_state.velocity),
            "effort": list(joint_state.effort),
        }

    def _store_latest_handeye_solution(
        self,
        job_name: str,
        chosen_solution: dict[str, Any],
        source: str,
    ) -> None:
        with self._verification_lock:
            self._latest_handeye_job_name = job_name
            self._latest_handeye_solution = copy.deepcopy(chosen_solution)
            self._latest_handeye_source = source

            # Automatic Archival: Export to a fixed location in teach_data
            # We keep this inside the lock to prevent concurrent writes to the same archive file
            try:
                latest_path = self._latest_calibration_archive_path()
                self.get_logger().info(f"Automatically archiving calibration result to {latest_path}")
                save_yaml(latest_path, chosen_solution)
            except Exception as e:
                self.get_logger().warn(f"Failed to auto-archive calibration: {str(e)}")

    def _latest_calibration_archive_path(self) -> Path:
        return detect_workspace_root() / "teach_data" / "latest_calibration.yaml"

    def _load_latest_saved_handeye_solution(
        self, job: CalibrationJobConfig
    ) -> tuple[dict[str, Any] | None, str]:
        run_root = job.output_root / job.job_name
        if not run_root.exists():
            return None, ""

        for result_path in sorted(run_root.glob("*/result_handeye.yaml"), reverse=True):
            try:
                payload = load_calibration_result(result_path.parent)
            except (OSError, ValueError, FileNotFoundError):
                continue
            return {
                "camera_to_gripper": payload["camera_to_gripper"],
                "gripper_to_camera": payload["gripper_to_camera"],
                "metrics": payload.get("metrics", {}),
            }, str(result_path)
        return None, ""

    def _load_archived_handeye_solution(self) -> tuple[dict[str, Any] | None, str]:
        archive_path = self._latest_calibration_archive_path()
        if not archive_path.exists():
            return None, ""
        try:
            with archive_path.open("r", encoding="utf-8") as stream:
                payload = yaml.safe_load(stream) or {}
        except OSError:
            return None, ""
        if not isinstance(payload, dict):
            return None, ""
        camera_to_gripper = payload.get("camera_to_gripper")
        gripper_to_camera = payload.get("gripper_to_camera")
        if not camera_to_gripper or not gripper_to_camera:
            return None, ""
        return {
            "camera_to_gripper": camera_to_gripper,
            "gripper_to_camera": gripper_to_camera,
            "metrics": payload.get("metrics", {}),
        }, str(archive_path)

    def _resolve_handeye_solution(self, job: CalibrationJobConfig) -> tuple[dict[str, Any], str]:
        with self._verification_lock:
            if (
                self._latest_handeye_solution is not None
                # If we manually loaded an archive, we ignore the job name check 
                # because the user explicitly wants to use it.
                and (self._latest_handeye_job_name == job.job_name or self._latest_handeye_source.startswith("manually"))
            ):
                return copy.deepcopy(self._latest_handeye_solution), self._latest_handeye_source

        if self._current_samples_job_name == job.job_name and len(self._current_valid_samples) >= 3:
            solution = solve_handeye_from_samples(self._current_valid_samples)
            chosen_method = str(solution["chosen_method"])
            chosen = solution["methods"][chosen_method]
            source = f"live estimate ({len(self._current_valid_samples)} samples)"
            self._store_latest_handeye_solution(job.job_name, chosen, source)
            return copy.deepcopy(chosen), source

        saved_solution, source = self._load_latest_saved_handeye_solution(job)
        if saved_solution is not None:
            self._store_latest_handeye_solution(job.job_name, saved_solution, source)
            return copy.deepcopy(saved_solution), source

        archived_solution, source = self._load_archived_handeye_solution()
        if archived_solution is not None:
            self._store_latest_handeye_solution(job.job_name, archived_solution, source)
            return copy.deepcopy(archived_solution), source

        raise RuntimeError("No solved hand-eye result is available yet. Run calibration first.")

    def _handle_save_archive_key(self) -> None:
        with self._verification_lock:
            solution = copy.deepcopy(self._latest_handeye_solution)
            job_name = self._latest_handeye_job_name
        
        if solution is None:
            self._update_preview_status(["Save failed: no solution in memory"])
            return

        try:
            latest_path = self._latest_calibration_archive_path()
            self.get_logger().info(f"Manually archiving calibration result to {latest_path}")
            save_yaml(latest_path, solution)
            self._update_preview_status([f"Archived to {latest_path.name}"])
        except Exception as e:
            self.get_logger().error(f"Failed to manually archive: {str(e)}")
            self._update_preview_status([f"Save failed: {str(e)}"])

    def _handle_load_archive_key(self) -> None:
        archived_solution, source = self._load_archived_handeye_solution()
        if archived_solution is None:
            self._update_preview_status(["Load failed: no archive found"])
            return
        
        with self._verification_lock:
            # We use a dummy job name or current job name if available
            job_preview_name = self._latest_handeye_job_name or self._default_job_name
            self._store_latest_handeye_solution(job_preview_name, archived_solution, f"manually loaded from {source}")
        
        self._update_preview_status([f"Loaded {Path(source).name}"])

    def _has_verification_input(self, job: CalibrationJobConfig) -> bool:
        try:
            self._resolve_handeye_solution(job)
        except RuntimeError:
            return False
        return True

    def _current_base_to_ee(self, snapshot: Snapshot | None = None, job: CalibrationJobConfig | None = None) -> np.ndarray | None:
        """Helper to get the current base-to-ee transform from the snapshot or cached state."""
        # 1. Determine Source
        source = "robot_state"
        if job is not None:
            source = job.robot_pose_source
        elif self._live_detect_config is not None:
            # If we happen to have a live config cached from the last run
            source = self._preview_robot_pose_source

        # 2. Extract robot state
        robot_state = None
        if snapshot is not None:
            robot_state = snapshot.robot_state
        else:
            with self._latest_lock:
                entry = self._latest_robot_state
                if entry is not None:
                    robot_state, _ = entry

        # 3. Pull transform
        if source == "robot_state":
            if robot_state is not None:
                # Check freshness
                _, entry_time = self._latest_robot_state
                if (time.monotonic() - entry_time) > 0.5:
                    return None  # Stale data
                return franka_matrix_to_transform(robot_state.o_t_ee)
            return None # Return None if requesting robot_state but it's missing
        
        # 4. Fallback to FK if required or if FCL data is missing (Wait, user said NO fallback)
        # Actually, if we are in FK mode, we need snapshot/joint_state
        joint_state = None
        if snapshot is not None:
            joint_state = snapshot.joint_state
        else:
            with self._latest_lock:
                entry = self._latest_joint_state
                if entry is not None:
                    joint_state, _ = entry
        
        if joint_state is None or job is None:
            return None

        return compute_fk_transform(
            self._fk_client,
            joint_state,
            fk_link_name=job.fk_link_name,
            frame_id=job.robot_base_frame,
            timeout_sec=5.0,
        )

    def _select_verification_charuco_point(self, board) -> tuple[np.ndarray, int]:
        object_points = np.asarray(board.chessboardCorners, dtype=np.float64).reshape(-1, 3)
        if len(object_points) == 0:
            raise RuntimeError("The configured ChArUco board contains no corners.")
        centroid = np.mean(object_points, axis=0)
        selected_index = int(np.argmin(np.linalg.norm(object_points - centroid, axis=1)))
        return object_points[selected_index], selected_index

    def _verification_pose_guard(
        self,
        current_base_to_ee: np.ndarray,
        desired_base_to_ee: np.ndarray,
    ) -> None:
        delta_m = float(np.linalg.norm(desired_base_to_ee[:3, 3] - current_base_to_ee[:3, 3]))
        if delta_m > self._verification_max_translation_m:
            raise RuntimeError(
                f"Verification move is too large ({delta_m:.3f} m > "
                f"{self._verification_max_translation_m:.3f} m). Reposition closer first."
            )

        x, y, z = desired_base_to_ee[:3, 3]
        if not (
            self._verification_workspace_min_x <= x <= self._verification_workspace_max_x
            and self._verification_workspace_min_y <= y <= self._verification_workspace_max_y
            and self._verification_workspace_min_z <= z <= self._verification_workspace_max_z
        ):
            raise RuntimeError(
                "Verification target is outside the guarded workspace bounds "
                f"(x={x:+.3f}, y={y:+.3f}, z={z:+.3f})."
            )

    def _build_real_verification_target(
        self,
        job: CalibrationJobConfig,
        snapshot: Snapshot,
    ) -> dict[str, Any]:
        intrinsics = intrinsics_from_camera_info(snapshot.camera_info)
        image_bgr = self._cv_bridge.imgmsg_to_cv2(snapshot.image, desired_encoding="bgr8")

        self.get_logger().info("Verification using ArUco detection (DICT_APRILTAG_36h11, 6cm).")
        aruco_dict = get_dictionary("DICT_APRILTAG_36h11")
        marker_length_m = 0.06
        detection = detect_aruco_pose(
            image_bgr,
            dictionary=aruco_dict,
            intrinsics=intrinsics,
            marker_length_m=marker_length_m,
        )
        
        self._set_preview_frame(detection.preview_bgr)
        if not detection.success or detection.target_to_camera is None:
            raise RuntimeError(detection.error_message or "ArUco detection failed for verification.")

        chosen_solution, solution_source = self._resolve_handeye_solution(job)
        camera_to_gripper = np.asarray(
            chosen_solution["camera_to_gripper"]["matrix"],
            dtype=np.float64,
        )
        gripper_to_camera_payload = chosen_solution.get("gripper_to_camera")
        if gripper_to_camera_payload is None:
            gripper_to_camera = invert_transform(camera_to_gripper)
        else:
            gripper_to_camera = np.asarray(
                gripper_to_camera_payload["matrix"],
                dtype=np.float64,
            )

        current_base_to_ee = self._current_base_to_ee(snapshot, job)
        base_to_target = current_base_to_ee @ camera_to_gripper @ detection.target_to_camera
        
        marker_label = "aruco marker corner (+X, +Y)"

        # We want the End Effector to be directly above the marker, pointing DOWN towards it.
        # Marker Frame: Z is UP (out of the table). X and Y are along the marker edges.
        # EE Frame: Z is FORWARD (out of the gripper fingers).
        # To point the Gripper down at the marker, we rotate 180 degrees around the X-axis mapping (Y->-Y, Z->-Z).
        desired_target_to_ee = np.eye(4, dtype=np.float64)
        desired_target_to_ee[:3, :3] = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]
        ], dtype=np.float64)
        
        # Translate the EE to a corner of the 6cm marker (+0.03m X, +0.03m Y) and straight up. 
        desired_target_to_ee[:3, 3] = np.array([0.03, 0.03, self._verification_camera_standoff_m], dtype=np.float64)
        
        # Base to new EE is composed: Base -> Target -> new EE
        desired_base_to_ee = base_to_target @ desired_target_to_ee
        self._verification_pose_guard(current_base_to_ee, desired_base_to_ee)

        return {
            "fake_hardware": False,
            "selected_charuco_id": 0, # Placeholder
            "label": marker_label,
            "solution_source": solution_source,
            "base_to_ee": desired_base_to_ee,
        }

    def _build_fake_verification_target(
        self,
        job: CalibrationJobConfig,
        snapshot: Snapshot,
    ) -> dict[str, Any]:
        current_base_to_ee = self._current_base_to_ee(snapshot, job)
        candidate_offsets = np.asarray(
            [
                [0.04, 0.00, 0.02],
                [0.03, 0.03, 0.01],
                [0.03, -0.03, 0.01],
                [-0.03, 0.02, 0.02],
                [-0.02, -0.03, 0.02],
            ],
            dtype=np.float64,
        )
        for candidate_index in self._rng.permutation(len(candidate_offsets)):
            desired_base_to_ee = np.asarray(current_base_to_ee, dtype=np.float64).copy()
            desired_base_to_ee[:3, 3] += candidate_offsets[int(candidate_index)]
            try:
                self._verification_pose_guard(current_base_to_ee, desired_base_to_ee)
            except RuntimeError:
                continue
            return {
                "fake_hardware": True,
                "selected_charuco_id": None,
                "label": f"fake offset #{int(candidate_index) + 1}",
                "solution_source": "fake hardware preview",
                "base_to_ee": desired_base_to_ee,
            }
        raise RuntimeError("Could not find a guarded fake-hardware verification target.")

    def _solve_verification_ik(
        self,
        snapshot: Snapshot,
        job: CalibrationJobConfig,
        desired_base_to_ee: np.ndarray,
    ) -> JointState:
        if self._ik_client is None:
            raise RuntimeError("MoveIt's /compute_ik service is not configured.")

        request = GetPositionIK.Request()
        request.ik_request.group_name = str(self._planning_group)
        request.ik_request.pose_stamped.header.frame_id = job.robot_base_frame
        request.ik_request.pose_stamped.pose.position.x = float(desired_base_to_ee[0, 3])
        request.ik_request.pose_stamped.pose.position.y = float(desired_base_to_ee[1, 3])
        request.ik_request.pose_stamped.pose.position.z = float(desired_base_to_ee[2, 3])
        quat = matrix_to_quaternion(desired_base_to_ee[:3, :3])
        request.ik_request.pose_stamped.pose.orientation.x = float(quat[0])
        request.ik_request.pose_stamped.pose.orientation.y = float(quat[1])
        request.ik_request.pose_stamped.pose.orientation.z = float(quat[2])
        request.ik_request.pose_stamped.pose.orientation.w = float(quat[3])
        request.ik_request.ik_link_name = job.fk_link_name
        request.ik_request.avoid_collisions = True
        request.ik_request.robot_state.joint_state = copy.deepcopy(snapshot.joint_state)
        request.ik_request.timeout.sec = 1

        future = self._ik_client.call_async(request)
        completion = threading.Event()

        def _notify(_future) -> None:
            completion.set()

        future.add_done_callback(_notify)
        if future.done():
            completion.set()
        if not completion.wait(5.0):
            raise RuntimeError("Timed out waiting for MoveIt's /compute_ik service.")

        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt's /compute_ik service returned no response.")
        if response.error_code.val != 1 or not response.solution.joint_state.name:
            raise RuntimeError(
                f"MoveIt's /compute_ik failed with error code {response.error_code.val}."
            )
        return copy.deepcopy(response.solution.joint_state)

    def _write_verification_point(
        self,
        point_name: str,
        job: CalibrationJobConfig,
        joint_state: JointState,
    ) -> str:
        joint_names = list(joint_state.name)
        positions = list(joint_state.position)
        velocities = list(joint_state.velocity)
        efforts = list(joint_state.effort)
        while len(velocities) < len(joint_names):
            velocities.append(0.0)
        while len(efforts) < len(joint_names):
            efforts.append(0.0)

        point_payload = {
            "name": point_name,
            "type": "point",
            "captured_at": utc_now_iso(),
            "joint_states_topic": job.joint_state_topic,
            "joint_state_stamp": float(joint_state.header.stamp.sec)
            + float(joint_state.header.stamp.nanosec) * 1e-9,
            "joint_names": joint_names,
            "positions": positions,
            "velocities": velocities[: len(joint_names)],
            "efforts": efforts[: len(joint_names)],
        }

        point_path = self._teach_points_dir / f"{point_name}.yaml"
        save_yaml(point_path, point_payload)
        return str(point_path)

    def _verification_plan_is_fresh_locked(self) -> bool:
        if self._last_verification_plan is None:
            return False
        if (
            time.monotonic() - self._last_verification_plan.created_mono
            > self._verification_plan_ttl_sec
        ):
            self._last_verification_plan = None
            return False
        return True

    def _verification_plan_is_fresh(self) -> bool:
        with self._verification_lock:
            return self._verification_plan_is_fresh_locked()

    def _mock_target_to_camera(self, step_index: int):
        transform = np.eye(4, dtype=np.float64)
        transform[0, 3] = 0.25 + 0.01 * step_index
        transform[1, 3] = -0.03 + 0.005 * step_index
        transform[2, 3] = 0.45
        return transform

    def _build_mock_preview(self, *, point_name: str, step_index: int, retry_count: int):
        preview = np.zeros((480, 640, 3), dtype=np.uint8)
        lines = [
            "Mock Capture Mode",
            f"Point: {point_name}",
            f"Step: {step_index}",
            f"Retry: {retry_count}",
            "No camera required",
        ]
        y = 80
        for line in lines:
            cv2.putText(
                preview,
                line,
                (40, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
            )
            y += 50
        return preview

    def _publish_feedback(
        self,
        goal_handle: ServerGoalHandle | None,
        *,
        state: str,
        active_point_name: str,
        active_step_index: int,
        total_steps: int,
        attempted_samples: int,
        valid_samples: int,
        retry_count: int,
        last_message: str,
    ) -> None:
        if goal_handle is None:
            return
        feedback = RunCalibration.Feedback()
        feedback.state = state
        feedback.active_point_name = active_point_name
        feedback.active_step_index = int(active_step_index)
        feedback.total_steps = int(total_steps)
        feedback.attempted_samples = int(attempted_samples)
        feedback.valid_samples = int(valid_samples)
        feedback.retry_count = int(retry_count)
        feedback.last_message = last_message
        goal_handle.publish_feedback(feedback)

    def _update_preview_status(self, lines: list[str]) -> None:
        with self._preview_lock:
            self._preview_status_lines = lines

    def _set_preview_calibration_lines(self, lines: list[str]) -> None:
        with self._preview_lock:
            self._preview_calibration_lines = lines

    def _set_preview_verification_lines(self, lines: list[str]) -> None:
        with self._preview_lock:
            self._preview_verification_lines = lines

    def _set_preview_frame(self, frame_bgr) -> None:
        if not self._enable_preview or self._preview_disabled:
            return
        with self._preview_lock:
            self._preview_frame = frame_bgr.copy()

    def _preview_timer_callback(self) -> None:
        if not self._enable_preview or self._preview_disabled:
            return
        if not self._ensure_preview_window():
            return
        with self._preview_lock:
            frame = None if self._preview_frame is None else self._preview_frame.copy()
            status_lines = list(self._preview_status_lines)
            calibration_lines = list(self._preview_calibration_lines)
            verification_lines = list(self._preview_verification_lines)
        status_lines.extend(self._preview_ee_status_lines())
        status_lines.extend(calibration_lines)
        status_lines.extend(verification_lines)
        with self._run_lock:
            run_active = self._run_active
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "Waiting for preview data",
                (120, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
            )

        sidebar_width = 340
        frame_height, frame_width = frame.shape[:2]
        canvas_height = max(frame_height, 520)
        canvas = np.zeros((canvas_height, sidebar_width + frame_width, 3), dtype=np.uint8)
        canvas[:, :sidebar_width] = (25, 25, 25)
        canvas[:frame_height, sidebar_width : sidebar_width + frame_width] = frame

        cv2.putText(
            canvas,
            "Calibration Debug",
            (18, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
        )

        y = 72
        for line in status_lines:
            wrapped_lines = textwrap.wrap(line, width=28) or [line]
            for wrapped_line in wrapped_lines:
                cv2.putText(
                    canvas,
                    wrapped_line,
                    (18, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (0, 255, 255),
                    2,
                )
                y += 24
            y += 8

        verify_rect = (18, canvas_height - 380, sidebar_width - 36, 44)
        execute_rect = (18, canvas_height - 331, sidebar_width - 36, 44)
        save_rect = (18, canvas_height - 282, (sidebar_width - 36) // 2 - 4, 44)
        load_rect = (18 + (sidebar_width - 36) // 2 + 4, canvas_height - 282, (sidebar_width - 36) // 2 - 4, 44)
        add_sample_rect = (18, canvas_height - 233, sidebar_width - 36, 44)
        solve_rect = (18, canvas_height - 184, sidebar_width - 36, 44) # NEW
        start_rect = (18, canvas_height - 130, sidebar_width - 36, 54)
        restart_rect = (18, canvas_height - 66, sidebar_width - 36, 54)

        verify_label = "Verify (V)"
        execute_label = "Execute (C)"
        solve_label = "Solve (End Run)"
        verify_btn_enabled = False
        execute_btn_enabled = False
        save_btn_enabled = False
        job_preview = None
        if not run_active:
            try:
                job_preview, _ = load_job_config(self._default_job_name, self._default_job_config_path)
            except Exception:
                job_preview = None
        has_verification_input = bool(job_preview and self._has_verification_input(job_preview))
        
        with self._verification_lock:
            if self._verification_busy:
                verify_label = "Planning..."
            else:
                execute_btn_enabled = self._verification_plan_is_fresh_locked()
            save_btn_enabled = self._latest_handeye_solution is not None

        verify_btn_enabled = (not run_active) and (not self._verification_busy) and has_verification_input

        self._draw_button(
            canvas,
            verify_rect,
            verify_label,
            enabled=verify_btn_enabled,
        )
        self._draw_button(
            canvas,
            execute_rect,
            execute_label,
            enabled=execute_btn_enabled,
        )
        self._draw_button(canvas, save_rect, "Save (A)", enabled=save_btn_enabled)
        self._draw_button(canvas, load_rect, "Load (L)", enabled=not run_active)

        # Movement status for manual capture
        can_add = False
        movement_status = "No Robot State"
        try:
            curr_ee = self._current_base_to_ee()
            if curr_ee is not None:
                significant, reason = self._is_new_pose_significant(curr_ee)
                can_add = significant
                movement_status = reason
        except Exception:
            pass

        self._draw_button(
            canvas, 
            add_sample_rect, 
            f"Add Sample: {movement_status}", 
            enabled=run_active and can_add
        )

        self._draw_button(
            canvas,
            solve_rect,
            solve_label,
            enabled=run_active and len(self._current_valid_samples) >= 3
        )

        self._draw_button(
            canvas,
            start_rect,
            "Start Run (S)",
            enabled=not run_active,
        )
        self._draw_button(canvas, restart_rect, "Restart Run (R)")
        with self._preview_lock:
            self._preview_button_rects = {
                "verify": verify_rect,
                "execute": execute_rect,
                "save": save_rect,
                "load": load_rect,
                "add_sample": add_sample_rect,
                "solve": solve_rect,
                "start": start_rect,
                "restart": restart_rect,
            }
        try:
            # Live detection in preview if idle
            if not run_active and self._latest_image is not None and not self._preview_disabled:
                # Load/Cache job config for live detection
                with self._live_detect_config_lock:
                    if (
                        self._live_detect_config is None
                        or self._live_detect_config[0] != self._default_job_name
                        or self._live_detect_config[1] != self._default_job_config_path
                    ):
                        try:
                            job_live, _ = load_job_config(
                                self._default_job_name, self._default_job_config_path
                            )
                            dict_live, board_live = create_charuco_board(job_live.board)
                            self._live_detect_config = (
                                self._default_job_name,
                                self._default_job_config_path,
                                job_live,
                                (dict_live, board_live),
                            )
                        except Exception:
                            job_live = None
                    else:
                        job_live = self._live_detect_config[2]
                        dict_live, board_live = self._live_detect_config[3]

                if job_live and job_live.capture_mode == "charuco":
                    with self._latest_lock:
                        img_entry = copy.deepcopy(self._latest_image)
                        info_entry = copy.deepcopy(self._latest_camera_info)

                    if img_entry and info_entry:
                        img_msg, _ = img_entry
                        info_msg, _ = info_entry
                        try:
                            cv_img = self._cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
                            intrinsics = intrinsics_from_camera_info(info_msg)
                            detection = detect_charuco_pose(
                                cv_img,
                                dictionary=dict_live,
                                board=board_live,
                                intrinsics=intrinsics,
                                min_corners=max(job_live.min_corners, 4),
                            )
                            frame = detection.preview_bgr
                            if detection.success:
                                status_lines.append(f"Live: detected {len(detection.charuco_ids)} corners")
                            else:
                                status_lines.append(f"Live: {detection.error_message}")
                        except Exception as e:
                            status_lines.append(f"Live CV Error: {str(e)}")

            cv2.imshow(self._preview_window_name, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("v"), ord("V")):
                self._handle_verify_key()
            if key in (ord("c"), ord("C")):
                self._handle_confirm_key()
            if key in (ord("a"), ord("A")):
                self._handle_save_archive_key()
            if key in (ord("l"), ord("L")):
                self._handle_load_archive_key()
            if key in (ord("s"), ord("S")):
                self._queue_start()
            if key in (ord("r"), ord("R")):
                self._queue_restart()
        except cv2.error as error:
            self.get_logger().warn(f"Disabling preview window because OpenCV GUI failed: {error}")
            self._preview_disabled = True

    def _preview_ee_status_lines(self) -> list[str]:
        with self._latest_lock:
            robot_state_entry = copy.deepcopy(self._latest_robot_state)
            joint_state_entry = copy.deepcopy(self._latest_joint_state)

        # Use robot_state if configured and available
        if self._preview_robot_pose_source == "robot_state" and robot_state_entry is not None:
            robot_state_msg, received_time = robot_state_entry
            transform = franka_matrix_to_transform(robot_state_msg.o_t_ee)
            xyz = transform[:3, 3]
            age_sec = time.monotonic() - received_time
            return [
                "EE position (robot)",
                f"x={xyz[0]:+.3f}  y={xyz[1]:+.3f}",
                f"z={xyz[2]:+.3f}  age={age_sec:.2f}s",
            ]

        now = time.monotonic()
        if (
            joint_state_entry is None
            or self._fk_client is None
            or not self._preview_fk_link_name
            or not self._preview_base_frame
        ):
            return list(self._preview_ee_cache_lines)

        if now - self._preview_ee_cache_time < 0.25:
            return list(self._preview_ee_cache_lines)

        joint_state_msg, received_time = joint_state_entry
        try:
            base_to_ee = compute_fk_transform(
                self._fk_client,
                joint_state_msg,
                fk_link_name=self._preview_fk_link_name,
                frame_id=self._preview_base_frame,
                timeout_sec=0.5,
            )
            lines = [
                "EE position (FK)",
                f"x={base_to_ee[0, 3]:+.3f}  y={base_to_ee[1, 3]:+.3f}",
                f"z={base_to_ee[2, 3]:+.3f}  age={now - received_time:.2f}s",
            ]
        except Exception:  # noqa: BLE001
            lines = ["EE xyz [m]: unavailable"]

        self._preview_ee_cache_lines = lines
        self._preview_ee_cache_time = now
        return list(lines)

    def _register_valid_sample(self, record: dict[str, Any]) -> None:
        self._current_valid_samples.append(copy.deepcopy(record))
        
        # Persist to disk immediately if we have a run directory
        with self._run_lock:
            active_run_paths = self._current_run_paths
        
        if active_run_paths:
            append_jsonl(active_run_paths.samples_jsonl, record)

        valid_count = len(self._current_valid_samples)
        if valid_count < 3:
            self._set_preview_calibration_lines(
                [
                    f"Calibration: {valid_count} valid sample(s)",
                    "Estimate: need at least 3 samples",
                ]
            )
            return

        try:
            solution = solve_handeye_from_samples(self._current_valid_samples)
            chosen_method = str(solution["chosen_method"])
            chosen = solution["methods"][chosen_method]
            ee_to_camera = np.asarray(chosen["gripper_to_camera"]["matrix"], dtype=np.float64)
            quat = chosen["gripper_to_camera"]["quaternion_xyzw"]
            metrics = chosen["metrics"]
            self._set_preview_calibration_lines(
                [
                    f"Calibration: {valid_count} valid sample(s)",
                    f"EE source: {'FrankaRobotState' if self._latest_robot_state is not None else 'FK'}",
                    f"Best method: {chosen_method}",
                    (
                        f"ee_T_camera xyz [m]: "
                        f"{ee_to_camera[0, 3]:+.3f}, {ee_to_camera[1, 3]:+.3f}, {ee_to_camera[2, 3]:+.3f}"
                    ),
                    (
                        f"quat xyzw: "
                        f"{quat[0]:+.3f}, {quat[1]:+.3f}, {quat[2]:+.3f}, {quat[3]:+.3f}"
                    ),
                    (
                        f"RMS: {metrics['translation_rms_m']*1000.0:.1f} mm / "
                        f"{metrics['rotation_rms_deg']:.2f} deg"
                    ),
                ]
            )
        except Exception as error:  # noqa: BLE001
            self._set_preview_calibration_lines(
                [
                    f"Calibration: {valid_count} valid sample(s)",
                    f"Estimate error: {error}",
                ]
            )

    def _is_new_pose_significant(self, current_base_to_ee: np.ndarray) -> tuple[bool, str]:
        if self._last_saved_base_to_ee is None:
            return True, "First Sample"

        residual = residual_between_transforms(self._last_saved_base_to_ee, current_base_to_ee)
        dist_cm = residual.translation_error_m * 100.0
        rot_deg = residual.rotation_error_deg

        if dist_cm >= self._manual_translation_threshold_m * 100.0:
            return True, f"Moved {dist_cm:.1f}cm"
        if rot_deg >= self._manual_rotation_threshold_deg:
            return True, f"Rotated {rot_deg:.1f}deg"

        return False, f"Too close ({dist_cm:.1f}cm, {rot_deg:.1f}deg)"

    def _handle_add_sample_manual(self) -> None:
        try:
            # Load default job to get board config if nothing else is active
            job, _ = load_job_config(self._default_job_name, self._default_job_config_path)
            
            # 1. Get snapshot
            snapshot = self._snapshot(job)
            
            # 2. Check board detection
            dict_charuco, board = create_charuco_board(job.board)
            detect_res = detect_charuco_pose(
                snapshot.image_bgr,
                dictionary=dict_charuco,
                board=board,
                intrinsics=snapshot.intrinsics,
                min_corners=job.min_corners,
            )
            if not detect_res.success:
                self._update_preview_status(["Capture Failed: board not detected", "Check lighting and board visibility."])
                return
                
            # 3. Check movement threshold
            current_base_to_ee = self._current_base_to_ee()
            if current_base_to_ee is None:
                self._update_preview_status(["Capture Failed: robot pose unavailable or stale", "Check if robot is faulted or broadcaster died."])
                return
                
            significant, reason = self._is_new_pose_significant(current_base_to_ee)
            if not significant:
                self._set_preview_verification_lines([f"Capture Rejected: {reason}", "Move the robot more!"])
                return

            # 4. Save sample
            # Ensure we are in an active session
            if not self._run_active:
                self._update_preview_status(["Please click 'Start Run' first", "Manual mode needs an active archive session."])
                return

            # Note: _register_valid_sample handles persistence to the session sample list
            record = {
                "base_to_ee": serialize_transform(current_base_to_ee),
                "target_to_camera": serialize_transform(detect_res.target_to_camera),
            }
            self._register_valid_sample(record)
            self._last_saved_base_to_ee = current_base_to_ee.copy()
            self._update_preview_status([f"Sample Added! ({reason})", f"Total: {len(self._current_valid_samples)}"])
            
        except Exception as e:
            self.get_logger().error(f"Manual capture failed: {str(e)}")
            self._update_preview_status([f"Capture Error: {str(e)}"])

    def _ensure_preview_window(self) -> bool:
        if self._preview_window_initialized:
            return True
        try:
            cv2.namedWindow(self._preview_window_name, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(self._preview_window_name, self._preview_mouse_callback)
            self._preview_window_initialized = True
            return True
        except cv2.error as error:
            self.get_logger().warn(f"Disabling preview window because OpenCV GUI failed: {error}")
            self._preview_disabled = True
            return False

    def _draw_button(
        self,
        canvas,
        rect: tuple[int, int, int, int],
        label: str,
        *,
        enabled: bool = True,
    ) -> None:
        x, y, w, h = rect
        fill_color = (70, 70, 70) if enabled else (45, 45, 45)
        border_color = (0, 255, 255) if enabled else (110, 110, 110)
        text_color = (255, 255, 255) if enabled else (170, 170, 170)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), fill_color, thickness=-1)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), border_color, thickness=2)
        text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        text_x = x + (w - text_size[0]) // 2
        text_y = y + (h + text_size[1]) // 2
        cv2.putText(
            canvas,
            label,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            text_color,
            2,
        )

    def _handle_verify_key(self) -> None:
        with self._verification_lock:
            if self._verification_busy or self._run_active:
                return
            self._verification_busy = True
            self._last_verification_plan = None
        self._set_preview_verification_lines(
            [
                "Verify: planning target...",
                "Execute: waiting for a fresh plan",
            ]
        )
        threading.Thread(target=self._calculate_verification_pose, daemon=True).start()

    def _handle_confirm_key(self) -> None:
        plan: VerificationPlan | None = None
        with self._verification_lock:
            if self._verification_busy or self._run_active:
                return
            if not self._verification_plan_is_fresh_locked():
                return
            plan = copy.deepcopy(self._last_verification_plan)
            if not plan:
                return
            self._verification_busy = True
            self._last_verification_plan = None
        self._set_preview_verification_lines(
            [
                "Verify: plan accepted",
                "Execute: sending motion command...",
            ]
        )

        threading.Thread(target=self._execute_verification_move, args=(plan,), daemon=True).start()

    def _calculate_verification_pose(self) -> None:
        try:
            self.get_logger().info("Starting verification pose calculation...")
            job, _ = load_job_config(self._default_job_name, self._default_job_config_path)
            self._prepare_verification_stack(job)
            snapshot = self._wait_for_snapshot(job, timeout_sec=5.0)

            if self._use_fake_hardware or job.capture_mode == "mock":
                target = self._build_fake_verification_target(job, snapshot)
            else:
                target = self._build_real_verification_target(job, snapshot)

            desired_base_to_ee = np.asarray(target["base_to_ee"], dtype=np.float64)
            joint_state = self._solve_verification_ik(snapshot, job, desired_base_to_ee)
            point_name = f"{job.job_name}_verify_approach"
            point_path = self._write_verification_point(point_name, job, joint_state)

            plan = VerificationPlan(
                point_name=point_name,
                point_path=point_path,
                created_mono=time.monotonic(),
                fake_hardware=bool(target["fake_hardware"]),
                selected_charuco_id=target["selected_charuco_id"],
                target_translation_xyz_m=(
                    float(desired_base_to_ee[0, 3]),
                    float(desired_base_to_ee[1, 3]),
                    float(desired_base_to_ee[2, 3]),
                ),
            )
            with self._verification_lock:
                self._last_verification_plan = plan

            self.get_logger().info("Verification plan ready. Sending plan-only preview.")
            self._set_preview_verification_lines(
                [
                    f"Verify: ready using {target['solution_source']}",
                    f"Target: {target['label']}",
                    "Execute: press C or click Execute",
                ]
            )
            self._update_preview_status(
                [
                    "Verification target planned.",
                    "A plan-only preview is being sent to the executor.",
                ]
            )
            threading.Thread(target=self._execute_verification_move, args=(plan, True), daemon=True).start()
        except Exception as error:
            self.get_logger().error(f"Verification calculation failed: {str(error)}")
            self._set_preview_verification_lines(
                [
                    f"Verify Error: {str(error)}",
                    "Execute: unavailable",
                ]
            )
            self._update_preview_status([f"Verify Error: {str(error)}"])
        finally:
            with self._verification_lock:
                self._verification_busy = False

    def _execute_verification_move(self, plan: VerificationPlan, plan_only: bool = False) -> None:
        try:
            self.get_logger().info(f"Sending verification goal (plan_only={plan_only})...")
            job, _ = load_job_config(self._default_job_name, self._default_job_config_path)
            self._prepare_verification_stack(job)
            if plan_only:
                self._update_preview_status(
                    ["Sending plan-only verification preview...", "Check the ghost in RViz."]
                )
            else:
                self._update_preview_status(
                    ["Moving to verification target...", "Hand on E-Stop!"]
                )
            wrapped_result = self._executor_bridge.run_point(
                plan.point_name,
                plan_only=plan_only,
                velocity_scale=self._verification_velocity_scale,
                acceleration_scale=self._verification_acceleration_scale,
                planning_time=self._verification_planning_time,
                goal_tolerance=self._verification_goal_tolerance,
                result_timeout_sec=60.0,
            )
            point_result = wrapped_result.result
            if (
                wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
                or point_result is None
                or not point_result.success
            ):
                error_message = "Executor rejected the verification point."
                if point_result is not None and point_result.error_message:
                    error_message = point_result.error_message
                raise RuntimeError(error_message)

            if plan_only:
                self._set_preview_verification_lines(
                    [
                        "Verify: ghost plan sent successfully",
                        "Execute: press C or click Execute",
                    ]
                )
            else:
                self._set_preview_verification_lines(
                    [
                        "Verify: plan executed",
                        (
                            "Target xyz [m]: "
                            f"{plan.target_translation_xyz_m[0]:+.3f}, "
                            f"{plan.target_translation_xyz_m[1]:+.3f}, "
                            f"{plan.target_translation_xyz_m[2]:+.3f}"
                        ),
                    ]
                )
        except Exception as error:
            self.get_logger().error(f"Failed to launch verification move: {str(error)}")
            self._set_preview_verification_lines(
                [
                    f"Execute Error: {str(error)}",
                    "Verify again to generate a fresh plan",
                ]
            )
        finally:
            if not plan_only:
                with self._verification_lock:
                    self._verification_busy = False

    def _preview_mouse_callback(self, event, x: int, y: int, _flags, _param) -> None:
        if event != cv2.EVENT_LBUTTONUP:
            return
        with self._preview_lock:
            button_rects = dict(self._preview_button_rects)

        verify_rect = button_rects.get("verify")
        execute_rect = button_rects.get("execute")
        save_rect = button_rects.get("save")
        load_rect = button_rects.get("load")
        add_sample_rect = button_rects.get("add_sample")
        start_rect = button_rects.get("start")
        restart_rect = button_rects.get("restart")
        solve_rect = button_rects.get("solve")

        if verify_rect is not None and self._point_in_rect(x, y, verify_rect):
            self._handle_verify_key()
            return
        if execute_rect is not None and self._point_in_rect(x, y, execute_rect):
            self._handle_confirm_key()
            return
        if save_rect is not None and self._point_in_rect(x, y, save_rect):
            self._handle_save_archive_key()
            return
        if load_rect is not None and self._point_in_rect(x, y, load_rect):
            self._handle_load_archive_key()
            return
        if add_sample_rect is not None and self._point_in_rect(x, y, add_sample_rect):
            self._handle_add_sample_manual()
            return
        if start_rect is not None and self._point_in_rect(x, y, start_rect):
            self._queue_start()
            return
        if solve_rect is not None and self._point_in_rect(x, y, solve_rect):
            self._manual_solve_requested = True
            return
        
        if restart_rect is not None and self._point_in_rect(x, y, restart_rect):
            self._queue_restart()
            return
        if solve_rect is not None and self._point_in_rect(x, y, solve_rect):
            with self._run_lock:
                self._run_active = False
            return

    def _point_in_rect(self, x: int, y: int, rect: tuple[int, int, int, int]) -> bool:
        rect_x, rect_y, rect_w, rect_h = rect
        return rect_x <= x <= rect_x + rect_w and rect_y <= y <= rect_y + rect_h

    def _queue_start(self) -> None:
        run_request: tuple[str, str, bool] | None = None
        with self._run_lock:
            if self._run_active:
                self.get_logger().warn("Ignoring start request because a run is already active.")
                return
            run_request = self._last_run_request
        self.get_logger().info("Queued start of the current calibration/debug run.")
        self._update_preview_status(["Start requested...", *self._preview_status_lines[:5]])
        if run_request is not None:
            threading.Thread(
                target=self._run_background_job,
                args=(*run_request, 0.0),
                daemon=True,
            ).start()

    def _queue_restart(self) -> None:
        pending_run: tuple[str, str, bool] | None = None
        with self._run_lock:
            self._pending_background_run = self._last_run_request
            run_active = self._run_active
            if not run_active:
                pending_run = self._pending_background_run
                self._pending_background_run = None
        self.get_logger().info("Queued restart of the current calibration/debug run.")
        self._update_preview_status(["Restart requested...", *self._preview_status_lines[:5]])
        if not run_active:
            if pending_run is not None:
                threading.Thread(
                    target=self._run_background_job,
                    args=(*pending_run, 0.0),
                    daemon=True,
                ).start()
            return
        if run_active:
            try:
                self._executor_bridge.stop_execution(timeout_sec=2.0)
            except Exception as error:  # noqa: BLE001
                self.get_logger().warn(f"Could not stop the active run before restart: {error}")
