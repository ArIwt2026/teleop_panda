from __future__ import annotations

import copy
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from franka_msgs.action import Grasp, Homing, Move
from franka_msgs.msg import FrankaRobotState
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import yaml

from .storage import resolve_storage_root

ROBOT_MODE_NAMES = {
    FrankaRobotState.ROBOT_MODE_OTHER: "OTHER",
    FrankaRobotState.ROBOT_MODE_IDLE: "IDLE",
    FrankaRobotState.ROBOT_MODE_MOVE: "MOVE",
    FrankaRobotState.ROBOT_MODE_GUIDING: "GUIDING",
    FrankaRobotState.ROBOT_MODE_REFLEX: "REFLEX",
    FrankaRobotState.ROBOT_MODE_USER_STOPPED: "USER_STOPPED",
    FrankaRobotState.ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY: "AUTOMATIC_ERROR_RECOVERY",
}
GUIDING_MODES = {"GUIDING", "USER_STOPPED"}

DEFAULT_GRIPPER_SPEED = 0.1
DEFAULT_GRIPPER_FORCE = 50.0
DEFAULT_GRIPPER_MAX_WIDTH = 0.08
DEFAULT_TAG_SIZE = 0.06
DEFAULT_BASE_FRAME = "panda_link0"
DEFAULT_EE_FRAME = "panda_hand_tcp"
DEFAULT_MARKER_LOCK_SAMPLES = 100
DEFAULT_MARKER_LOCK_MIN_SAMPLES = 60
DEFAULT_MARKER_TRANSLATION_INLIER_FLOOR_M = 0.0015
DEFAULT_MARKER_YAW_INLIER_FLOOR_RAD = np.deg2rad(0.5)


@dataclass(frozen=True)
class MarkerObservation:
    tag_id: int
    target_to_camera: np.ndarray
    rvec: np.ndarray
    tvec: np.ndarray

@dataclass
class Snapshot:
    payload: dict[str, Any]
    robot_mode: str | None = None

class LiveStateListener(Node):
    def __init__(
        self,
        joint_states_topic: str = "/franka/joint_states",
        robot_state_topic: str | None = None,
        gripper_joint_states_topic: str | None = None,
        gripper_grasp_action: str = "/panda_gripper/grasp",
        gripper_move_action: str = "/panda_gripper/move",
        gripper_homing_action: str = "/panda_gripper/homing",
        image_topic: str = "/camera_front/camera_front/color/image_raw",
        camera_info_topic: str = "/camera_front/camera_front/color/camera_info",
        base_frame: str = DEFAULT_BASE_FRAME,
        ee_frame: str = DEFAULT_EE_FRAME,
        calibration_result_path: str | None = None,
    ) -> None:
        super().__init__("franka_live_listener")
        self._lock = threading.Lock()
        self._joint_states_topic = joint_states_topic
        self._robot_state_topic = robot_state_topic
        self._gripper_joint_states_topic = gripper_joint_states_topic
        self._gripper_grasp_action = gripper_grasp_action
        self._gripper_move_action = gripper_move_action
        self._gripper_homing_action = gripper_homing_action
        self._base_frame = base_frame
        self._ee_frame = ee_frame
        self._calibration_result_path = self._resolve_calibration_result_path(calibration_result_path)
        self._camera_to_gripper = self._load_camera_to_gripper(self._calibration_result_path)

        self._latest_joint_state: JointState | None = None
        self._latest_joint_received_monotonic: float = 0.0
        self._latest_robot_state: FrankaRobotState | None = None
        self._latest_robot_received_monotonic: float = 0.0
        self._latest_gripper_state: JointState | None = None
        self._latest_gripper_received_monotonic: float = 0.0

        self.create_subscription(JointState, self._joint_states_topic, self._joint_state_callback, 10)
        if self._robot_state_topic:
            self.create_subscription(
                FrankaRobotState, self._robot_state_topic, self._robot_state_callback, 10
            )
        if self._gripper_joint_states_topic:
            self.create_subscription(
                JointState, self._gripper_joint_states_topic, self._gripper_state_callback, 10
            )

        self._gripper_grasp_client = ActionClient(self, Grasp, self._gripper_grasp_action)
        self._gripper_move_client = ActionClient(self, Move, self._gripper_move_action)
        self._gripper_homing_client = ActionClient(self, Homing, self._gripper_homing_action)

        # Perception setup
        self.bridge = CvBridge()
        self._latest_image = None
        self._latest_image_header = None
        self.cam_matrix = None
        self.dist_coeffs = None
        self.tag_size = DEFAULT_TAG_SIZE
        
        self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._info_callback, 10)
        
        # AprilTag Setup
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.parameters = self._create_detector_parameters()
        self._aruco_detector = (
            aruco.ArucoDetector(self.dictionary, self.parameters)
            if hasattr(aruco, "ArucoDetector")
            else None
        )
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _resolve_calibration_result_path(self, calibration_result_path: str | None) -> Path | None:
        if calibration_result_path:
            candidate = Path(calibration_result_path).expanduser().resolve()
            return candidate if candidate.exists() else None

        candidate = resolve_storage_root(None) / "latest_calibration.yaml"
        if candidate.exists():
            return candidate
        return None

    def _load_camera_to_gripper(self, calibration_result_path: Path | None) -> np.ndarray | None:
        if calibration_result_path is None:
            self.get_logger().warn(
                "No hand-eye calibration file found. Relative AprilTag features will stay unavailable."
            )
            return None

        try:
            with calibration_result_path.open("r", encoding="utf-8") as stream:
                payload = yaml.safe_load(stream) or {}
            matrix = np.asarray(payload["camera_to_gripper"]["matrix"], dtype=np.float64)
            if matrix.shape != (4, 4):
                raise ValueError(f"Expected a 4x4 camera_to_gripper matrix, got shape {matrix.shape}.")
            self.get_logger().info(f"Loaded hand-eye calibration from {calibration_result_path}")
            return matrix
        except Exception as error:
            self.get_logger().error(
                f"Failed to load hand-eye calibration from {calibration_result_path}: {error}"
            )
            return None

    def _require_camera_to_gripper(self) -> np.ndarray:
        if self._camera_to_gripper is None:
            raise RuntimeError(
                "Hand-eye calibration is not loaded. Expected camera_to_gripper in teach_data/latest_calibration.yaml."
            )
        return self._camera_to_gripper

    def _matrix_to_pose_stamped(self, transform: np.ndarray, frame_id: str) -> PoseStamped:
        from scipy.spatial.transform import Rotation as R

        quat = R.from_matrix(np.asarray(transform[:3, :3], dtype=np.float64)).as_quat()
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(transform[0, 3])
        pose.pose.position.y = float(transform[1, 3])
        pose.pose.position.z = float(transform[2, 3])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        return pose

    def _build_transform(self, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
        transform = np.identity(4, dtype=np.float64)
        transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
        transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
        return transform

    def _create_detector_parameters(self):
        if hasattr(aruco, "DetectorParameters"):
            params = aruco.DetectorParameters()
        else:
            params = aruco.DetectorParameters_create()

        if hasattr(aruco, "CORNER_REFINE_SUBPIX"):
            params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        if hasattr(params, "cornerRefinementWinSize"):
            params.cornerRefinementWinSize = 10
        if hasattr(params, "cornerRefinementMaxIterations"):
            params.cornerRefinementMaxIterations = 50
        if hasattr(params, "cornerRefinementMinAccuracy"):
            params.cornerRefinementMinAccuracy = 0.01

        if hasattr(params, "aprilTagQuadDecimate"):
            params.aprilTagQuadDecimate = 1.0
        if hasattr(params, "aprilTagQuadSigma"):
            params.aprilTagQuadSigma = 0.0
        if hasattr(params, "aprilTagDeglitch"):
            params.aprilTagDeglitch = 0

        return params

    def _detect_markers(self, gray: np.ndarray):
        if self._aruco_detector is not None:
            return self._aruco_detector.detectMarkers(gray)
        return aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

    def _refine_marker_corners(self, gray: np.ndarray, marker_corners) -> np.ndarray:
        refined = np.asarray(marker_corners, dtype=np.float32).reshape(-1, 1, 2).copy()
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            40,
            1e-3,
        )
        cv2.cornerSubPix(gray, refined, (5, 5), (-1, -1), criteria)
        return refined.reshape(1, 4, 2)

    def _estimate_marker_pose(self, gray: np.ndarray, marker_corners, mtx: np.ndarray, dist: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        half_size = self.tag_size * 0.5
        object_points = np.array(
            [
                [-half_size, half_size, 0.0],
                [half_size, half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ],
            dtype=np.float64,
        )
        image_points = self._refine_marker_corners(gray, marker_corners).reshape(4, 2).astype(np.float64)

        pose_solutions: list[tuple[float, np.ndarray, np.ndarray]] = []
        solve_flags = []
        if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
            solve_flags.append(cv2.SOLVEPNP_IPPE_SQUARE)
        if hasattr(cv2, "SOLVEPNP_ITERATIVE"):
            solve_flags.append(cv2.SOLVEPNP_ITERATIVE)

        for flag in solve_flags:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                mtx,
                dist,
                flags=flag,
            )
            if not success:
                continue

            projected_points, _ = cv2.projectPoints(object_points, rvec, tvec, mtx, dist)
            reprojection_error = float(
                np.mean(
                    np.linalg.norm(
                        projected_points.reshape(4, 2) - image_points,
                        axis=1,
                    )
                )
            )
            pose_solutions.append(
                (
                    reprojection_error,
                    np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                    np.asarray(tvec, dtype=np.float64).reshape(3, 1),
                )
            )

        if not pose_solutions:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                [marker_corners],
                self.tag_size,
                mtx,
                dist,
            )
            rvec = np.asarray(rvecs[0], dtype=np.float64).reshape(3, 1)
            tvec = np.asarray(tvecs[0], dtype=np.float64).reshape(3, 1)
            return self._build_transform(cv2.Rodrigues(rvec)[0], tvec.reshape(3)), rvec, tvec

        _, best_rvec, best_tvec = min(pose_solutions, key=lambda item: item[0])
        rotation, _ = cv2.Rodrigues(best_rvec)
        return self._build_transform(rotation, best_tvec.reshape(3)), best_rvec, best_tvec

    def _wrap_to_pi(self, angle: np.ndarray | float) -> np.ndarray | float:
        return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi

    def _mean_yaw(self, yaws: np.ndarray) -> float:
        return float(np.arctan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws))))

    def _fuse_marker_transforms(self, transforms: list[np.ndarray]) -> np.ndarray:
        if not transforms:
            raise RuntimeError("No AprilTag detections were collected for fusion.")

        stacked = np.stack([np.asarray(transform, dtype=np.float64) for transform in transforms], axis=0)
        translations = stacked[:, :3, 3]
        yaws = np.arctan2(stacked[:, 1, 0], stacked[:, 0, 0])
        median_translation = np.median(translations, axis=0)
        mean_yaw = self._mean_yaw(yaws)
        translation_errors = np.linalg.norm(translations - median_translation, axis=1)
        yaw_errors = np.abs(self._wrap_to_pi(yaws - mean_yaw))
        median_error = float(np.median(translation_errors))
        median_yaw_error = float(np.median(yaw_errors))
        translation_threshold = (
            DEFAULT_MARKER_TRANSLATION_INLIER_FLOOR_M
            if median_error < 1e-9
            else max(DEFAULT_MARKER_TRANSLATION_INLIER_FLOOR_M, 3.0 * median_error)
        )
        yaw_threshold = (
            DEFAULT_MARKER_YAW_INLIER_FLOOR_RAD
            if median_yaw_error < 1e-9
            else max(DEFAULT_MARKER_YAW_INLIER_FLOOR_RAD, 3.0 * median_yaw_error)
        )
        inlier_mask = np.logical_and(
            translation_errors <= translation_threshold,
            yaw_errors <= yaw_threshold,
        )
        inlier_transforms = stacked[inlier_mask]
        if len(inlier_transforms) < max(3, min(DEFAULT_MARKER_LOCK_MIN_SAMPLES, len(stacked))):
            inlier_transforms = stacked

        mean_translation = np.median(inlier_transforms[:, :3, 3], axis=0)
        rotation_sum = np.sum(inlier_transforms[:, :3, :3], axis=0)
        u, _, vt = np.linalg.svd(rotation_sum)
        mean_rotation = u @ vt
        if np.linalg.det(mean_rotation) < 0.0:
            u[:, -1] *= -1.0
            mean_rotation = u @ vt

        return self._build_transform(mean_rotation, mean_translation)

    def _lookup_base_to_ee_matrix(self) -> np.ndarray:
        with self._lock:
            robot_state = copy.deepcopy(self._latest_robot_state)
            robot_received = self._latest_robot_received_monotonic

        if robot_state is not None:
            robot_age = time.monotonic() - robot_received
            if robot_age <= 1.0:
                return np.array(robot_state.o_t_ee, dtype=np.float64).reshape((4, 4), order="F")

        try:
            transform = self.tf_buffer.lookup_transform(self._base_frame, self._ee_frame, rclpy.time.Time())
        except Exception as error:
            if robot_state is not None:
                raise RuntimeError(
                    f"Robot state for '{self._robot_state_topic}' is stale and TF lookup "
                    f"'{self._base_frame}' -> '{self._ee_frame}' also failed: {error}"
                ) from error
            raise RuntimeError(
                f"Unable to determine current EE pose. No fresh robot state and TF lookup "
                f"'{self._base_frame}' -> '{self._ee_frame}' failed: {error}"
            ) from error

        from scipy.spatial.transform import Rotation as R

        matrix = np.identity(4, dtype=np.float64)
        matrix[0, 3] = transform.transform.translation.x
        matrix[1, 3] = transform.transform.translation.y
        matrix[2, 3] = transform.transform.translation.z
        q = transform.transform.rotation
        matrix[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return matrix

    def _detect_target_to_camera_matrix(
        self,
        timeout_sec: float = 30.0,
        show_window: bool = True,
        auto_confirm: bool = False,
    ) -> np.ndarray:
        target_samples = 12 if auto_confirm else DEFAULT_MARKER_LOCK_SAMPLES
        min_samples = 3 if auto_confirm else DEFAULT_MARKER_LOCK_MIN_SAMPLES
        deadline = time.monotonic() + timeout_sec
        window_name = "AprilTag Scan - SPACE to confirm" if show_window else None
        confirmed_transform = None
        collecting = False
        locked_tag_id: int | None = None
        collected_transforms: list[np.ndarray] = []
        last_collected_stamp: tuple[int, int] | None = None

        try:
            while rclpy.ok() and time.monotonic() < deadline:
                with self._lock:
                    img = copy.deepcopy(self._latest_image)
                    img_header = copy.deepcopy(self._latest_image_header)
                    mtx = None if self.cam_matrix is None else np.array(self.cam_matrix, copy=True)
                    dist = None if self.dist_coeffs is None else np.array(self.dist_coeffs, copy=True)

                if img is None or mtx is None:
                    time.sleep(0.01)
                    continue

                frame_stamp = None
                if img_header is not None:
                    frame_stamp = (img_header.stamp.sec, img_header.stamp.nanosec)

                gray_clean = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                
                # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
                # This helps "pop" the edges of the AprilTag if the feed is washed out or dark.
                clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                gray_detect = clahe.apply(gray_clean)
                
                corners, ids, _ = self._detect_markers(gray_detect)

                current_observation = None
                current_tag_id = None
                if ids is not None and len(ids) > 0:
                    selected_index = 0
                    if collecting and locked_tag_id is not None:
                        matches = np.where(ids.flatten() == locked_tag_id)[0]
                        selected_index = int(matches[0]) if len(matches) > 0 else -1

                    if selected_index >= 0:
                        current_tag_id = int(ids[selected_index][0])
                        target_to_camera, rvec, tvec = self._estimate_marker_pose(
                            gray_clean,
                            corners[selected_index],
                            mtx,
                            dist,
                        )
                        current_observation = MarkerObservation(
                            tag_id=current_tag_id,
                            target_to_camera=target_to_camera,
                            rvec=rvec,
                            tvec=tvec,
                        )

                if show_window:
                    display_img = img.copy()
                    tag_found = current_observation is not None
                    if tag_found:
                        aruco.drawDetectedMarkers(display_img, corners, ids)
                        cv2.drawFrameAxes(
                            display_img,
                            mtx,
                            dist,
                            current_observation.rvec,
                            current_observation.tvec,
                            self.tag_size * 0.5,
                        )

                    if collecting:
                        status_text = (
                            f"LOCKING TAG {locked_tag_id}: "
                            f"{len(collected_transforms)}/{target_samples}"
                        )
                    else:
                        status_text = (
                            "TAG FOUND - Press SPACE to Confirm"
                            if tag_found
                            else "SEARCHING - Position the robot..."
                        )
                    cv2.putText(
                        display_img,
                        status_text,
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0) if tag_found else (0, 0, 255),
                        2,
                    )
                    cv2.imshow(window_name, display_img)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord(" "):
                        if current_observation is not None and not collecting:
                            collecting = True
                            locked_tag_id = current_tag_id
                            collected_transforms = [current_observation.target_to_camera]
                            last_collected_stamp = frame_stamp
                    elif key == 27 or key == ord("q"):
                        raise InterruptedError("Cancelled by user.")
                elif auto_confirm and current_observation is not None and not collecting:
                    collecting = True
                    locked_tag_id = current_tag_id
                    collected_transforms = [current_observation.target_to_camera]
                    last_collected_stamp = frame_stamp

                if collecting and current_observation is not None and current_tag_id == locked_tag_id:
                    if frame_stamp is None or frame_stamp != last_collected_stamp:
                        collected_transforms.append(current_observation.target_to_camera)
                        last_collected_stamp = frame_stamp
                    if len(collected_transforms) >= target_samples:
                        confirmed_transform = self._fuse_marker_transforms(collected_transforms)
                        break

                time.sleep(0.01)

            if confirmed_transform is None and len(collected_transforms) >= min_samples:
                confirmed_transform = self._fuse_marker_transforms(collected_transforms)
            if confirmed_transform is None:
                raise TimeoutError("No AprilTag detected.")
            return confirmed_transform
        finally:
            if show_window:
                try:
                    cv2.destroyWindow(window_name)
                except Exception:
                    pass

    def _joint_state_callback(self, msg: JointState) -> None:
        with self._lock:
            self._latest_joint_state = msg
            self._latest_joint_received_monotonic = time.monotonic()

    def _robot_state_callback(self, msg: FrankaRobotState) -> None:
        with self._lock:
            self._latest_robot_state = msg
            self._latest_robot_received_monotonic = time.monotonic()

    def _gripper_state_callback(self, msg: JointState) -> None:
        with self._lock:
            self._latest_gripper_state = msg
            self._latest_gripper_received_monotonic = time.monotonic()

    def _info_callback(self, msg):
        with self._lock:
            self.cam_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def _image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self._latest_image = cv_image
                self._latest_image_header = msg.header
                # print("DEBUG: Received image from " + msg.header.frame_id)
        except Exception as e:
            print(f"DEBUG: Image callback error: {e}")

    def wait_for_joint_state(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest_joint_state is not None:
                    return
            time.sleep(0.05)
        raise TimeoutError(f"No JointState message arrived on within {timeout_sec:.1f}s.")

    def capture_snapshot(self, timeout_sec: float, max_age_sec: float, require_guiding: bool) -> Snapshot:
        self.wait_for_joint_state(timeout_sec)
        with self._lock:
            joint_state = copy.deepcopy(self._latest_joint_state)
            joint_received = self._latest_joint_received_monotonic
            robot_state = copy.deepcopy(self._latest_robot_state)
            robot_received = self._latest_robot_received_monotonic

        if joint_state is None:
            raise RuntimeError("JointState capture failed.")

        age = time.monotonic() - joint_received
        if age > max_age_sec:
            raise RuntimeError(f"Latest JointState is stale ({age:.2f}s old).")

        robot_mode_name = None
        if robot_state is not None:
            robot_mode_name = ROBOT_MODE_NAMES.get(robot_state.robot_mode, f"UNKNOWN_{robot_state.robot_mode}")

        if require_guiding:
            if self._robot_state_topic is None:
                raise RuntimeError("Guiding mode check requested, but no topic was configured.")
            if robot_state is None:
                raise RuntimeError(f"No FrankaRobotState message arrived on '{self._robot_state_topic}'.")
            robot_age = time.monotonic() - robot_received
            if robot_age > max_age_sec:
                raise RuntimeError(f"Latest FrankaRobotState is stale ({robot_age:.2f}s old).")
            if robot_mode_name not in GUIDING_MODES:
                raise RuntimeError(f"Robot mode is '{robot_mode_name}', not GUIDING/USER_STOPPED.")

        stamp_sec = joint_state.header.stamp.sec + joint_state.header.stamp.nanosec / 1_000_000_000.0
        payload = {
            "joint_states_topic": self._joint_states_topic,
            "joint_state_stamp": stamp_sec,
            "joint_names": list(joint_state.name),
            "positions": [float(value) for value in joint_state.position],
            "velocities": [float(value) for value in joint_state.velocity],
            "efforts": [float(value) for value in joint_state.effort],
        }
        if self._robot_state_topic:
            payload["robot_state_topic"] = self._robot_state_topic
        if robot_mode_name is not None:
            payload["robot_mode"] = robot_mode_name
        return Snapshot(payload=payload, robot_mode=robot_mode_name)

    def capture_gripper_width(self, timeout_sec: float = 5.0, max_age_sec: float = 1.0) -> float:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest_gripper_state is not None:
                    break
            time.sleep(0.05)
        else:
            raise TimeoutError(f"No gripper JointState arrived within {timeout_sec:.1f}s.")
        with self._lock:
            gripper_state = copy.deepcopy(self._latest_gripper_state)
            age = time.monotonic() - self._latest_gripper_received_monotonic
        if age > max_age_sec:
            raise RuntimeError(f"Gripper state is stale ({age:.2f}s old).")
        if gripper_state.position:
            return float(gripper_state.position[0]) * 2.0
        raise RuntimeError("Gripper JointState has no position data.")

    def detect_marker_pose_base(self, timeout_sec: float = 30.0, show_window: bool = True, auto_confirm: bool = False) -> PoseStamped:
        target_to_camera = self._detect_target_to_camera_matrix(
            timeout_sec=timeout_sec,
            show_window=show_window,
            auto_confirm=auto_confirm,
        )
        base_to_ee = self._lookup_base_to_ee_matrix()
        base_to_marker = base_to_ee @ self._require_camera_to_gripper() @ target_to_camera
        return self._matrix_to_pose_stamped(base_to_marker, self._base_frame)

    def get_ee_pose_base(self, use_direct_state: bool = True) -> PoseStamped:
        del use_direct_state
        return self._matrix_to_pose_stamped(self._lookup_base_to_ee_matrix(), self._base_frame)

    def command_gripper_open(self, speed: float = DEFAULT_GRIPPER_SPEED, timeout_sec: float = 15.0) -> float:
        if not self._gripper_move_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Server not available at '{self._gripper_move_action}'")
        goal = Move.Goal()
        goal.width = DEFAULT_GRIPPER_MAX_WIDTH
        goal.speed = speed
        future = self._gripper_move_client.send_goal_async(goal)
        start_time = time.monotonic()
        while not future.done() and (time.monotonic() - start_time < 5.0):
            time.sleep(0.1)
        if not future.done():
            raise RuntimeError("Timed out sending goal.")
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            raise RuntimeError("Goal rejected.")
        result_future = goal_handle.get_result_async()
        start_time = time.monotonic()
        while not result_future.done() and (time.monotonic() - start_time < timeout_sec):
            time.sleep(0.1)
        if not result_future.done():
            raise RuntimeError("Timed out waiting for result.")
        return self.capture_gripper_width(timeout_sec=3.0)

    def command_gripper_close(self, speed: float = DEFAULT_GRIPPER_SPEED, force: float = DEFAULT_GRIPPER_FORCE, timeout_sec: float = 15.0) -> float:
        if not self._gripper_grasp_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Server not available at '{self._gripper_grasp_action}'")
        goal = Grasp.Goal()
        goal.width = 0.0
        goal.speed = speed
        goal.force = force
        goal.epsilon.inner = 0.04
        goal.epsilon.outer = 0.04
        future = self._gripper_grasp_client.send_goal_async(goal)
        start_time = time.monotonic()
        while not future.done() and (time.monotonic() - start_time < 5.0):
            time.sleep(0.1)
        if not future.done():
            raise RuntimeError("Timed out sending goal.")
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            raise RuntimeError("Goal rejected.")
        result_future = goal_handle.get_result_async()
        start_time = time.monotonic()
        while not result_future.done() and (time.monotonic() - start_time < timeout_sec):
            time.sleep(0.1)
        if not result_future.done():
            raise RuntimeError("Timed out waiting for result.")
        try:
            return self.capture_gripper_width(timeout_sec=3.0)
        except Exception:
            return 0.0

    def command_gripper_home(self, timeout_sec: float = 30.0) -> bool:
        if not self._gripper_homing_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Server not available at '{self._gripper_homing_action}'")
        future = self._gripper_homing_client.send_goal_async(Homing.Goal())
        start_time = time.monotonic()
        while not future.done() and (time.monotonic() - start_time < 5.0):
            time.sleep(0.1)
        if not future.done():
            raise RuntimeError("Timed out sending goal.")
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            raise RuntimeError("Goal rejected.")
        result_future = goal_handle.get_result_async()
        start_time = time.monotonic()
        while not result_future.done() and (time.monotonic() - start_time < timeout_sec):
            time.sleep(0.1)
        if not result_future.done():
            raise RuntimeError("Timed out waiting for result.")
        return result_future.result().result.success

class Spinner:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=2.0)
