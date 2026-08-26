from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .job_config import BoardConfig, CameraIntrinsics
from .transforms import serialize_transform, transform_from_rvec_tvec


@dataclass(frozen=True)
class CharucoDetection:
    success: bool
    preview_bgr: np.ndarray
    marker_ids: np.ndarray | None
    charuco_corners: np.ndarray | None
    charuco_ids: np.ndarray | None
    target_to_camera: np.ndarray | None
    rvec: np.ndarray | None
    tvec: np.ndarray | None
    reprojection_error_px: float | None
    error_message: str = ""


def get_dictionary(dictionary_name: str):
    if not hasattr(cv2.aruco, dictionary_name):
        supported = [name for name in dir(cv2.aruco) if name.startswith("DICT_")]
        raise ValueError(
            f"Unknown ArUco dictionary '{dictionary_name}'. Example supported values: {supported[:8]}"
        )
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.Dictionary_get(dictionary_id)


def create_charuco_board(board_config: BoardConfig):
    dictionary = get_dictionary(board_config.dictionary_name)
    if hasattr(cv2.aruco, "CharucoBoard"):
        board = cv2.aruco.CharucoBoard(
            (board_config.squares_x, board_config.squares_y),
            board_config.square_length_m,
            board_config.marker_length_m,
            dictionary,
        )
    else:
        board = cv2.aruco.CharucoBoard_create(
            board_config.squares_x,
            board_config.squares_y,
            board_config.square_length_m,
            board_config.marker_length_m,
            dictionary,
        )
    return dictionary, board


def _detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def detect_charuco_pose(
    frame_bgr: np.ndarray,
    dictionary,
    board,
    intrinsics: CameraIntrinsics,
    min_corners: int,
) -> CharucoDetection:
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    params = _detector_parameters()
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    preview = frame_bgr.copy()
    charuco_corners = None
    charuco_ids = None

    if marker_ids is not None and len(marker_ids) > 0:
        cv2.aruco.drawDetectedMarkers(preview, marker_corners, marker_ids)
        valid, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners,
            marker_ids,
            gray,
            board,
        )
        if valid is None or int(valid) < min_corners:
            return CharucoDetection(
                success=False,
                preview_bgr=preview,
                marker_ids=marker_ids,
                charuco_corners=charuco_corners,
                charuco_ids=charuco_ids,
                target_to_camera=None,
                rvec=None,
                tvec=None,
                reprojection_error_px=None,
                error_message=f"Detected fewer than {min_corners} ChArUco corners.",
            )

        if charuco_corners is not None and len(charuco_corners) > 0:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            cv2.cornerSubPix(gray, charuco_corners, (5, 5), (-1, -1), criteria)
    else:
        return CharucoDetection(
            success=False,
            preview_bgr=preview,
            marker_ids=marker_ids,
            charuco_corners=None,
            charuco_ids=None,
            target_to_camera=None,
            rvec=None,
            tvec=None,
            reprojection_error_px=None,
            error_message="No ArUco markers detected.",
        )

    cv2.aruco.drawDetectedCornersCharuco(preview, charuco_corners, charuco_ids)

    ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    image_points = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
    object_points = np.asarray(board.chessboardCorners, dtype=np.float32)[ids]

    if len(object_points) < 4:
        return CharucoDetection(
            success=False,
            preview_bgr=preview,
            marker_ids=marker_ids,
            charuco_corners=charuco_corners,
            charuco_ids=charuco_ids,
            target_to_camera=None,
            rvec=None,
            tvec=None,
            reprojection_error_px=None,
            error_message="Need at least 4 ChArUco corners for PnP.",
        )

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        intrinsics.camera_matrix,
        intrinsics.dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        return CharucoDetection(
            success=False,
            preview_bgr=preview,
            marker_ids=marker_ids,
            charuco_corners=charuco_corners,
            charuco_ids=charuco_ids,
            target_to_camera=None,
            rvec=None,
            tvec=None,
            reprojection_error_px=None,
            error_message="cv2.solvePnP failed for the ChArUco board.",
        )

    projected_points, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        intrinsics.camera_matrix,
        intrinsics.dist_coeffs,
    )
    projected_points = np.asarray(projected_points, dtype=np.float64).reshape(-1, 2)
    reprojection_error = float(np.mean(np.linalg.norm(projected_points - image_points, axis=1)))

    if hasattr(cv2, "drawFrameAxes"):
        cv2.drawFrameAxes(
            preview,
            intrinsics.camera_matrix,
            intrinsics.dist_coeffs,
            rvec,
            tvec,
            board.getSquareLength() if hasattr(board, "getSquareLength") else 0.05,
            2,
        )

    return CharucoDetection(
        success=True,
        preview_bgr=preview,
        marker_ids=marker_ids,
        charuco_corners=charuco_corners,
        charuco_ids=charuco_ids,
        target_to_camera=transform_from_rvec_tvec(rvec, tvec),
        rvec=np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        tvec=np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        reprojection_error_px=reprojection_error,
        error_message="",
    )


def detect_aruco_pose(
    frame_bgr: np.ndarray,
    dictionary,
    intrinsics: CameraIntrinsics,
    marker_length_m: float,
) -> CharucoDetection:
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    params = _detector_parameters()
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    preview = frame_bgr.copy()

    if marker_ids is None or len(marker_ids) == 0:
        return CharucoDetection(
            success=False,
            preview_bgr=preview,
            marker_ids=marker_ids,
            charuco_corners=None,
            charuco_ids=None,
            target_to_camera=None,
            rvec=None,
            tvec=None,
            reprojection_error_px=None,
            error_message="No ArUco markers detected.",
        )

    cv2.aruco.drawDetectedMarkers(preview, marker_corners, marker_ids)

    # We use estimatePoseSingleMarkers for simplicity with a single marker
    # In newer OpenCV versions, this might be deprecated in favor of manual solvePnP,
    # but it usually still works or has a direct replacement.
    rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
        marker_corners,
        marker_length_m,
        intrinsics.camera_matrix,
        intrinsics.dist_coeffs,
    )

    # Take the first detected marker for pose
    rvec = rvecs[0]
    tvec = tvecs[0]

    if hasattr(cv2, "drawFrameAxes"):
        cv2.drawFrameAxes(
            preview,
            intrinsics.camera_matrix,
            intrinsics.dist_coeffs,
            rvec,
            tvec,
            marker_length_m,
            2,
        )

    return CharucoDetection(
        success=True,
        preview_bgr=preview,
        marker_ids=marker_ids,
        charuco_corners=None,
        charuco_ids=None,
        target_to_camera=transform_from_rvec_tvec(rvec, tvec),
        rvec=np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        tvec=np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        reprojection_error_px=0.0, # Not easily available for single marker without manual projectPoints
        error_message="",
    )


def serialize_detection(detection: CharucoDetection) -> dict:
    payload = {
        "success": detection.success,
        "error_message": detection.error_message,
        "num_markers": int(0 if detection.marker_ids is None else len(detection.marker_ids)),
        "num_charuco_corners": int(0 if detection.charuco_ids is None else len(detection.charuco_ids)),
        "charuco_ids": []
        if detection.charuco_ids is None
        else [int(value) for value in np.asarray(detection.charuco_ids).reshape(-1).tolist()],
        "charuco_corners_xy": []
        if detection.charuco_corners is None
        else np.asarray(detection.charuco_corners, dtype=np.float64).reshape(-1, 2).tolist(),
        "reprojection_error_px": detection.reprojection_error_px,
    }
    if detection.target_to_camera is not None:
        payload["target_to_camera"] = serialize_transform(detection.target_to_camera)
    if detection.rvec is not None:
        payload["rvec"] = np.asarray(detection.rvec, dtype=np.float64).reshape(3).tolist()
    if detection.tvec is not None:
        payload["tvec"] = np.asarray(detection.tvec, dtype=np.float64).reshape(3).tolist()
    return payload
