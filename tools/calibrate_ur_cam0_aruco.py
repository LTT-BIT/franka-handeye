#!/usr/bin/env python3
"""固定 cam0 + 左/右臂末端 ArUco / ChArUco：只读采集、离线手眼标定。"""
from __future__ import annotations

import argparse
from collections import deque
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "robot_control/config/right_cam0_handeye.json"
DEFAULT_OUTPUT = PROJECT_ROOT / "grasp/config/ur_cam0_calibration/right"


def transform(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation).reshape(3)
    return result


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def load_config(path):
    config = json.loads(Path(path).read_text())
    board = config["board"]
    kind = board.get("type", "gridboard")
    if kind not in ("gridboard", "charuco"):
        raise ValueError("board.type 必须是 gridboard 或 charuco")
    for key in (("marker_length_m", "square_length_m") if kind == "charuco"
                else ("marker_length_m", "marker_separation_m")):
        value = board.get(key)
        minimum_ok = isinstance(value, (int, float)) and not isinstance(value, bool)
        if (not minimum_ok or not np.isfinite(value)
                or (value < 0 if key == "marker_separation_m" else value <= 0)):
            raise ValueError(f"请先在配置里填写实测的 board.{key}（米），不能填写像素尺寸")
    cols, rows = board["columns"], board["rows"]
    minimum = 2 if kind == "charuco" else 1
    if (type(cols) is not int or type(rows) is not int or cols < minimum or rows < minimum):
        raise ValueError(f"标定板的列数和行数必须是至少 {minimum} 的整数")
    if kind == "gridboard" and cols * rows > 1 and board["marker_separation_m"] <= 0:
        raise ValueError("多标记板的 marker_separation_m 必须大于 0")
    if kind == "charuco":
        if board["marker_length_m"] >= board["square_length_m"]:
            raise ValueError("ChArUco 码边长必须小于棋盘方格边长")
        if type(board.get("legacy_pattern")) is not bool:
            raise ValueError("ChArUco 必须明确配置 legacy_pattern，避免不同 OpenCV 版本混用棋盘布局")
        count = board.get("minimum_charuco_corners")
        if type(count) is not int or not 4 <= count <= (cols - 1) * (rows - 1):
            raise ValueError("minimum_charuco_corners 必须在 4 与棋盘内部交点总数之间")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, board["dictionary"]))
    ids = board["ids_row_major"]
    marker_count = cols * rows // 2 if kind == "charuco" else cols * rows
    if (len(ids) != marker_count or len(set(ids)) != len(ids)
            or any(type(i) is not int or not 0 <= i < len(dictionary.bytesList) for i in ids)):
        raise ValueError("ids_row_major 必须按从左到右、从上到下填写所有不重复的有效 ID")
    if type(board["minimum_visible_markers"]) is not int or not 1 <= board["minimum_visible_markers"] <= marker_count:
        raise ValueError("minimum_visible_markers 必须介于 1 与标记总数之间")
    for key in ("base_frame", "tool_frame", "color_topic", "camera_info_topic"):
        if not isinstance(config.get(key), str) or not config[key].strip():
            raise ValueError(f"配置缺少 {key}")
    return config


def create_charuco_board(board):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, board["dictionary"]))
    ids = np.asarray(board["ids_row_major"], dtype=np.int32)
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        if not board["legacy_pattern"]:
            raise ValueError("当前 OpenCV 仅支持 legacy_pattern=true 的 ChArUco 布局")
        result = cv2.aruco.CharucoBoard_create(board["columns"], board["rows"],
                                             board["square_length_m"], board["marker_length_m"], dictionary)
        result.setIds(ids)
    else:
        result = cv2.aruco.CharucoBoard((board["columns"], board["rows"]),
                                       board["square_length_m"], board["marker_length_m"], dictionary, ids)
        result.setLegacyPattern(board["legacy_pattern"])
    return result


def board_points(board):
    """板左上黑色角为原点，x 向右、y 向下，单位米；角点顺序遵循 ArUco。"""
    length = board["marker_length_m"]
    pitch = length + board["marker_separation_m"]
    points = {}
    for index, marker_id in enumerate(board["ids_row_major"]):
        x, y = (index % board["columns"]) * pitch, (index // board["columns"]) * pitch
        points[marker_id] = np.array([[x, y, 0], [x + length, y, 0],
                                      [x + length, y + length, 0], [x, y + length, 0]], dtype=float)
    return points


def camera_parameters(info, image):
    if info.width != image.shape[1] or info.height != image.shape[0]:
        raise ValueError("图像与 CameraInfo 分辨率不一致")
    if info.distortion_model not in ("plumb_bob", "rational_polynomial"):
        raise ValueError(f"不支持畸变模型 {info.distortion_model!r}；需要标准针孔相机内参")
    k = np.array(info.k, dtype=float).reshape(3, 3)
    d = np.array(info.d, dtype=float)
    if (not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0
            or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1])
            or len(d) not in (0, 4, 5, 8, 12, 14)):
        raise ValueError("CameraInfo 内参无效")
    if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
        raise ValueError("暂不支持 CameraInfo 的 binning/ROI；请使用原始全尺寸彩色图")
    return k, d


def estimate_board_pose(objects, pixels, k, d, max_reprojection_px=3.0):
    # 选择正深度、重投影误差最小的平面解；不能用旋转向量大小选解。
    count, rvecs, tvecs, _ = cv2.solvePnPGeneric(objects, pixels, k, d, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    if count:
        for rvec, tvec in zip(rvecs, tvecs):
            rotation = cv2.Rodrigues(rvec)[0]
            if np.min((rotation @ objects.T + tvec.reshape(3, 1))[2]) <= 0:
                continue
            projected = cv2.projectPoints(objects, rvec, tvec, k, d)[0].reshape(-1, 2)
            error = float(np.sqrt(np.mean(np.sum((projected - pixels) ** 2, axis=1))))
            if np.isfinite(error):
                candidates.append((error, rvec, tvec))
    if not candidates:
        raise ValueError("标定板 PnP 无有效正深度解")
    candidates.sort(key=lambda item: item[0])
    error, rvec, tvec = candidates[0]
    if error > max_reprojection_px:
        raise ValueError(f"重投影 RMSE {error:.2f}px > {max_reprojection_px}px；检查尺寸、ID 顺序与清晰度")
    # 几乎正对、像素尺寸小的平面可能出现不可区分的两个解，拒绝明显的朝向歧义。
    if len(candidates) > 1 and candidates[1][0] - error < 0.1:
        angle = Rotation.from_matrix(cv2.Rodrigues(candidates[1][1])[0].T @ cv2.Rodrigues(rvec)[0]).magnitude()
        if np.degrees(angle) > 10:
            raise ValueError("平面位姿存在歧义，请稍微倾斜标定板或使其在图像中更大")
    return transform(cv2.Rodrigues(rvec)[0], tvec), error


class BoardDetector:
    def __init__(self, board):
        self.board = board
        self.charuco = create_charuco_board(board) if board.get("type") == "charuco" else None
        if self.charuco is not None:
            points = (self.charuco.getObjPoints() if hasattr(self.charuco, "getObjPoints")
                      else self.charuco.objPoints)
            self.points = dict(zip(board["ids_row_major"], points))
            self.chessboard_points = (self.charuco.getChessboardCorners() if hasattr(self.charuco, "getChessboardCorners")
                                      else self.charuco.chessboardCorners)
        else:
            self.points = board_points(board)
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, board["dictionary"]))
        # OpenCV 4.6 exposes the class but its direct constructor can segfault.
        self.parameters = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create")
                           else cv2.aruco.DetectorParameters())
        # ChArUco 由棋盘交点做亚像素定位，避免标记角点被相邻黑格吸附。
        self.parameters.cornerRefinementMethod = (cv2.aruco.CORNER_REFINE_NONE if self.charuco is not None
                                                  else cv2.aruco.CORNER_REFINE_SUBPIX)
        # 默认5px窗口会吸附到紧邻标记的角点；小间隔板使用更小窗口。
        self.parameters.cornerRefinementWinSize = 2
        self.detector = (cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
                         if hasattr(cv2.aruco, "ArucoDetector") else None)

    def detect(self, image, info):
        k, d = camera_parameters(info, image)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = (self.detector.detectMarkers(gray) if self.detector else
                           cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters))
        canvas = image.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(canvas, corners, ids)
        detected = [] if ids is None else ids.flatten().tolist()
        if len(set(detected)) != len(detected):
            return canvas, None, "Duplicate IDs detected"
        indices = [i for i, marker_id in enumerate(detected) if marker_id in self.points]
        if len(indices) < self.board["minimum_visible_markers"]:
            return canvas, None, f"Found IDs {detected}; need {self.board['minimum_visible_markers']} board markers"
        metadata = {}
        if self.charuco is not None:
            selected = [corners[i] for i in indices]
            selected_ids = np.array([detected[i] for i in indices], dtype=np.int32).reshape(-1, 1)
            if hasattr(cv2.aruco, "interpolateCornersCharuco"):
                count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    selected, selected_ids, gray, self.charuco, cameraMatrix=k, distCoeffs=d)
            else:
                parameters = cv2.aruco.CharucoParameters()
                parameters.cameraMatrix, parameters.distCoeffs = k, d
                charuco_corners, charuco_ids, _, _ = cv2.aruco.CharucoDetector(
                    self.charuco, parameters).detectBoard(gray, markerCorners=selected, markerIds=selected_ids)
                if charuco_ids is not None:
                    charuco_corners = charuco_corners.reshape(-1, 1, 2)
                    charuco_ids = charuco_ids.reshape(-1, 1)
                count = 0 if charuco_ids is None else charuco_ids.size
            if count is None or count < self.board["minimum_charuco_corners"]:
                return canvas, None, f"Found {count or 0} ChArUco corners; need {self.board['minimum_charuco_corners']}"
            objects = np.asarray(self.chessboard_points[charuco_ids.flatten()], dtype=float)
            if np.linalg.matrix_rank(objects[:, :2] - objects[0, :2]) < 2:
                return canvas, None, "ChArUco corners are collinear; show a larger board area"
            pixels = charuco_corners.reshape(-1, 2).astype(float)
            cv2.aruco.drawDetectedCornersCharuco(canvas, charuco_corners, charuco_ids)
            metadata = {"charuco_corner_ids": charuco_ids.flatten().tolist(), "object_points_m": objects.tolist()}
        else:
            objects = np.concatenate([self.points[detected[i]] for i in indices])
            pixels = np.concatenate([corners[i].reshape(4, 2) for i in indices]).astype(float)
        try:
            matrix, error = estimate_board_pose(objects, pixels, k, d)
        except ValueError as exc:
            return canvas, None, str(exc)
        cv2.drawFrameAxes(canvas, k, d, Rotation.from_matrix(matrix[:3, :3]).as_rotvec(),
                          matrix[:3, 3], self.board["marker_length_m"])
        result = {"T_cam_board": matrix.tolist(), "reprojection_rmse_px": error,
                  "visible_ids": [detected[i] for i in indices], "corners_px": pixels.tolist(), **metadata}
        label = f"ChArUco OK: {len(objects)} corners" if self.charuco is not None else "Board OK"
        return canvas, result, f"{label}: {len(indices)} markers; reprojection {error:.2f}px"


def rigid_matrix(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1])
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-5)):
        raise ValueError("样本包含无效刚体变换")
    return matrix


def excitation_check(samples):
    if len(samples) < 8:
        raise ValueError("至少需要 8 组样本，建议 25 组")
    rotations = [rigid_matrix(row["T_base_tool"])[:3, :3] for row in samples]
    vectors = [Rotation.from_matrix(rotations[0].T @ r).as_rotvec() for r in rotations[1:]]
    singular = np.linalg.svd(np.array(vectors), compute_uv=False)
    if singular[1] < 0.25:
        raise ValueError("旋转变化不足：不能只平移或只绕一个轴转；请绕至少两个不同轴各变化约 20–40°")
    return singular.tolist()


def errors(samples, base_cam, tool_board):
    result = []
    for index, row in enumerate(samples):
        delta = np.linalg.inv(base_cam @ rigid_matrix(row["T_cam_board"])) @ (rigid_matrix(row["T_base_tool"]) @ tool_board)
        result.append({"sample": row.get("index", index + 1),
                       "translation_m": float(np.linalg.norm(delta[:3, 3])),
                       "rotation_deg": float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude()))})
    return {"translation_rmse_m": float(np.sqrt(np.mean([r["translation_m"] ** 2 for r in result]))),
            "translation_max_m": max(r["translation_m"] for r in result),
            "rotation_rmse_deg": float(np.sqrt(np.mean([r["rotation_deg"] ** 2 for r in result]))),
            "rotation_max_deg": max(r["rotation_deg"] for r in result), "sample_errors": result}


def fit_transforms(samples):
    excitation_check(samples)
    a = [rigid_matrix(row["T_base_tool"]) for row in samples]
    b = [rigid_matrix(row["T_cam_board"]) for row in samples]
    inverse_a = [np.linalg.inv(matrix) for matrix in a]
    # 虚拟 eye-in-hand：gripper2base = inv(T_base_tool)，输出就是 T_base_cam。
    rotation, translation = cv2.calibrateHandEye(
        [m[:3, :3] for m in inverse_a], [m[:3, 3] for m in inverse_a],
        [m[:3, :3] for m in b], [m[:3, 3] for m in b], method=cv2.CALIB_HAND_EYE_PARK)
    x = rigid_matrix(transform(rotation, translation))
    ys = [np.linalg.inv(ai) @ x @ bi for ai, bi in zip(a, b)]
    y = transform(Rotation.from_matrix(np.array([m[:3, :3] for m in ys])).mean().as_matrix(),
                  np.mean([m[:3, 3] for m in ys], axis=0))

    def pack(matrix):
        return np.r_[matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_rotvec()]

    def unpack(vector):
        return transform(Rotation.from_rotvec(vector[3:]).as_matrix(), vector[:3])

    def residual(vector):
        x, y = unpack(vector[:6]), unpack(vector[6:])
        values = []
        for ai, bi in zip(a, b):
            delta = np.linalg.inv(x @ bi) @ (ai @ y)
            values.extend(delta[:3, 3])
            values.extend(0.1 * Rotation.from_matrix(delta[:3, :3]).as_rotvec())
        return values

    fit = least_squares(residual, np.r_[pack(x), pack(y)], loss="soft_l1", f_scale=.003, max_nfev=1000)
    if not fit.success or np.linalg.matrix_rank(fit.jac) < 12:
        raise ValueError("求解未收敛或运动退化，请增加不同方向的旋转样本")
    return unpack(fit.x[:6]), unpack(fit.x[6:])


def quality_pass(metrics):
    return (metrics["translation_rmse_m"] <= .010 and metrics["translation_max_m"] <= .010
            and metrics["rotation_rmse_deg"] <= 2 and metrics["rotation_max_deg"] <= 5)


def solve(data, *, minimum_samples=12):
    samples = data["samples"]
    if len(samples) < minimum_samples:
        raise ValueError(f"带留出验证的标定至少需要 {minimum_samples} 组样本，建议 25 组")
    excitation = excitation_check(samples)
    # 每第五个姿态留出，避免所有留出样本集中在一次采集末尾。
    train = [row for i, row in enumerate(samples) if i % 5 != 4]
    held = [row for i, row in enumerate(samples) if i % 5 == 4]
    x_train, y_train = fit_transforms(train)
    held_metrics = errors(held, x_train, y_train)
    x, y = fit_transforms(samples)
    all_metrics = errors(samples, x, y)
    reprojection_ok = all(np.isfinite(row.get("reprojection_rmse_px", np.inf))
                          and 0 <= row["reprojection_rmse_px"] <= 3.0 for row in samples)
    base_frame = data["config"]["base_frame"]
    return {"schema_version": 2, "model": "eye_to_hand_static_cam0_aruco",
            "equation": "T_base_tool @ T_tool_board = T_base_cam0 @ T_cam_board",
            "point_transform": "p_base = T_base_cam0 @ p_cam0_optical",
            "parent_frame": data["config"]["base_frame"], "child_frame": data["camera"]["frame_id"],
            "tool_frame": data["config"]["tool_frame"], "config": data["config"], "camera": data["camera"],
            "T_base_cam0": x.tolist(), "T_cam0_base": np.linalg.inv(x).tolist(),
            f"T_{base_frame}_cam0": x.tolist(), f"T_cam0_{base_frame}": np.linalg.inv(x).tolist(),
            "T_tool_board": y.tolist(), "translation_m": x[:3, 3].tolist(),
            "quaternion_xyzw": Rotation.from_matrix(x[:3, :3]).as_quat().tolist(),
            "sample_count": len(samples), "rotation_excitation_singular_values_rad": excitation,
            "all_samples": all_metrics, "held_out_validation": held_metrics,
            "reprojection_quality_pass": reprojection_ok,
            "quality_pass": reprojection_ok and quality_pass(all_metrics) and quality_pass(held_metrics)}


def validate(data, calibration):
    """使用新采集姿态检验固定结果；不重新拟合相机或板的安装变换。"""
    check_compatible(data, calibration["config"], calibration["camera"])
    if len(data["samples"]) < 5:
        raise ValueError("独立验证至少需要5组新姿态，建议10组；不要使用标定拟合样本")
    metrics = errors(data["samples"], rigid_matrix(calibration["T_base_cam0"]),
                     rigid_matrix(calibration["T_tool_board"]))
    reprojection_ok = all(np.isfinite(row.get("reprojection_rmse_px", np.inf))
                          and 0 <= row["reprojection_rmse_px"] <= 3.0 for row in data["samples"])
    return {"sample_count": len(data["samples"]), "metrics": metrics,
            "reprojection_quality_pass": reprojection_ok,
            "calibration_quality_pass": calibration["quality_pass"],
            "validation_pass": quality_pass(metrics) and reprojection_ok,
            "parent_frame": calibration["parent_frame"], "child_frame": calibration["child_frame"]}


class CameraTf:
    def __init__(self, config, node_name="ur_cam0_handeye_readonly"):
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        import tf2_ros
        self.config, self.rclpy = config, rclpy
        self.node = Node(node_name)
        self.bridge = CvBridge()
        self.color = self.info = None
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self.node)
        self.node.create_subscription(Image, config["color_topic"], self.on_color, qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo, config["camera_info_topic"], self.on_info, qos_profile_sensor_data)

    def on_color(self, message):
        self.color = message

    def on_info(self, message):
        self.info = message

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=.01)

    def observation(self, detector):
        from rclpy.time import Time
        from tf2_ros import TransformException
        if self.color is None or self.info is None:
            raise ValueError("等待 cam0 图像和 CameraInfo")
        message, info = self.color, self.info
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds == 0 or abs((self.node.get_clock().now() - stamp).nanoseconds) > 1_000_000_000:
            raise ValueError("图像时间戳陈旧或相机/ROS 时钟未对齐（超过1秒）")
        if not message.header.frame_id or message.header.frame_id != info.header.frame_id:
            raise ValueError("Image 与 CameraInfo 必须使用同一非空彩色 optical frame")
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        canvas, board, status = detector.detect(image, info)
        if board is None:
            return canvas, None, status
        deadline = time.monotonic() + .4
        while True:
            try:
                tf = self.buffer.lookup_transform(self.config["base_frame"], self.config["tool_frame"], stamp)
                break
            except TransformException as exc:
                if time.monotonic() >= deadline:
                    return canvas, None, f"无图像时刻的 TF：{exc}"
                self.spin()
        t, q = tf.transform.translation, tf.transform.rotation
        base_tool = transform(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [t.x, t.y, t.z])
        camera = {"frame_id": message.header.frame_id, "width": info.width, "height": info.height,
                  "k": list(info.k), "d": list(info.d), "distortion_model": info.distortion_model}
        return canvas, {**board, "T_base_tool": base_tool.tolist(), "image_stamp_ns": stamp.nanoseconds,
                        "camera": camera}, status


def check_compatible(data, config, camera=None):
    if data["schema_version"] != 2 or data["config"] != config:
        raise ValueError("配置与已有样本不一致；不要混用不同板尺寸、ID、TF 或相机，请使用新输出目录")
    if camera is not None and data.get("camera") != camera:
        raise ValueError("相机内参/分辨率/坐标系与已有样本不一致，请使用新输出目录")


def stationary(previous, current):
    for key, max_translation, max_rotation in (("T_base_tool", .001, .5), ("T_cam_board", .005, 2)):
        delta = np.linalg.inv(np.array(previous[key])) @ np.array(current[key])
        if (np.linalg.norm(delta[:3, 3]) > max_translation
                or np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude()) > max_rotation):
            return False
    return True


def live(config, output, collect):
    import rclpy
    samples_path = output / "samples.json"
    data = json.loads(samples_path.read_text()) if samples_path.exists() else {
        "schema_version": 2, "config": config, "samples": []}
    check_compatible(data, config)
    detector = BoardDetector(config["board"])
    history = deque(maxlen=90)
    rclpy.init(args=[])
    node = CameraTf(config)
    last_stamp = None
    last_notice = 0.0
    print("只读模式。窗口内：空格=保存一个停稳姿态（collect模式）；Q/ESC=结束。", flush=True)
    print("预览请核对 ID、框线和板坐标轴。每次移动后停稳至少1秒。", flush=True)
    try:
        while rclpy.ok():
            node.spin()
            try:
                canvas, row, status = node.observation(detector)
            except ValueError as exc:
                canvas = np.zeros((480, 800, 3), np.uint8)
                row, status = None, str(exc)
            if row:
                if data.get("camera") is not None:
                    check_compatible(data, config, row["camera"])
                if row["image_stamp_ns"] != last_stamp:
                    history.append(row)
                    last_stamp = row["image_stamp_ns"]
            else:
                history.clear()
                if time.monotonic() - last_notice > 3:
                    print(f"等待有效观测：{status}", flush=True)
                    last_notice = time.monotonic()
            # OpenCV默认字体不支持中文；具体错误同时输出终端。
            cv2.putText(canvas, f"Saved: {len(data['samples'])} | SPACE save | Q quit", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
            if status.isascii():
                cv2.putText(canvas, status[:110], (10, 50), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
            cv2.imshow(f"{config['base_frame']} - cam0 {config['board'].get('type', 'gridboard')} (read only)", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == 32:
                if not collect:
                    print(f"预览：{status}；如需保存请用 collect 子命令", flush=True)
                    continue
                if row is None:
                    print(f"本次未保存：{status}", flush=True)
                    continue
                previous = [r for r in history if 0 < (row["image_stamp_ns"] - r["image_stamp_ns"]) / 1e9 <= 1]
                if (len(previous) < 3 or (row["image_stamp_ns"] - previous[0]["image_stamp_ns"]) < 500_000_000
                        or not all(stationary(r, row) for r in previous)):
                    print("本次未保存：需要至少0.5秒稳定的图像与 TF；请停稳，确认板未晃动", flush=True)
                    continue
                if data["samples"]:
                    old = np.array(data["samples"][-1]["T_base_tool"])
                    delta = np.linalg.inv(old) @ np.array(row["T_base_tool"])
                    if np.linalg.norm(delta[:3, 3]) < .02 and Rotation.from_matrix(delta[:3, :3]).magnitude() < np.deg2rad(5):
                        print("本次未保存：与上个姿态太接近，请平移至少2cm或旋转至少5°", flush=True)
                        continue
                camera = row.pop("camera")
                data.setdefault("camera", camera)
                check_compatible(data, config, camera)
                row["index"] = len(data["samples"]) + 1
                row["stability_window_s"] = (row["image_stamp_ns"] - previous[0]["image_stamp_ns"]) / 1e9
                output.mkdir(parents=True, exist_ok=True)
                image_path = output / f"sample_{row['index']:03d}.png"
                if not cv2.imwrite(str(image_path), canvas):
                    raise OSError(f"无法保存图片 {image_path}")
                data["samples"].append(row)
                atomic_json(samples_path, data)
                # 不修改 history 的 row 引用。
                history.clear()
                print(f"已保存第 {row['index']} 组：{samples_path}；重投影 {row['reprojection_rmse_px']:.3f}px", flush=True)
    finally:
        cv2.destroyAllWindows()
        node.node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("preview", "collect", "solve", "validate"))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path,
                        help="样本与结果目录；默认按配置的 base_frame 分开保存左/右臂")
    parser.add_argument("--calibration", type=Path, help="validate模式：要检验的calibration.json")
    args = parser.parse_args(argv)
    if args.mode == "validate" and args.calibration is None:
        parser.error("validate模式需要 --calibration，指向之前求解的calibration.json")
    try:
        config = load_config(args.config.expanduser())
        arm = config["base_frame"].replace("/", "_").removesuffix("_base")
        output_root = (DEFAULT_OUTPUT.parent.with_name("ur_cam0_charuco_calibration")
                       if config["board"].get("type") == "charuco" else DEFAULT_OUTPUT.parent)
        output = (args.output_dir or output_root / arm).expanduser().resolve()
        print(f"标定坐标系：{config['base_frame']} <- cam0；末端：{config['tool_frame']}；目录：{output}", flush=True)
        if args.mode in ("preview", "collect"):
            live(config, output, args.mode == "collect")
        else:
            data = json.loads((output / "samples.json").read_text())
            check_compatible(data, config)
            if args.mode == "validate":
                calibration_path = args.calibration.expanduser().resolve()
                calibration = json.loads(calibration_path.read_text())
                result = validate(data, calibration)
                result["calibration_path"] = str(calibration_path)
                atomic_json(output / "validation.json", result)
                for row in result["metrics"]["sample_errors"]:
                    print(f"样本{row['sample']}: 平移={1000*row['translation_m']:.2f}mm，旋转={row['rotation_deg']:.2f}°")
                m = result["metrics"]
                print(f"独立验证：平移RMSE={1000*m['translation_rmse_m']:.2f}mm，最大={1000*m['translation_max_m']:.2f}mm；"
                      f"旋转RMSE={m['rotation_rmse_deg']:.2f}°，最大={m['rotation_max_deg']:.2f}°")
                print(f"validation_pass={result['validation_pass']}；原标定quality_pass={result['calibration_quality_pass']}")
                print(f"结果：{output / 'validation.json'}")
                return 0 if result["validation_pass"] else 2
            result = solve(data)
            atomic_json(output / "calibration.json", result)
            print(json.dumps({k: result[k] for k in ("parent_frame", "child_frame", "sample_count",
                                                    "T_base_cam0", "quality_pass")}, indent=2))
            for key in ("all_samples", "held_out_validation"):
                m = result[key]
                print(f"{key}: 平移 RMSE={1000*m['translation_rmse_m']:.2f}mm，"
                      f"最大={1000*m['translation_max_m']:.2f}mm；"
                      f"旋转 RMSE={m['rotation_rmse_deg']:.2f}°，最大={m['rotation_max_deg']:.2f}°")
            print(f"结果：{output / 'calibration.json'}")
            if not result["quality_pass"]:
                print("质量未通过：结果仅供诊断，查看 sample_errors；检查板尺寸、固定方式、同步与姿态覆盖", file=sys.stderr)
                return 2
    except KeyboardInterrupt:
        return 130
    except (ValueError, OSError, KeyError, AttributeError, cv2.error) as exc:
        print(f"标定失败：{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
