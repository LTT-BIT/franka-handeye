#!/usr/bin/env python3
"""cam0/左 UR 杯子定位标定工具。

模式：
  --camera-check       读取 cam0 彩色、深度和 CameraInfo，保存内参快照
  --collect-handeye    用固定在左 O30/末端的 ArUco 板采集眼在手外样本
  --solve-handeye      从样本求 T_left_base_cam0
  --detect-cup         用 YOLO 杯口模型和深度输出杯子基座坐标，只读不运动
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DIR = PROJECT_ROOT / "grasp/config/cup_calibration"
DEFAULT_COLOR = "/cam0/color/image_raw"
DEFAULT_DEPTH = "/cam0/depth/image_raw"
DEFAULT_INFO = "/cam0/color/camera_info"
DEFAULT_DEPTH_INFO = "/cam0/depth/camera_info"
DEFAULT_BASE_FRAME = "left_base"
DEFAULT_TOOL_FRAME = "left_tool0"


def pose_matrix(position, rotation):
    result = np.eye(4, dtype=float)
    result[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    result[:3, 3] = np.asarray(position, dtype=float).reshape(3)
    return result


def pose_from_matrix(matrix):
    matrix = np.asarray(matrix, dtype=float)
    return matrix[:3, 3].tolist(), Rotation.from_matrix(matrix[:3, :3]).as_rotvec().tolist()


def json_matrix(matrix):
    return np.asarray(matrix, dtype=float).tolist()


def matrix_from_json(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("变换矩阵必须是有限的 4x4 数组")
    return matrix


def frame_from_vector(vector):
    vector = np.asarray(vector, dtype=float)
    return pose_matrix(vector[:3], Rotation.from_rotvec(vector[3:]).as_matrix())


def vector_from_frame(frame):
    return np.r_[frame[:3, 3], Rotation.from_matrix(frame[:3, :3]).as_rotvec()]


def se3_mean(matrices):
    """Average rigid transforms: mean translation plus SVD-projected mean rotation."""
    matrices = [np.asarray(matrix, dtype=float) for matrix in matrices]
    translation = np.mean([matrix[:3, 3] for matrix in matrices], axis=0)
    rotation_sum = np.sum([matrix[:3, :3] for matrix in matrices], axis=0)
    u, _, vt = np.linalg.svd(rotation_sum)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = u @ vt
    return pose_matrix(translation, rotation)


def solve_eye_to_hand(samples):
    """Solve T_base_cam and T_tool_tag from T_base_tool*T_tool_tag=T_base_cam*T_cam_tag."""
    if len(samples) < 8:
        raise ValueError("至少需要 8 个手眼样本，建议 15~25 个")
    base_tools = [matrix_from_json(row["T_base_tool"]) for row in samples]
    cam_tags = [matrix_from_json(row["T_cam_tag"]) for row in samples]
    tool_tag_initial = np.linalg.inv(base_tools[0]) @ cam_tags[0]
    base_cam_initial = None
    try:
        # Closed form A·Y = Y·B over every pose pair: A is the tool motion in the
        # base frame, B the tag motion in the camera frame, so Y = T_tool_tag.
        # Use it as the starting point so the nonlinear refinement does not have
        # to pull T_base_cam out of an identity guess on real (noisy) samples.
        r_gripper2base, t_gripper2base, r_target2cam, t_target2cam = [], [], [], []
        for i in range(len(base_tools)):
            for j in range(i + 1, len(base_tools)):
                a = np.linalg.inv(base_tools[j]) @ base_tools[i]
                b = np.linalg.inv(cam_tags[j]) @ cam_tags[i]
                r_gripper2base.append(a[:3, :3])
                t_gripper2base.append(a[:3, 3])
                r_target2cam.append(b[:3, :3])
                t_target2cam.append(b[:3, 3])
        r_tool_tag, t_tool_tag = cv2.calibrateHandEye(
            r_gripper2base, t_gripper2base, r_target2cam, t_target2cam,
            method=cv2.CALIB_HAND_EYE_PARK,
        )
        tool_tag_initial = pose_matrix(np.asarray(t_tool_tag).reshape(3), r_tool_tag)
        base_cam_initial = se3_mean(
            base_tools[i] @ tool_tag_initial @ np.linalg.inv(cam_tags[i])
            for i in range(len(base_tools))
        )
        tool_tag_initial = se3_mean(
            np.linalg.inv(base_tools[i]) @ base_cam_initial @ cam_tags[i]
            for i in range(len(base_tools))
        )
    except Exception:
        base_cam_initial = None  # degenerate motion; fall back to identity guess
    x0 = np.r_[
        vector_from_frame(base_cam_initial) if base_cam_initial is not None else np.zeros(6),
        vector_from_frame(tool_tag_initial),
    ]

    def residual(x):
        base_cam = frame_from_vector(x[:6])
        tool_tag = frame_from_vector(x[6:])
        result = []
        for base_tool, cam_tag in zip(base_tools, cam_tags):
            lhs = base_tool @ tool_tag
            rhs = base_cam @ cam_tag
            delta = np.linalg.inv(rhs) @ lhs
            result.extend(delta[:3, 3])
            result.extend(.05 * Rotation.from_matrix(delta[:3, :3]).as_rotvec())
        return np.asarray(result)

    fit = least_squares(residual, x0, loss="soft_l1", f_scale=.003, max_nfev=3000)
    base_cam = frame_from_vector(fit.x[:6])
    tool_tag = frame_from_vector(fit.x[6:])
    errors = []
    for base_tool, cam_tag in zip(base_tools, cam_tags):
        delta = np.linalg.inv(base_cam @ cam_tag) @ (base_tool @ tool_tag)
        errors.append({
            "translation_m": float(np.linalg.norm(delta[:3, 3])),
            "rotation_deg": float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude())),
        })
    return {
        "schema_version": 1,
        "model": "eye_to_hand_static_cam0",
        "T_left_base_cam0": json_matrix(base_cam),
        "T_left_tool_tag": json_matrix(tool_tag),
        "samples": len(samples),
        "translation_rmse_m": float(np.sqrt(np.mean([e["translation_m"] ** 2 for e in errors]))),
        "translation_max_m": max(e["translation_m"] for e in errors),
        "rotation_rmse_deg": float(np.sqrt(np.mean([e["rotation_deg"] ** 2 for e in errors]))),
        "rotation_max_deg": max(e["rotation_deg"] for e in errors),
        "sample_errors": errors,
    }


class CameraRobotNode:
    def __init__(self, color_topic, depth_topic, info_topic, depth_info_topic, base_frame, tool_frame):
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        import tf2_ros

        self.rclpy = rclpy
        self.node = Node("cup_calibration_readonly")
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_depth = None
        self.latest_info = None
        self.latest_depth_info = None
        self.color_generation = 0
        self.depth_generation = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node, spin_thread=False)
        self.node.create_subscription(Image, color_topic, self._color, qos_profile_sensor_data)
        self.node.create_subscription(Image, depth_topic, self._depth, qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo, info_topic, self._info, qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo, depth_info_topic, self._depth_info, qos_profile_sensor_data)
        self.base_frame, self.tool_frame = base_frame, tool_frame

    def _color(self, message):
        self.latest_color = message
        self.color_generation += 1

    def _depth(self, message):
        self.latest_depth = message
        self.depth_generation += 1

    def _info(self, message):
        self.latest_info = message

    def _depth_info(self, message):
        self.latest_depth_info = message

    def spin(self, seconds=.05):
        self.rclpy.spin_once(self.node, timeout_sec=seconds)

    def wait(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin()
            if predicate():
                return
        raise TimeoutError("等待 cam0/TF 数据超时")

    def transform_base_tool(self):
        from rclpy.time import Time
        tf = self.tf_buffer.lookup_transform(self.base_frame, self.tool_frame, Time())
        t, q = tf.transform.translation, tf.transform.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return pose_matrix([t.x, t.y, t.z], rotation)

    def close(self):
        self.node.destroy_node()


def ros_init():
    import rclpy
    owns = not rclpy.ok()
    if owns:
        rclpy.init(args=None)
    return rclpy, owns


def camera_check(args):
    rclpy, owns = ros_init()
    node = CameraRobotNode(args.color_topic, args.depth_topic, args.info_topic, args.depth_info_topic,
                           args.base_frame, args.tool_frame)
    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    try:
        node.wait(lambda: node.latest_color is not None and node.latest_depth is not None
                  and node.latest_info is not None and node.latest_depth_info is not None, args.timeout)
        info = node.latest_info
        color = node.bridge.imgmsg_to_cv2(node.latest_color, desired_encoding="bgr8")
        depth = node.bridge.imgmsg_to_cv2(node.latest_depth, desired_encoding="passthrough")
        depth = np.asarray(depth)
        scale = args.depth_scale if depth.dtype == np.uint16 else 1.0
        valid = depth[np.isfinite(depth) & (depth > 0)] * scale
        metadata = {
            "schema_version": 1,
            "color_topic": args.color_topic,
            "depth_topic": args.depth_topic,
            "color_shape": list(color.shape),
            "depth_shape": list(depth.shape),
            "color_frame": node.latest_color.header.frame_id,
            "depth_frame": node.latest_depth.header.frame_id,
            "camera_info_frame": info.header.frame_id,
            "depth_camera_info_frame": node.latest_depth_info.header.frame_id,
            "width": int(info.width), "height": int(info.height),
            "K": list(info.k), "D": list(info.d),
            "depth_width": int(node.latest_depth_info.width),
            "depth_height": int(node.latest_depth_info.height),
            "depth_K": list(node.latest_depth_info.k),
            "depth_D": list(node.latest_depth_info.d),
            "depth_encoding": node.latest_depth.encoding,
            "depth_scale_m": scale,
            "valid_depth_min_m": float(valid.min()) if valid.size else None,
            "valid_depth_max_m": float(valid.max()) if valid.size else None,
            "valid_depth_median_m": float(np.median(valid)) if valid.size else None,
        }
        (output / "cam0_intrinsics.json").write_text(json.dumps(metadata, indent=2) + "\n")
        cv2.imwrite(str(output / "camera_check_color.jpg"), color)
        np.save(output / "camera_check_depth.npy", depth)
        print(json.dumps(metadata, ensure_ascii=False, indent=2))
        if tuple(depth.shape[:2]) != tuple(color.shape[:2]):
            raise RuntimeError("彩色和深度分辨率不同；先确认深度已对齐到彩色坐标，不能直接取同一像素")
    finally:
        node.close()
        if owns:
            rclpy.shutdown()


def detect_tag(image, info, tag_id, tag_size_m):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        raise ValueError("当前图像未检测到 ArUco 标定板")
    matches = [index for index, value in enumerate(ids.flatten()) if int(value) == tag_id]
    if len(matches) != 1:
        raise ValueError(f"ArUco id={tag_id} 检测数量为 {len(matches)}")
    index = matches[0]
    camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(info.d, dtype=np.float64)
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
        [corners[index]], tag_size_m, camera_matrix, distortion)
    rotation, _ = cv2.Rodrigues(rvec[0, 0])
    return pose_matrix(tvec[0, 0], rotation), corners[index][0].tolist()


def collect_handeye(args):
    rclpy, owns = ros_init()
    node = CameraRobotNode(args.color_topic, args.depth_topic, args.info_topic, args.depth_info_topic,
                           args.base_frame, args.tool_frame)
    output = Path(args.samples).expanduser().resolve()
    samples = json.loads(output.read_text()).get("samples", []) if output.exists() else []
    try:
        print(f"眼在手外采集：共 {args.count} 个姿态；不要让标定板被遮挡。")
        for index in range(len(samples), args.count):
            input(f"移动左臂到第 {index + 1}/{args.count} 个姿态，停稳后按回车采集…")
            node.wait(lambda: node.latest_color is not None and node.latest_info is not None, args.timeout)
            image = node.bridge.imgmsg_to_cv2(node.latest_color, desired_encoding="bgr8")
            tag, corners = detect_tag(image, node.latest_info, args.tag_id, args.tag_size_m)
            base_tool = node.transform_base_tool()
            samples.append({
                "index": index,
                "color_stamp_ns": int(node.latest_color.header.stamp.sec * 1_000_000_000
                                       + node.latest_color.header.stamp.nanosec),
                "T_base_tool": json_matrix(base_tool),
                "T_cam_tag": json_matrix(tag),
                "corners_px": corners,
            })
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps({"schema_version": 1, "samples": samples}, indent=2) + "\n")
            print(f"已保存样本 {index + 1}；T_base_tool 和 T_cam_tag 均有效。")
    finally:
        node.close()
        if owns:
            rclpy.shutdown()


def solve_handeye(args):
    path = Path(args.samples).expanduser().resolve()
    data = json.loads(path.read_text())
    result = solve_eye_to_hand(data.get("samples", []))
    output = Path(args.calibration).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result["translation_rmse_m"] > args.max_translation_rmse:
        raise RuntimeError("手眼平移残差过大，不能用于抓杯")
    if result["rotation_rmse_deg"] > args.max_rotation_rmse:
        raise RuntimeError("手眼旋转残差过大，不能用于抓杯")


def detect_cup(args):
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError("杯子探测需要 ultralytics；请在部署环境安装后再运行") from exc
    calibration = json.loads(Path(args.calibration).read_text())
    base_cam = matrix_from_json(calibration["T_left_base_cam0"])
    model = YOLO(str(Path(args.model).expanduser()))
    rclpy, owns = ros_init()
    node = CameraRobotNode(args.color_topic, args.depth_topic, args.info_topic, args.depth_info_topic,
                           args.base_frame, args.tool_frame)
    positions = []
    try:
        last_generation = -1
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and len(positions) < args.frames:
            node.spin()
            if node.color_generation == last_generation or node.latest_color is None or node.latest_depth is None:
                continue
            last_generation = node.color_generation
            color = node.bridge.imgmsg_to_cv2(node.latest_color, desired_encoding="bgr8")
            depth = np.asarray(node.bridge.imgmsg_to_cv2(node.latest_depth, desired_encoding="passthrough"))
            if tuple(depth.shape[:2]) != tuple(color.shape[:2]):
                raise RuntimeError("彩色和深度分辨率不同；不能安全把杯子像素转换为三维坐标")
            result = model.predict(color, conf=args.confidence, iou=0.45, max_det=1, verbose=False)[0]
            if result.boxes is None or len(result.boxes) != 1:
                continue
            box = result.boxes.xyxy[0].cpu().numpy().tolist()
            u = int(round((box[0] + box[2]) / 2))
            v = int(round((box[1] + box[3]) / 2))
            radius = max(2, int(round(min(box[2] - box[0], box[3] - box[1]) * .12)))
            patch = depth[max(0, v-radius):v+radius+1, max(0, u-radius):u+radius+1]
            scale = args.depth_scale if depth.dtype == np.uint16 else 1.0
            valid = patch[np.isfinite(patch) & (patch > 0)] * scale
            if valid.size < 10:
                continue
            z = float(np.median(valid))
            info = node.latest_info
            fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
            point_cam = np.array([(u-cx)*z/fx, (v-cy)*z/fy, z, 1.0])
            point_base = base_cam @ point_cam
            positions.append(point_base[:3])
        if len(positions) < args.frames:
            raise TimeoutError(f"只获得 {len(positions)}/{args.frames} 个稳定杯子观测")
        values = np.asarray(positions)
        result = {"schema_version": 1, "frames": len(values),
                  "cup_center_left_base_m": values.mean(axis=0).tolist(),
                  "std_m": values.std(axis=0).tolist(),
                  "max_frame_spread_m": float(np.max(np.linalg.norm(values-values.mean(axis=0), axis=1)))}
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if result["max_frame_spread_m"] > args.max_spread_m:
            raise RuntimeError("杯子多帧位置抖动过大，拒绝生成抓取目标")
    finally:
        node.close()
        if owns:
            rclpy.shutdown()


def cup_pose_from_ellipse(ellipse, info, cup_radius_m):
    """Estimate cup rim center/axis in cam0 coordinates from a known-radius ellipse."""
    from grasp.hybrid.vision import fit_known_circle

    camera_matrix = np.asarray(info.k, dtype=float).reshape(3, 3)
    k = dict(fx=float(camera_matrix[0, 0]), fy=float(camera_matrix[1, 1]),
             cx=float(camera_matrix[0, 2]), cy=float(camera_matrix[1, 2]))
    fit = fit_known_circle(
        ellipse, cup_radius_m, k, np.array([0.0, 0.0, -1.0]),
        max_tilt_deg=50.0, max_rmse_px=3.0,
    )
    axis = np.asarray(fit["axis"], dtype=float)
    axis /= np.linalg.norm(axis)
    if axis[2] > 0:
        axis *= -1.0
    image_x = np.array([1.0, 0.0, 0.0])
    x_axis = image_x - axis * float(image_x @ axis)
    if np.linalg.norm(x_axis) < 1e-6:
        x_axis = np.array([0.0, 1.0, 0.0]) - axis * float(axis[1])
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    rotation = np.column_stack((x_axis, y_axis, axis))
    fit.update(
        center_camera_m=np.asarray(fit["center_m"], dtype=float).tolist(),
        axis_camera=axis.tolist(),
        rotation_camera_cup=rotation.tolist(),
        pose_convention="cup z axis points toward cam0 (negative camera Z branch)",
    )
    return fit


def detect_cup_camera(args):
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError("杯子探测需要 ultralytics；请在当前 .venv 安装后再运行") from exc
    model = YOLO(str(Path(args.model).expanduser()))
    rclpy, owns = ros_init()
    node = CameraRobotNode(args.color_topic, args.depth_topic, args.info_topic, args.depth_info_topic,
                           args.base_frame, args.tool_frame)
    poses = []
    last_generation = -1
    output = Path(args.pose_output).expanduser().resolve()
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and len(poses) < args.frames:
            node.spin()
            if node.color_generation == last_generation or node.latest_color is None or node.latest_info is None:
                continue
            last_generation = node.color_generation
            image = node.bridge.imgmsg_to_cv2(node.latest_color, desired_encoding="bgr8")
            prediction = model.predict(image, conf=args.confidence, iou=0.45, max_det=1, verbose=False)[0]
            if prediction.boxes is None or len(prediction.boxes) != 1:
                continue
            if prediction.masks is None or len(prediction.masks.xy) != 1:
                continue
            polygon = np.asarray(prediction.masks.xy[0], dtype=np.float32)
            if polygon.shape[0] < 5:
                continue
            ellipse = cv2.fitEllipse(polygon)
            fit = cup_pose_from_ellipse(ellipse, node.latest_info, args.cup_radius_m)
            center_u, center_v = int(round(ellipse[0][0])), int(round(ellipse[0][1]))
            depth_validation_m = None
            if node.latest_depth is not None:
                depth = np.asarray(node.bridge.imgmsg_to_cv2(node.latest_depth, desired_encoding="passthrough"))
                if depth.shape[:2] == image.shape[:2]:
                    radius = max(2, int(round(min(ellipse[1]) * .08)))
                    patch = depth[max(0, center_v-radius):center_v+radius+1,
                                  max(0, center_u-radius):center_u+radius+1]
                    scale = args.depth_scale if depth.dtype == np.uint16 else 1.0
                    valid = patch[np.isfinite(patch) & (patch > 0)] * scale
                    if valid.size >= 10:
                        depth_validation_m = float(np.median(valid))
                        fit["depth_center_error_m"] = abs(depth_validation_m - fit["center_m"][2])
            fit["ellipse"] = [[float(v) for v in ellipse[0]],
                               [float(v) for v in ellipse[1]], float(ellipse[2])]
            fit["confidence"] = float(prediction.boxes.conf[0].cpu().item())
            fit["depth_center_m"] = depth_validation_m
            poses.append(fit)
        if len(poses) < args.frames:
            raise TimeoutError(f"只获得 {len(poses)}/{args.frames} 个有效杯口位姿")
        centers = np.asarray([p["center_camera_m"] for p in poses])
        axes = np.asarray([p["axis_camera"] for p in poses])
        mean_center = centers.mean(axis=0)
        mean_axis = axes.mean(axis=0)
        mean_axis /= np.linalg.norm(mean_axis)
        result = {
            "schema_version": 1,
            "frame": node.latest_color.header.frame_id,
            "frames": len(poses),
            "cup_radius_m": args.cup_radius_m,
            "center_camera_m": mean_center.tolist(),
            "axis_camera": mean_axis.tolist(),
            "center_std_m": centers.std(axis=0).tolist(),
            "max_center_spread_m": float(np.max(np.linalg.norm(centers - mean_center, axis=1))),
            "confidence_min": min(p["confidence"] for p in poses),
            "confidence_mean": float(np.mean([p["confidence"] for p in poses])),
            "observations": poses,
        }
        if result["max_center_spread_m"] > args.max_spread_m:
            raise RuntimeError("杯口中心多帧抖动过大，拒绝输出稳定位姿")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        node.close()
        if owns:
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--camera-check", action="store_true")
    mode.add_argument("--collect-handeye", action="store_true")
    mode.add_argument("--solve-handeye", action="store_true")
    mode.add_argument("--detect-cup", action="store_true")
    mode.add_argument("--detect-cup-camera", action="store_true")
    parser.add_argument("--color-topic", default=DEFAULT_COLOR)
    parser.add_argument("--depth-topic", default=DEFAULT_DEPTH)
    parser.add_argument("--info-topic", default=DEFAULT_INFO)
    parser.add_argument("--depth-info-topic", default=DEFAULT_DEPTH_INFO)
    parser.add_argument("--depth-scale", type=float, default=.001)
    parser.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    parser.add_argument("--tool-frame", default=DEFAULT_TOOL_FRAME)
    parser.add_argument("--timeout", type=float, default=30.)
    parser.add_argument("--output", default=str(DEFAULT_DIR))
    parser.add_argument("--samples", default=str(DEFAULT_DIR / "handeye_samples.json"))
    parser.add_argument("--calibration", default=str(DEFAULT_DIR / "cam0_left_handeye.json"))
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size-m", type=float, default=.04)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--max-translation-rmse", type=float, default=.005)
    parser.add_argument("--max-rotation-rmse", type=float, default=2.)
    parser.add_argument("--model", default=str(PROJECT_ROOT / "grasp/runs/cup_20261021_215/cup/weights/best.pt"))
    parser.add_argument("--confidence", type=float, default=.64)
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--max-spread-m", type=float, default=.005)
    parser.add_argument("--pose-output", default=str(DEFAULT_DIR / "cup_pose_camera.json"))
    parser.add_argument("--cup-radius-m", type=float, help="实测杯口内半径（米）；相机系三维位姿模式必填")
    args = parser.parse_args(argv)
    if args.camera_check:
        return camera_check(args)
    if args.collect_handeye:
        return collect_handeye(args)
    if args.solve_handeye:
        return solve_handeye(args)
    if args.detect_cup_camera:
        if args.cup_radius_m is None or not math.isfinite(args.cup_radius_m) or args.cup_radius_m <= 0:
            parser.error("--detect-cup-camera 必须提供实测的 --cup-radius-m（米）")
        return detect_cup_camera(args)
    return detect_cup(args)


if __name__ == "__main__":
    raise SystemExit(main())
