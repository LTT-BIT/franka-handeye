"""Read-only hardware input: ROS camera with libfranka state or existing TF."""
from __future__ import annotations

import time
from .core import solver


class RosSource:
    def __init__(self, cfg):
        import rclpy
        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=[])
        self.reader = solver.CameraTf(cfg, node_name="franka_handeye_browser_readonly")
        self.detector = solver.BoardDetector(cfg["board"])
        self.franka = None
        try:
            if cfg.get("pose_source") == "libfranka":
                from .franka_readonly import FrankaReader
                self.franka = FrankaReader(cfg["robot_ip"])
        except Exception:
            self.reader.node.destroy_node()
            raise
        # Drain Image/TF subscriptions continuously even during ChArUco detection.
        # A single spin_once per video frame would fall behind high-rate TF.
        from rclpy.executors import SingleThreadedExecutor
        import threading
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.reader.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.reader.spin = lambda: time.sleep(.005)

    def next(self):
        self.reader.spin()
        if self.franka:
            return self.manual_observation()
        return self.reader.observation(self.detector)

    def manual_observation(self):
        from rclpy.time import Time
        message, info = self.reader.color, self.reader.info
        if message is None or info is None:
            raise ValueError("等待头部相机图像和 CameraInfo")
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds <= 0 or abs((self.reader.node.get_clock().now()-stamp).nanoseconds) > 1_000_000_000:
            raise ValueError("相机时间戳陈旧或 ROS 时钟不一致")
        if abs(time.time_ns()-stamp.nanoseconds) > 1_000_000_000:
            raise ValueError("只读 Franka 通道要求相机 / ROS 使用同机 system/global 时间，不能使用仿真时钟")
        if not message.header.frame_id or message.header.frame_id != info.header.frame_id:
            raise ValueError("Image 与 CameraInfo 必须使用同一非空 optical frame")
        image = self.reader.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        canvas, board, status = self.detector.detect(image, info)
        if board is None:
            return canvas, None, status
        try:
            matrix, timing = self.franka.at(stamp.nanoseconds)
        except ValueError as exc:
            return canvas, None, str(exc)
        camera = {"frame_id": message.header.frame_id, "width": info.width, "height": info.height,
                  "k": list(info.k), "d": list(info.d), "distortion_model": info.distortion_model}
        return canvas, {**board, "T_base_tool": matrix.tolist(), "image_stamp_ns": stamp.nanoseconds,
                        "camera": camera, "pose_timing": timing}, status

    def close(self):
        if self.franka:
            self.franka.close()
        self.executor.shutdown(timeout_sec=2)
        self.spin_thread.join(timeout=2)
        self.reader.node.destroy_node()
        # TransformListener owns additional subscriptions on this node.
