"""Session storage and calibration rules shared by the browser and tests."""
from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import time
import uuid
from collections import deque

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("handeye_solver", ROOT / "tools/calibrate_ur_cam0_aruco.py")
solver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(solver)


def default_config(side="left", profile="camera"):
    cfg = solver.load_config(ROOT / "robot_control/config/left_cam0_charuco_handeye.json")
    cfg.update(base_frame=f"{side}_fr3_link0", tool_frame=f"{side}_fr3_link8",
               color_topic=f"/{profile}/color/image_raw", camera_info_topic=f"/{profile}/color/camera_info")
    cfg.update(pose_source="libfranka", robot_ip="172.16.26.5" if side == "left" else "172.16.26.4")
    return cfg


def validate_config(cfg):
    # Reuse the established board checks without temporary files.
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "config.json"
        path.write_text(json.dumps(cfg, allow_nan=False))
        result = solver.load_config(path)
    if result["base_frame"] == result["tool_frame"]:
        raise ValueError("基座与末端坐标系不能相同")
    if result.get("pose_source") not in ("libfranka", "tf"):
        raise ValueError("pose_source 必须是 libfranka 或 tf")
    if result["pose_source"] == "libfranka":
        import ipaddress
        ipaddress.ip_address(result["robot_ip"])
        if result["base_frame"] not in ("left_fr3_link0", "right_fr3_link0") or result["tool_frame"] != result["base_frame"].replace("link0", "link8"):
            raise ValueError("libfranka 模式固定输出该臂基座 link0 → 法兰 link8；自定义坐标链请使用 tf 模式")
    return result


class Session:
    def __init__(self, root, cfg, mode, side, purpose="fit", calibration_id=None):
        if side not in ("left", "right") or mode not in ("live", "demo") or purpose not in ("fit", "validation"):
            raise ValueError("无效的侧别、模式或数据用途")
        self.id = time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
        self.path = Path(root) / mode / side / self.id
        self.data = {"schema_version": 2, "config": copy.deepcopy(cfg), "samples": [],
                     "session": {"id": self.id, "mode": mode, "side": side, "purpose": purpose,
                                 "calibration_id": calibration_id}}
        self.history = deque(maxlen=120)
        self.row = None
        self.canvas = None
        self.received_at = 0
        self.status = "等待图像、CameraInfo 与图像时刻的 TF"
        self.result = None
        self.persist()

    @classmethod
    def load(cls, path):
        self = cls.__new__(cls)
        self.path = Path(path)
        self.data = json.loads((self.path / "samples.json").read_text())
        self.id = self.data["session"]["id"]
        self.history = deque(maxlen=120)
        self.row = self.canvas = self.result = None
        self.received_at = 0
        self.status = "历史会话；连接数据源后可继续采样"
        result_name = "calibration.json" if self.data["session"]["purpose"] == "fit" else "validation.json"
        if (self.path / result_name).exists():
            self.result = json.loads((self.path / result_name).read_text())
        return self

    def persist(self):
        solver.atomic_json(self.path / "samples.json", self.data)

    def observe(self, canvas, row, status):
        self.canvas, self.status = canvas, status
        if row is None:
            self.row = None
            self.history.clear()
            return
        if self.row is not None and row["image_stamp_ns"] <= self.row["image_stamp_ns"]:
            if row["image_stamp_ns"] < self.row["image_stamp_ns"]:
                self.row = None
                self.history.clear()
                self.status = "图像时间戳倒退，等待新的稳定窗口"
            return
        if "camera" in self.data:
            solver.check_compatible(self.data, self.data["config"], row["camera"])
        self.row = copy.deepcopy(row)
        self.received_at = time.monotonic()
        self.history.append(self.row)

    def readiness(self):
        if self.row is None or time.monotonic() - self.received_at > .8:
            return False, "等待新鲜且有效的图像 / TF"
        row = self.row
        previous = [r for r in self.history if 0 <= row["image_stamp_ns"] - r["image_stamp_ns"] <= 1_000_000_000]
        if len(previous) < 4 or row["image_stamp_ns"] - previous[0]["image_stamp_ns"] < 500_000_000:
            return False, "停稳至少 0.5 秒，建议 1 秒"
        if any(b["image_stamp_ns"] - a["image_stamp_ns"] > 250_000_000 for a, b in zip(previous, previous[1:])):
            return False, "观测存在超过 250 ms 的间断，等待连续数据"
        if not all(solver.stationary(r, row) for r in previous):
            return False, "机械臂或板仍在移动"
        for old in self.data["samples"]:
            delta = np.linalg.inv(solver.rigid_matrix(old["T_base_tool"])) @ solver.rigid_matrix(row["T_base_tool"])
            if np.linalg.norm(delta[:3, 3]) < .02 and Rotation.from_matrix(delta[:3, :3]).magnitude() < np.deg2rad(5):
                return False, "姿态已采集，请平移至少 2 cm 或旋转至少 5°"
        return True, "已停稳，可以采样"

    def capture(self):
        ready, reason = self.readiness()
        if not ready:
            raise ValueError(reason)
        row = copy.deepcopy(self.row)
        camera = row.pop("camera")
        solver.check_compatible(self.data, self.data["config"], camera if "camera" in self.data else None)
        self.data.setdefault("camera", camera)
        row["index"] = len(self.data["samples"]) + 1
        window = [r for r in self.history if 0 <= row["image_stamp_ns"] - r["image_stamp_ns"] <= 1_000_000_000]
        row["stability_window_s"] = (row["image_stamp_ns"] - window[0]["image_stamp_ns"]) / 1e9
        image_path = self.path / f"sample_{row['index']:03d}.png"
        if not cv2.imwrite(str(image_path), self.canvas):
            raise OSError("检查图保存失败，本次未保存样本")
        self.data["samples"].append(row)
        try:
            self.persist()
        except Exception:
            self.data["samples"].pop()
            raise
        self.history.clear()
        # A previous fit no longer represents the current data.
        self.result = None
        for name in ("calibration.json", "validation.json"):
            if (self.path / name).exists():
                (self.path / name).replace(self.path / (name + ".previous"))
        return row["index"]

    def solve(self):
        if self.data["session"]["purpose"] != "fit":
            raise ValueError("独立验证会话不能拟合")
        result = solver.solve(self.data, minimum_samples=10)
        result.update(model="franka_eye_to_hand_static_head_charuco", session=self.data["session"],
                      camera_identity="operator_must_verify_driver_serial", independent_validation_pass=False,
                      deployment_eligible=False)
        result["T_base_camera"] = result["T_base_cam0"]
        result["T_camera_base"] = result["T_cam0_base"]
        self.result = result
        solver.atomic_json(self.path / "calibration.json", result)
        return result

    def validate(self, calibration):
        if self.data["session"]["purpose"] != "validation":
            raise ValueError("请新建独立验证会话，采集未参与拟合的新姿态")
        if calibration["session"]["id"] != self.data["session"]["calibration_id"]:
            raise ValueError("独立验证关联的标定会话不匹配")
        if calibration["session"]["mode"] != self.data["session"]["mode"]:
            raise ValueError("仿真与真实数据不能混用")
        result = solver.validate(self.data, calibration)
        result.update(session=self.data["session"], calibration_id=calibration["session"]["id"],
                      deployment_eligible=False)
        # Validation does not automatically authorize installation on a robot.
        self.result = result
        solver.atomic_json(self.path / "validation.json", result)
        return result
