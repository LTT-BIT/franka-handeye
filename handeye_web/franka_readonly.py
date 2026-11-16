"""Passive libfranka subprocess and host-timestamp pose interpolation."""
from __future__ import annotations

from collections import deque
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .core import ROOT, solver


def flange_pose(state):
    # Vendor arrays are column-major. O_T_EE includes configurable EE offset.
    o_ee = solver.rigid_matrix(np.array(state["O_T_EE"]).reshape(4, 4, order="F"))
    f_ee = solver.rigid_matrix(np.array(state["F_T_EE"]).reshape(4, 4, order="F"))
    return o_ee @ np.linalg.inv(f_ee)


def interpolate_pose(states, image_stamp_ns):
    before = after = None
    for state in states:
        if state["host_stamp_ns"] <= image_stamp_ns:
            before = state
        if state["host_stamp_ns"] >= image_stamp_ns:
            after = state
            break
    if before is None or after is None:
        raise ValueError("等待覆盖图像时刻的 Franka 状态；相机必须使用同机 system/global 时间")
    start, end = before["host_stamp_ns"], after["host_stamp_ns"]
    if end - start > 50_000_000:
        raise ValueError("图像附近 Franka 状态间隔超过 50 ms，拒绝插值")
    if before["robot_mode"] != "idle" or after["robot_mode"] != "idle":
        raise ValueError("请松开示教按钮并停稳；当前 Franka 模式：" + after["robot_mode"])
    a, b = flange_pose(before), flange_pose(after)
    if end == start:
        matrix = a
    else:
        fraction = (image_stamp_ns - start) / (end - start)
        rotation = Slerp([0, 1], Rotation.from_matrix(np.array([a[:3, :3], b[:3, :3]])))(fraction).as_matrix()
        matrix = solver.transform(rotation, a[:3, 3] * (1-fraction) + b[:3, 3] * fraction)
    metadata = {"pose_source": "libfranka_read_only", "timestamp_basis": "host_receive_time_not_hardware_sync",
                "before_host_ns": start, "after_host_ns": end,
                "robot_time_ms": [before["robot_time_ms"], after["robot_time_ms"]], "robot_mode": "idle",
                "O_T_EE": before["O_T_EE"], "F_T_EE": before["F_T_EE"]}
    return matrix, metadata


class FrankaReader:
    def __init__(self, robot_ip):
        executable = Path(os.environ.get("HANDEYE_FRANKA_READER", ROOT / "native/build/franka_state_reader"))
        if not executable.is_file():
            raise ValueError("缺少只读状态读取器。请先运行 cmake -S native -B native/build && cmake --build native/build")
        self.states = deque(maxlen=1000)
        self.lock = threading.Lock()
        self.error = ""
        self.error_tail = deque(maxlen=5)
        self.process = subprocess.Popen([str(executable), robot_ip], stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True, bufsize=1)
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.stderr_thread = threading.Thread(target=self._stderr, daemon=True)
        self.thread.start()
        self.stderr_thread.start()

    def _stderr(self):
        for line in self.process.stderr:
            self.error_tail.append(line.strip())

    def _read(self):
        try:
            for line in self.process.stdout:
                state = json.loads(line)
                flange_pose(state)
                with self.lock:
                    if self.states and (state["host_stamp_ns"] <= self.states[-1]["host_stamp_ns"]
                                        or state["robot_time_ms"] <= self.states[-1]["robot_time_ms"]):
                        self.states.clear()
                    self.states.append(state)
        except Exception as exc:
            self.error = str(exc)

    def at(self, stamp):
        if self.error:
            raise ValueError("Franka 状态解码失败：" + self.error)
        if self.process.poll() is not None:
            raise ValueError("Franka 只读连接已退出：" + " ".join(self.error_tail))
        with self.lock:
            states = list(self.states)
        if not states or abs(time.time_ns() - states[-1]["host_stamp_ns"]) > 500_000_000:
            raise ValueError("等待新鲜 Franka 只读状态；检查 FCI、网络和连接占用")
        return interpolate_pose(states, stamp)

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1)
        self.thread.join(timeout=1)
        self.stderr_thread.join(timeout=1)
        self.process.stdout.close()
        self.process.stderr.close()
