"""Synthetic ChArUco input for offline tests only; never used by the web server."""
from __future__ import annotations

import time
from types import SimpleNamespace
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from handeye_web.core import solver


class DemoSource:
    """Rendered ChArUco -> real detector -> the same solver, no robot/network IO."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.detector = solver.BoardDetector(cfg["board"])
        self.board = self.detector.charuco
        self.board_image = (self.board.generateImage((1120, 720)) if hasattr(self.board, "generateImage")
                            else self.board.draw((1120, 720)))
        self.k = np.array([[850., 0, 480], [0, 850., 300], [0, 0, 1]])
        self.info = SimpleNamespace(width=960, height=600, k=self.k.ravel().tolist(), d=[0.] * 5,
                                    distortion_model="plumb_bob", binning_x=0, binning_y=0,
                                    roi=SimpleNamespace(x_offset=0, y_offset=0))
        self.x = solver.transform(Rotation.from_euler("xyz", [165, 4, 80], degrees=True).as_matrix(), [.45, .05, .9])
        self.y = solver.transform(Rotation.from_euler("xyz", [5, 8, -3], degrees=True).as_matrix(), [.02, -.01, .12])
        self.index = 0
        self.cache = None

    def advance(self):
        self.index += 1
        self.cache = None

    def next(self):
        if self.cache is None:
            # Independent validation starts at a different deterministic pose sequence.
            i = self.index
            rotation = Rotation.from_euler("xyz", [22*np.sin(i*1.7)+7, 28*np.cos(i*1.3), 16*np.sin(i*.8)], degrees=True)
            b = solver.transform(rotation.as_matrix(), [-.14+.025*np.sin(i), -.09+.02*np.cos(i*.6), .64+.05*np.sin(i*.7)])
            w = self.cfg["board"]["columns"] * self.cfg["board"]["square_length_m"]
            h = self.cfg["board"]["rows"] * self.cfg["board"]["square_length_m"]
            points = np.array([[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0]], dtype=float)
            pixels = cv2.projectPoints(points, rotation.as_rotvec(), b[:3, 3], self.k, np.zeros(5))[0].reshape(4, 2).astype(np.float32)
            height, width = self.board_image.shape
            mapping = cv2.getPerspectiveTransform(np.float32([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]]), pixels)
            image = cv2.warpPerspective(self.board_image, mapping, (960, 600), borderValue=220)
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            canvas, row, status = self.detector.detect(image, self.info)
            if row is not None:
                row.update(T_base_tool=(self.x @ b @ np.linalg.inv(self.y)).tolist(),
                           camera={"frame_id": "demo_head_color_optical_frame", "width": 960, "height": 600,
                                   "k": self.k.ravel().tolist(), "d": [0.] * 5, "distortion_model": "plumb_bob"})
            cv2.putText(canvas, "SIMULATION / NO HARDWARE", (24, 38), cv2.FONT_HERSHEY_SIMPLEX, .75, (20, 90, 210), 2)
            self.cache = (canvas, row, "仿真图像 · " + status)
        canvas, row, status = self.cache
        if row:
            row = {**row, "image_stamp_ns": time.time_ns()}
        return canvas, row, status

    def close(self):
        pass
