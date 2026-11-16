"""Local browser app. No arm/hand command publishers or SDK control connections."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np
from .core import ROOT, Session, default_config, validate_config
from .sources import RosSource


class App:
    def __init__(self, output):
        self.output = Path(output)
        self.lock = threading.RLock()
        self.session = None
        self.source = None
        self.running = True
        self.connected = False
        self.last_error = ""
        self.generation = 0
        self.token = secrets.token_urlsafe(32)
        self.worker = threading.Thread(target=self.loop, daemon=True)
        self.worker.start()

    def session_path(self, sid):
        if not isinstance(sid, str) or not re.fullmatch(r"[0-9]{8}_[0-9]{6}_[a-f0-9]{8}", sid):
            raise ValueError("无效会话编号")
        matches = list(self.output.glob(f"*/*/{sid}/samples.json"))
        if len(matches) != 1:
            raise ValueError("找不到会话")
        return matches[0].parent

    def sessions(self):
        rows = []
        for path in sorted(self.output.glob("live/*/*/samples.json"), reverse=True):
            try:
                data = json.loads(path.read_text())
                if data["session"]["mode"] != "live":
                    continue
                rows.append({**data["session"], "count": len(data["samples"]),
                             "has_result": (path.parent / "calibration.json").exists()})
            except (ValueError, KeyError, OSError):
                continue
        return rows[:100]

    def hardware_session(self, sid):
        session = Session.load(self.session_path(sid))
        if session.data["session"]["mode"] != "live":
            raise ValueError("工作台仅支持硬件只读，无法打开仿真会话")
        return session

    def state(self):
        with self.lock:
            s = self.session
            ready, reason = s.readiness() if s else (False, "先创建采集会话")
            row = s.row if s else None
            return {"connected": self.connected, "error": self.last_error,
                    "hardware_read_only": True,
                    "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
                    "sessions": self.sessions(), "defaults": default_config(),
                    "session": s.data["session"] if s else None, "config": s.data["config"] if s else None,
                    "count": len(s.data["samples"]) if s else 0, "ready": ready and self.connected,
                    "reason": reason, "status": s.status if s else "等待创建会话",
                    "result": s.result if s else None,
                    "output": str(s.path) if s else str(self.output),
                    "observation": {k: row[k] for k in ("reprojection_rmse_px", "visible_ids", "T_base_tool") if k in row} if row else None,
                    "samples": [{"index": r["index"], "error": r["reprojection_rmse_px"],
                                 "xyz": np.array(r["T_base_tool"])[:3, 3].tolist()} for r in s.data["samples"]] if s else []}

    def action(self, body):
        action = body.get("action")
        with self.lock:
            if body.get("mode", "live") != "live":
                raise ValueError("工作台仅支持硬件只读，不支持切换运行模式")
            if action in ("new", "resume"):
                if action == "resume":
                    session = self.hardware_session(body["id"])
                else:
                    cfg = validate_config(body.get("config") or default_config(body.get("side", "left"), body.get("profile", "camera")))
                    purpose = body.get("purpose", "fit")
                    calibration_id = body.get("calibration_id")
                    if purpose == "validation":
                        previous = self.hardware_session(calibration_id)
                        if not previous.result or not previous.result.get("quality_pass"):
                            raise ValueError("先选择已通过拟合与留出检查的标定结果")
                        if previous.data["session"]["side"] != body["side"]:
                            raise ValueError("验证的侧别必须和标定一致")
                        cfg = previous.data["config"]
                    session = Session(self.output, cfg, "live", body["side"], purpose, calibration_id)
                    if purpose == "validation":
                        from .core import solver
                        solver.atomic_json(session.path / "calibration_reference.json", previous.result)
                self.connected = False
                self.source = None
                self.generation += 1
                self.last_error = ""
                self.session = session
                return {"message": "会话已保存；点击连接开始预览"}
            if self.session is None:
                raise ValueError("先创建采集会话")
            if action == "connect":
                self.connected = True
                self.source = None
                self.last_error = ""
                self.generation += 1
                self.session.history.clear()
                self.session.row = None
                return {"message": "正在连接数据源"}
            if action == "disconnect":
                self.connected = False
                self.source = None
                self.generation += 1
                self.session.history.clear()
                self.session.row = None
                return {"message": "已断开预览，数据已保存在磁盘"}
            if action == "capture":
                if not self.connected:
                    raise ValueError("先连接数据源")
                return {"message": f"已保存样本 {self.session.capture()}"}
            if action == "solve":
                self.session.solve()
                return {"message": "求解完成，请查看拟合与留出指标"}
            if action == "validate":
                calibration = json.loads((self.session.path / "calibration_reference.json").read_text())
                self.session.validate(calibration)
                return {"message": "独立验证完成；没有重新拟合"}
            raise ValueError("未知操作")

    def loop(self):
        source = None
        generation = -1
        while self.running:
            with self.lock:
                target = self.session if self.connected else None
                current_generation = self.generation
            if generation != current_generation:
                if source:
                    source.close()
                source = None
                generation = current_generation
                if target:
                    try:
                        source = RosSource(target.data["config"])
                        with self.lock:
                            if generation == self.generation:
                                self.source = source
                    except Exception as exc:
                        with self.lock:
                            if generation == self.generation:
                                self.last_error = f"数据源启动失败：{exc}。请在已 source ROS 的环境启动。"
                                self.connected = False
                                self.source = None
            if target and source:
                try:
                    canvas, row, status = source.next()
                    with self.lock:
                        if generation == self.generation and target is self.session and self.connected:
                            target.observe(canvas, row, status)
                            self.last_error = ""
                except Exception as exc:
                    with self.lock:
                        if generation == self.generation and target is self.session:
                            target.observe(target.canvas, None, str(exc))
                            self.last_error = str(exc)
            time.sleep(.055)
        if source:
            source.close()

    def close(self):
        self.running = False
        self.worker.join(timeout=3)


def handler_class(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            if args and str(args[0]).startswith("POST"):
                super().log_message(format, *args)

        def send(self, data, content_type="application/json; charset=utf-8", status=200):
            if not isinstance(data, bytes):
                data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; frame-ancestors 'self'")
            self.end_headers()
            self.wfile.write(data)

        def valid_host(self):
            return self.headers.get("Host", "") in (f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}")

        def do_GET(self):
            if not self.valid_host():
                return self.send({"error": "Use localhost or SSH local forwarding"}, status=403)
            path = urlparse(self.path).path
            try:
                if path == "/api/state":
                    return self.send(app.state())
                if path == "/api/token":
                    return self.send({"token": app.token})
                if path == "/api/frame":
                    with app.lock:
                        canvas = app.session.canvas.copy() if app.session and app.session.canvas is not None else np.full((600, 960, 3), (249, 244, 243), np.uint8)
                    encoded, data = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if not encoded:
                        raise ValueError("预览编码失败")
                    return self.send(data.tobytes(), "image/jpeg")
                if path in ("/api/download", "/api/samples"):
                    with app.lock:
                        if app.session is None:
                            raise ValueError("没有会话")
                        data = app.session.result if path.endswith("download") else app.session.data
                        if data is None:
                            raise ValueError("尚未生成结果")
                        return self.send(data)
                files = {"/": "index.html", "/app.js": "app.js", "/style.css": "style.css"}
                if path in files:
                    content_type = {"/": "text/html; charset=utf-8", "/app.js": "text/javascript; charset=utf-8", "/style.css": "text/css; charset=utf-8"}[path]
                    return self.send((ROOT / "handeye_web/static" / files[path]).read_bytes(), content_type)
                self.send({"error": "Not found"}, status=404)
            except (ValueError, OSError, KeyError) as exc:
                self.send({"error": str(exc)}, status=400)

        def do_POST(self):
            origin = self.headers.get("Origin")
            if (not self.valid_host() or (origin and origin not in (f"http://127.0.0.1:{self.server.server_port}", f"http://localhost:{self.server.server_port}"))
                    or self.headers.get("X-Handeye-Token") != app.token):
                return self.send({"error": "同源校验失败，请刷新本机页面"}, status=403)
            if self.path != "/api/action":
                return self.send({"error": "Not found"}, status=404)
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 64_000:
                    raise ValueError("请求大小无效")
                result = app.action(json.loads(self.rfile.read(length)))
                self.send(result)
            except Exception as exc:
                self.send({"error": str(exc)}, status=400)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "runs/franka_handeye")
    args = parser.parse_args()
    app = App(args.output_dir.expanduser().resolve())
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_class(app))
    server.daemon_threads = True
    print(f"Franka hand-eye: http://127.0.0.1:{args.port} (hardware read-only)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        app.close()


if __name__ == "__main__":
    main()
