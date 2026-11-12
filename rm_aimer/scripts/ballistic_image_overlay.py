#!/usr/bin/env python3
"""Display stamp-matched ballistic diagnostics on the detector's camera preview."""

import argparse
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import grp
import math
import os
from pathlib import Path
import pwd
import shlex
import signal
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def ballistic_points(data):
    """Return the ideal trajectory and straight compensated launch ray endpoint."""
    if data.get("schema") != 1 or not data.get("valid") or not data.get("ballistics_enabled"):
        raise ValueError("no valid ballistic solution")
    muzzle = np.asarray(data["muzzle_position_gravity_m"], dtype=float)
    target = np.asarray(data["target_position_gravity_m"], dtype=float)
    relative = np.asarray(data["relative_target_gravity_m"], dtype=float)
    speed, gravity, flight, pitch = [float(data[key]) for key in (
        "projectile_speed_mps", "gravity_mps2", "flight_time_s", "ballistic_pitch_gravity_rad")]
    if any(point.shape != (3,) or not np.isfinite(point).all()
           for point in (muzzle, target, relative)):
        raise ValueError("invalid ballistic positions")
    if not all(math.isfinite(x) for x in (speed, gravity, flight, pitch)) or min(speed, gravity, flight) <= 0:
        raise ValueError("invalid ballistic parameters")
    horizontal = np.linalg.norm(relative[:2])
    if horizontal <= 0:
        raise ValueError("invalid horizontal distance")
    velocity = speed * np.array([
        math.cos(pitch) * relative[0] / horizontal,
        math.cos(pitch) * relative[1] / horizontal, math.sin(pitch)])
    times = np.linspace(0, flight, 60)
    trajectory = muzzle + times[:, None] * velocity
    trajectory[:, 2] -= 0.5 * gravity * times**2
    if not np.allclose(target, trajectory[-1], atol=0.01, rtol=0):
        raise ValueError("trajectory endpoint differs from solver target")
    return trajectory, muzzle + velocity * flight


def project_points(points, transform, info, width, height):
    """Transform gravity-frame points and project onto the resized raw debug image."""
    if info.width <= 0 or info.height <= 0 or not info.header.frame_id:
        raise ValueError("missing camera calibration")
    sx, sy = width / info.width, height / info.height
    if abs(sx - sy) > 1e-3 or info.binning_x > 1 or info.binning_y > 1 or info.roi.width or info.roi.height:
        raise ValueError("camera calibration/image aspect or ROI mismatch")
    matrix = np.asarray(info.k, dtype=float).reshape(3, 3).copy()
    distortion = np.asarray(info.d, dtype=float)
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or min(matrix[0, 0], matrix[1, 1]) <= 0:
        raise ValueError("invalid camera calibration")
    matrix[0] *= sx
    matrix[1] *= sy
    q = transform.rotation
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid camera transform")
    x, y, z, w = quat / norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    t = transform.translation
    optical = np.asarray(points, dtype=float) @ rotation.T + [t.x, t.y, t.z]
    pixels = np.full((len(optical), 2), np.nan)
    visible = np.isfinite(optical).all(axis=1) & (optical[:, 2] > 0.03)
    if visible.any():
        objects = optical[visible].reshape(-1, 1, 3)
        if info.distortion_model == "equidistant" and distortion.size == 4:
            uv, _ = cv2.fisheye.projectPoints(objects, np.zeros(3), np.zeros(3), matrix, distortion)
        elif info.distortion_model in ("plumb_bob", "rational_polynomial", ""):
            uv, _ = cv2.projectPoints(objects, np.zeros(3), np.zeros(3), matrix, distortion)
        else:
            raise ValueError("unsupported camera distortion model")
        pixels[visible] = uv.reshape(-1, 2)
    return pixels


def matching_solution(data, stamp, now_ns, max_age_ns=500_000_000):
    return bool(data and data.get("valid") and data.get("ballistics_enabled")
                and data.get("source_stamp_ns") == stamp and stamp > 0
                and 0 <= now_ns - stamp <= max_age_ns)


PAGE = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>实时弹道预览</title><style>
body{background:#10141c;color:#e5eaf2;font:16px system-ui;margin:24px auto;max-width:1100px;padding:0 20px}
h1{font-size:24px;margin:0 0 12px}img{width:100%;max-height:76vh;object-fit:contain;background:#000}
.legend{display:flex;gap:24px;flex-wrap:wrap;margin:12px 0}.hit{color:#00ffff}.aim{color:#ff70ff}.curve{color:#ffe000}
p{color:#aab8cd}button,a{color:#c2dcff;background:#26344a;border:0;border-radius:5px;padding:7px 14px;text-decoration:none;cursor:pointer}
#status{min-height:28px}#metrics{font-variant-numeric:tabular-nums}</style>
<h1>相机实时画面 · 弹道解算预览</h1>
<div class="legend"><span class="hit">● 青色圆点：按解算方向发射后的预测命中位置</span>
<span class="aim">＋ 紫色十字：弹道补偿瞄准位置</span><span class="curve">━ 黄色：理想弹道</span></div>
<div id="status">正在连接…</div><img id="view" alt="等待相机画面"><p id="metrics"></p>
<p id="speed-note">等待弹速来源。当前相机外参待复核，标记表示模型预测。</p>
<a href="/frame.jpg" download="ballistic_preview.jpg">保存当前画面</a>
<button onclick="fetch('/stop',{method:'POST'}).then(()=>{running=false;view.hidden=true;statusBox.textContent='预览已停止，相机连接已释放'})">停止预览并释放相机</button>
<script>
const view=document.getElementById('view'),statusBox=document.getElementById('status'),metrics=document.getElementById('metrics');let running=true;
const reasons={waiting_image:'等待相机图像',waiting_solution:'等待匹配当前图像的解算结果',not_tracking:'未锁定目标',target_timeout:'目标数据超时',stale_image:'图像已过期',waiting_calibration:'等待相机标定',waiting_tf:'等待图像时间戳处的坐标变换'};
async function tick(){if(!running)return;try{const s=await(await fetch('/status.json',{cache:'no-store'})).json();
const stale=s.render_age_s>0.8||s.image_age_s>0.8;view.hidden=stale;
statusBox.textContent=stale?'等待新鲜相机画面…':s.valid?(s.hit_in_view?'弹道解算有效 · 已标记预测击打点':'弹道解算有效 · 预测击打点在画面外'):(reasons[s.reason]||s.reason);
statusBox.style.color=s.valid&&!stale?'#74e3ad':'#ffcb75';
metrics.textContent=s.valid&&!stale?`距离 ${s.distance_m.toFixed(2)} m · 飞行时间 ${(s.flight_time_s*1000).toFixed(1)} ms · 重力补偿 ${s.compensation_deg.toFixed(2)}° · 弹速 ${s.speed_mps.toFixed(1)} m/s · 图像年龄 ${(s.image_age_s*1000).toFixed(0)} ms`:'';
if(s.valid&&!stale){document.getElementById('speed-note').textContent=s.speed_source==='impact_fit'
?`弹速 ${s.speed_mps.toFixed(2)} m/s 来自实测落点拟合（档位 ${s.calibration_gear}），属于模型等效速度。相机外参仍需复核。`
:`弹速 ${s.speed_mps.toFixed(2)} m/s 为配置估算值，尚未使用真实落点拟合。相机外参仍需复核。`;}
if(!stale)view.src='/frame.jpg?t='+Date.now();
}catch(e){view.hidden=true;statusBox.textContent='预览连接已断开'}setTimeout(tick,100)}tick();
</script></html>"""


class BallisticImageOverlay(Node):
    def __init__(self, output):
        super().__init__("ballistic_image_overlay")
        self.output = output
        self.frames, self.solutions = OrderedDict(), OrderedDict()
        self.info = None
        self.last_solution = {}
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.jpeg = b""
        self.status = {"valid": False, "reason": "waiting_image", "rendered_at": time.monotonic(), "stamp_ns": 0}
        self.last_saved = 0
        self.create_subscription(Image, "/detector/result_img", self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(String, "/aimer/solution", self.on_solution, qos_profile_sensor_data)
        self.create_timer(1 / 15, self.render)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.frames[stamp] = msg
        while len(self.frames) > 16:
            self.frames.popitem(last=False)

    def on_solution(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or data.get("schema") != 1:
                return
            self.last_solution = data
            self.solutions[data.get("source_stamp_ns")] = data
            while len(self.solutions) > 128:
                self.solutions.popitem(last=False)
        except (ValueError, TypeError):
            pass

    def render(self):
        now_ns = self.get_clock().now().nanoseconds
        status = {"valid": False, "reason": "waiting_image", "stamp_ns": 0, "rendered_at": time.monotonic()}
        canvas = np.zeros((720, 960, 3), dtype=np.uint8)
        if self.frames:
            stamp = next(reversed(self.frames))
            if self.last_solution.get("valid"):
                for candidate in reversed(self.frames):
                    if matching_solution(self.solutions.get(candidate), candidate, now_ns):
                        stamp = candidate
                        break
            msg = self.frames[stamp]
            status["stamp_ns"] = stamp
            if msg.encoding != "bgr8" or msg.step < msg.width * 3 or len(msg.data) < msg.step * msg.height:
                status["reason"] = "unsupported preview image encoding"
            else:
                canvas = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
                canvas = canvas[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
                data = self.solutions.get(stamp)
                status["reason"] = self.last_solution.get("reason", "waiting_solution")
                if status["reason"] == "solved":
                    status["reason"] = "waiting_solution"
                if not 0 <= now_ns - stamp <= 500_000_000:
                    status["reason"] = "stale_image"
                elif self.last_solution.get("valid") and matching_solution(data, stamp, now_ns):
                    try:
                        if self.info is None:
                            raise ValueError("waiting_calibration")
                        if msg.header.frame_id != self.info.header.frame_id:
                            raise ValueError("image/calibration frame mismatch")
                        transform = self.buffer.lookup_transform(
                            msg.header.frame_id, data["gravity_frame"], Time(nanoseconds=stamp))
                        trajectory, aim = ballistic_points(data)
                        pixels = project_points(np.vstack((trajectory, aim)), transform.transform,
                                                self.info, msg.width, msg.height)
                        def inside(p):
                            return bool(np.isfinite(p).all() and 0 <= p[0] < msg.width and 0 <= p[1] < msg.height)
                        for a, b in zip(pixels[:-2], pixels[1:-1]):
                            if np.isfinite([a, b]).all() and np.max(np.abs([a, b])) < 1e6:
                                ok, first, second = cv2.clipLine((0, 0, msg.width, msg.height), tuple(a.astype(int)), tuple(b.astype(int)))
                                if ok:
                                    cv2.line(canvas, first, second, (0, 220, 255), 2, cv2.LINE_AA)
                        hit, compensated = pixels[-2], pixels[-1]
                        if inside(hit):
                            point = tuple(hit.astype(int))
                            cv2.circle(canvas, point, 9, (255, 255, 0), 2, cv2.LINE_AA)
                            cv2.circle(canvas, point, 2, (255, 255, 0), -1)
                            cv2.putText(canvas, "PREDICTED HIT", (point[0]+12, min(msg.height-40, point[1]+26)), 0, .5, (255, 255, 0), 2)
                        if inside(compensated):
                            point = tuple(compensated.astype(int))
                            cv2.drawMarker(canvas, point, (255, 0, 255), cv2.MARKER_CROSS, 24, 2)
                            cv2.putText(canvas, "COMPENSATED AIM", (point[0]+14, max(65, point[1]-18)), 0, .5, (255, 0, 255), 2)
                        status.update(valid=True, reason="solved", hit_in_view=inside(hit), aim_in_view=inside(compensated),
                                      distance_m=data["distance_from_muzzle_m"], flight_time_s=data["flight_time_s"],
                                      compensation_deg=math.degrees(data["gravity_compensation_rad"]),
                                      speed_mps=data["projectile_speed_mps"],
                                      speed_source=data.get("projectile_speed_source", "configured"),
                                      calibration_gear=data.get("speed_calibration_gear", ""),
                                      calibration_id=data.get("speed_calibration_id", ""), hit_pixel=hit.tolist() if np.isfinite(hit).all() else None,
                                      aim_pixel=compensated.tolist() if np.isfinite(compensated).all() else None)
                    except TransformException:
                        status["reason"] = "waiting_tf"
                    except (ValueError, KeyError, TypeError, cv2.error) as exc:
                        status["reason"] = str(exc)
        height, width = canvas.shape[:2]
        cv2.rectangle(canvas, (0, 0), (width, 46), (15, 20, 30), -1)
        label = "BALLISTIC PREVIEW" if status["valid"] else "NO VALID MARK: " + status["reason"]
        cv2.putText(canvas, label[:110], (14, 29), 0, .6, (160, 255, 190) if status["valid"] else (80, 200, 255), 1, cv2.LINE_AA)
        cv2.rectangle(canvas, (0, height-30), (width, height), (15, 20, 30), -1)
        source_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(status["stamp_ns"] / 1e9)) if status["stamp_ns"] else "waiting camera"
        source_label = "IMPACT-FIT SPEED / IDEAL MODEL" if status.get("speed_source") == "impact_fit" else "CONFIGURED SPEED / IDEAL MODEL"
        cv2.putText(canvas, source_time + " | " + source_label, (12, height-10), 0, .45, (220, 220, 220), 1)
        ok, encoded = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if not ok:
            return
        jpeg = encoded.tobytes()
        with self.lock:
            self.jpeg, self.status = jpeg, status
        if time.monotonic() - self.last_saved >= 1:
            self.last_saved = time.monotonic()
            for name in (["latest.jpg", "latest_valid.jpg"] if status["valid"] else ["latest.jpg"]):
                temporary = self.output / (name + ".tmp")
                temporary.write_bytes(jpeg)
                temporary.replace(self.output / name)
            (self.output / "latest.json").write_text(json.dumps(status, allow_nan=False, indent=2) + "\n")


def start_pipeline(root, output, speed_calibration=None):
    camera = ["ros2", "launch", "hik_camera", "hik_camera.launch.py", "publish_bayer:=true", f"params_file:={root}/hik_camera/config/camera_params.yaml"]
    vision = ["ros2", "launch", "auto_aim_bringup", "yolo_auto_aim.launch.py",
              f"yolo_params:={root}/auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml",
              f"tracker_params:={root}/auto_aim_bringup/params/tracker_params.yaml", "publish_debug:=true", "start_static_tf:=false"]
    commands = {
        "feedback": ["/usr/bin/python3", str(root / "rm_serial_driver/scripts/serial_bridge_node.py"), "--ros-args", "--params-file", str(root / "rm_serial_driver/config/serial_bridge.yaml"), "-p", "receive_only:=true"],
        "tf": ["/usr/bin/python3", str(root / "rm_serial_driver/scripts/gimbal_tf_broadcaster.py"), "--ros-args", "--params-file", str(root / "rm_serial_driver/config/gimbal_tf_provisional_handeye.yaml")],
        "camera": camera, "vision": vision,
        "aimer": ["/usr/bin/python3", str(root / "rm_aimer/scripts/basic_aimer_node.py"), "--ros-args", "--params-file", str(root / "rm_aimer/config/basic_aimer.yaml"), "--params-file", str(root / "rm_aimer/config/controller_preview.yaml"), "-p", "publish_diagnostics:=true", "-p", "output_topic:=/aimer/visualization_command"],
    }
    if speed_calibration is not None:
        commands["aimer"].extend(["-p", f"speed_calibration_file:={speed_calibration.resolve()}"])
    # A desktop session may predate membership in dialout. Use that existing
    # membership for the feedback reader without changing device permissions.
    port = "/dev/ttyACM0"
    if os.path.exists(port) and not os.access(port, os.R_OK | os.W_OK):
        user = pwd.getpwuid(os.getuid()).pw_name
        if user in grp.getgrnam("dialout").gr_mem:
            # sg is setgid and clears LD_LIBRARY_PATH; restore ROS inside it.
            shell = ("source /opt/ros/humble/setup.bash\nsource "
                     + shlex.quote(str(root / "install/setup.bash"))
                     + "\nexec " + shlex.join(commands["feedback"]))
            commands["feedback"] = ["sg", "dialout", "-c", shlex.join(["bash", "-c", shell])]
    processes = []
    try:
        for name, command in commands.items():
            with (output / (name + ".log")).open("w") as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
    except BaseException:
        stop_pipeline(processes)
        raise
    (output / "processes.json").write_text(json.dumps({name: process.pid for name, process in zip(commands, processes)}, indent=2))
    return processes


def stop_pipeline(processes):
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    deadline = time.monotonic() + 5
    for process in processes:
        try:
            process.wait(timeout=max(0.01, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start-pipeline", action="store_true")
    parser.add_argument("--speed-calibration", type=Path)

    args = parser.parse_args()
    if args.speed_calibration is not None:
        if not args.start_pipeline:
            parser.error("--speed-calibration requires --start-pipeline; existing nodes own their speed inputs")
        from impact_speed_calibration import load_speed_calibration
        load_speed_calibration(args.speed_calibration, 9.81)
    args.output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(1)
    rclpy.init(args=[])
    node = BallisticImageOverlay(args.output)
    stopping = threading.Event()
    processes = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *unused):
            pass

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                content, kind = PAGE.encode(), "text/html; charset=utf-8"
            elif path in ("/frame.jpg", "/status.json"):
                with node.lock:
                    content, state = node.jpeg, dict(node.status)
                if path == "/status.json":
                    state["render_age_s"] = time.monotonic() - state.pop("rendered_at")
                    state["image_age_s"] = max(0, (node.get_clock().now().nanoseconds - state["stamp_ns"]) / 1e9)
                    content = json.dumps(state, allow_nan=False).encode()
                kind = "image/jpeg" if path == "/frame.jpg" else "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if self.path != "/stop" or self.headers.get("Origin", "") not in ("", f"http://127.0.0.1:{args.port}"):
                self.send_error(403)
                return
            self.send_response(200)
            self.end_headers()
            stopping.set()

    server = None
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
        if args.start_pipeline:
            until = time.monotonic() + 1.5
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.1)
            existing = set(node.get_node_names()) & {"hik_camera", "yolo_armor_detector", "armor_tracker", "aimer", "serial_bridge", "gimbal_tf"}
            if existing:
                raise RuntimeError("Pipeline nodes already running: " + ", ".join(sorted(existing)))
            processes = start_pipeline(Path(__file__).resolve().parents[2], args.output, args.speed_calibration)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        node.get_logger().info(f"Camera ballistic preview: http://127.0.0.1:{args.port}")
        while rclpy.ok() and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError(f"Pipeline process {process.pid} exited: {process.returncode}; see {args.output}")
    except KeyboardInterrupt:
        pass
    finally:
        stop_pipeline(processes)
        if server is not None:
            server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
