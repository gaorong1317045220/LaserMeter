# SPDX-License-Identifier: MIT
"""PC-side receiver and web server for the laser room-measurement device.

The ESP32 never serves the product UI in normal operation.  It connects to the
PC hotspot as a station and pushes framed telemetry, JPEGs and completed scan
files to DEVICE_PORT.  Browsers connect only to this process on HTTP_PORT.
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import re
import socket
import socketserver
import struct
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request
import zlib
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import BinaryIO

try:
    from pc_app.scan_analysis import ScanFormatError, analyze_scan
except ModuleNotFoundError:  # Direct execution: python pc_app/server.py
    from scan_analysis import ScanFormatError, analyze_scan


MAGIC = b"LMP1"
HEADER = struct.Struct("!4sBBHI")
TYPE_HELLO = 1
TYPE_TELEMETRY = 2
TYPE_JPEG = 3
TYPE_FILE_BEGIN = 4
TYPE_FILE_CHUNK = 5
TYPE_FILE_END = 6
TYPE_EVENT = 7
MAX_PACKET = 2 * 1024 * 1024
MAX_BIND_BODY = 4096


def _safe_print(message: str) -> None:
    """Keep request handling alive when a detached console closes stdout."""
    try:
        print(message, flush=True)
    except (BrokenPipeError, OSError):
        pass


def _safe_component(value: object, fallback: str) -> str:
    text = str(value or "").strip()
    text = re.sub(r"[^0-9A-Za-z._-]+", "_", text).strip("._")
    return text[:96] or fallback


def _open_browser(url: str) -> None:
    """打开默认浏览器;失败时静默(无头环境/被策略拦截)。"""
    try:
        import webbrowser
        webbrowser.open(url)
    except OSError:
        pass


# ---------- 照片激光点标注:内参 + 外参 + 测量记录 ----------
# 与固件一致的基准偏移(毫米):REAR / FRONT / TRIPOD
# Firmware single-point reference corrections (REAR/FRONT/TRIPOD), mm.
# The tripod value is the current FUSION calibration's emitter-to-axis Y
# component, with the sign reversed and rounded to the firmware's integer mm.
_REFERENCE_OFFSET_MM = {0: 126.5, 1: -2.0, 2: 18.0}
_REFERENCE_NAMES = {0: "后基准", 1: "前基准", 2: "三脚架"}
# 激光发射点相对 IMU(米),与 fusion_config.h 一致
_LASER_ORIGIN_M = (0.0145, 0.0486527, -0.016256)
_annotation_calib_cache: dict[str, object] = {"intrinsics": None, "extrinsics": None}
_annotation_records: dict[str, tuple[float, int]] = {}  # 照片名 -> (原始距离米, reference)
_annotation_records_mtime = 0.0
_annotation_records_fetched = 0.0                   # 上次从设备拉取的时间戳


def _parse_records_csv(path: Path) -> dict[str, tuple[float, int]]:
    """解析测量记录 CSV,返回 {照片文件名: (原始激光距离米, reference)}。"""
    rows: dict[str, tuple[float, int]] = {}
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return rows
    for line in text.splitlines():
        fields = [f.strip() for f in line.split(",")]
        if len(fields) < 10 or fields[0] != "SAVE" or fields[9] in ("-", ""):
            continue
        try:
            recorded_mm = float(fields[3])
            reference = int(fields[4])
        except ValueError:
            continue
        raw_mm = recorded_mm - _REFERENCE_OFFSET_MM.get(reference, 0.0)
        rows[Path(fields[9]).name] = (raw_mm / 1000.0, reference)
    return rows


def _load_annotation_intrinsics() -> dict | None:
    """读取相机内参(json),带缓存;缺失返回 None。"""
    if _annotation_calib_cache["intrinsics"] is None:
        calib_file = _calib_root() / "camera_intrinsics.json"
        if calib_file.is_file():
            try:
                data = json.loads(calib_file.read_text(encoding="utf-8"))
                _annotation_calib_cache["intrinsics"] = {
                    "K": data["camera_matrix"],
                    "dist": data["dist_coeffs"],
                    "size": data["image_size"],
                }
            except (OSError, ValueError, KeyError):
                _annotation_calib_cache["intrinsics"] = False
    return _annotation_calib_cache["intrinsics"] or None


def _load_annotation_extrinsics() -> dict | None:
    """读取激光-相机外参(json),带缓存;缺失返回 None。"""
    if _annotation_calib_cache["extrinsics"] is None:
        calib_file = _calib_root() / "laser_camera_extrinsics.json"
        if calib_file.is_file():
            try:
                data = json.loads(calib_file.read_text(encoding="utf-8"))
                _annotation_calib_cache["extrinsics"] = {
                    "dir_cam": data["laser_dir_cam"],
                    "p0_cam": data["p0_cam_m"],
                }
            except (OSError, ValueError, KeyError):
                _annotation_calib_cache["extrinsics"] = False
    return _annotation_calib_cache["extrinsics"] or None


def _project_laser_spot(range_m: float, width: int, height: int) -> tuple[float, float] | None:
    """激光点 -> 图像像素(带畸变)。内参按实际照片尺寸缩放。"""
    intrinsics = _load_annotation_intrinsics()
    extrinsics = _load_annotation_extrinsics()
    if not intrinsics or not extrinsics:
        return None
    K = intrinsics["K"]
    dist = intrinsics["dist"]
    iw, ih = intrinsics["size"]
    sx, sy = width / iw, height / ih
    # 机身系激光点(米)
    px, py, pz = _LASER_ORIGIN_M
    dx, dy, dz = (0.0, 1.0, 0.0)
    body = (px + dx * range_m, py + dy * range_m, pz + dz * range_m)
    # 相机系:P_cam = p0_cam + dir_cam * range
    d0, d1, d2 = extrinsics["dir_cam"]
    o0, o1, o2 = extrinsics["p0_cam"]
    cam_x = o0 + d0 * range_m
    cam_y = o1 + d1 * range_m
    cam_z = o2 + d2 * range_m
    if cam_z <= 1e-6:
        return None
    xn, yn = cam_x / cam_z, cam_y / cam_z
    r2 = xn * xn + yn * yn
    k1, k2, p1, p2, k3 = dist
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
    xd = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    yd = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
    u = K[0][0] * sx * xd + K[0][2] * sx
    v = K[1][1] * sy * yd + K[1][2] * sy
    return u, v


def _calib_root() -> Path:
    """标定数据目录:打包版在 exe 旁,开发版在工程根。"""
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent / "calibration_capture"
    return Path(__file__).resolve().parents[1] / "calibration_capture"


def _recv_exact(sock: socket.socket, count: int) -> bytes:
    chunks: list[bytes] = []
    remaining = count
    while remaining:
        block = sock.recv(remaining)
        if not block:
            raise ConnectionError("device connection closed")
        chunks.append(block)
        remaining -= len(block)
    return b"".join(chunks)


@dataclass
class IncomingFile:
    final_path: Path
    partial_path: Path
    stream: BinaryIO
    expected_size: int
    expected_crc32: int
    received: int = 0
    crc32: int = 0


@dataclass
class DeviceHub:
    data_dir: Path
    lock: threading.RLock = field(default_factory=threading.RLock)
    frame_ready: threading.Condition = field(init=False)
    device_connected: bool = False
    device_id: str = ""
    peer: str = ""
    hello: dict = field(default_factory=dict)
    telemetry: dict = field(default_factory=dict)
    latest_jpeg: bytes = b""
    frame_sequence: int = 0
    last_seen: float = 0.0
    last_frame: float = 0.0
    last_event: dict = field(default_factory=dict)
    last_file: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.frame_ready = threading.Condition(self.lock)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        existing = [path for path in self.data_dir.rglob("*.csv")
                    if path.is_file() and not path.name.endswith(".part")]
        if existing:
            latest = max(existing, key=lambda path: path.stat().st_mtime_ns)
            crc32 = 0
            with latest.open("rb") as source:
                for chunk in iter(lambda: source.read(65536), b""):
                    crc32 = zlib.crc32(chunk, crc32)
            stat = latest.stat()
            self.last_file = {
                "name": latest.name,
                "path": str(latest.relative_to(self.data_dir)).replace("\\", "/"),
                "size": stat.st_size,
                "crc32": f"{crc32 & 0xffffffff:08x}",
                "received_at": int(stat.st_mtime * 1000),
            }

    def connected(self, device_id: str, peer: str, hello: dict) -> None:
        with self.lock:
            self.device_connected = True
            self.device_id = _safe_component(device_id, "laser_meter")
            self.peer = peer
            self.hello = dict(hello)
            self.last_event = {}
            self.last_seen = time.time()

    def disconnected(self, peer: str) -> None:
        with self.lock:
            if self.peer == peer:
                self.device_connected = False
            self.frame_ready.notify_all()

    def set_telemetry(self, payload: dict) -> None:
        with self.lock:
            self.telemetry = payload
            self.last_seen = time.time()

    def set_frame(self, jpeg: bytes) -> None:
        if len(jpeg) < 4 or not jpeg.startswith(b"\xff\xd8"):
            raise ValueError("invalid JPEG packet")
        with self.lock:
            self.latest_jpeg = jpeg
            self.frame_sequence += 1
            self.last_frame = self.last_seen = time.time()
            self.frame_ready.notify_all()

    def set_event(self, payload: dict) -> None:
        with self.lock:
            self.last_event = payload
            self.last_seen = time.time()

    def set_file(self, path: Path, size: int, crc32: int) -> None:
        with self.lock:
            self.last_file = {
                "name": path.name,
                "path": str(path.relative_to(self.data_dir)).replace("\\", "/"),
                "size": size,
                "crc32": f"{crc32:08x}",
                "received_at": int(time.time() * 1000),
            }
            self.last_seen = time.time()

    def snapshot(self) -> dict:
        with self.lock:
            now = time.time()
            peer_host = self.peer.rsplit(":", 1)[0] if ":" in self.peer else ""
            return {
                "ok": True,
                "connected": self.device_connected and now - self.last_seen < 15.0,
                "device_id": self.device_id,
                "peer": self.peer,
                "device_ip": peer_host,   # 设备 STA 地址,供文件管理代理使用
                "last_seen_ms": int(self.last_seen * 1000) if self.last_seen else 0,
                "frame_age_ms": int((now - self.last_frame) * 1000) if self.last_frame else None,
                "frame_sequence": self.frame_sequence,
                "telemetry": dict(self.telemetry),
                "hello": dict(self.hello),
                "last_event": dict(self.last_event),
                "last_file": dict(self.last_file),
            }


class DeviceStreamHandler(socketserver.BaseRequestHandler):
    hub: DeviceHub

    def handle(self) -> None:
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        device_id = "laser_meter"
        incoming: IncomingFile | None = None
        self.request.settimeout(12.0)
        try:
            while True:
                raw = _recv_exact(self.request, HEADER.size)
                magic, kind, _flags, _reserved, length = HEADER.unpack(raw)
                if magic != MAGIC:
                    raise ValueError("invalid packet magic")
                if length > MAX_PACKET:
                    raise ValueError(f"packet too large: {length}")
                body = _recv_exact(self.request, length)
                if kind == TYPE_HELLO:
                    payload = json.loads(body.decode("utf-8"))
                    device_id = _safe_component(payload.get("device_id"), "laser_meter")
                    self.hub.connected(device_id, peer, payload)
                    # 随 HELLO 应答下发 PC 日历时间与 UTC 时区偏移(分钟),设备据此同步。
                    # 注意:固件用 strstr("\"ok\":true") 精确匹配,必须使用紧凑分隔符。
                    tz_min = -(time.timezone // 60)
                    reply = json.dumps({"ok": True, "type": "hello",
                                        "utc_ms": int(time.time() * 1000),
                                        "tz_min": tz_min}, separators=(",", ":")).encode()
                    self.request.sendall(reply + b"\n")
                elif kind == TYPE_TELEMETRY:
                    self.hub.set_telemetry(json.loads(body.decode("utf-8")))
                elif kind == TYPE_JPEG:
                    self.hub.set_frame(body)
                elif kind == TYPE_EVENT:
                    self.hub.set_event(json.loads(body.decode("utf-8")))
                elif kind == TYPE_FILE_BEGIN:
                    if incoming is not None:
                        raise ValueError("nested file transfer")
                    meta = json.loads(body.decode("utf-8"))
                    name = _safe_component(Path(str(meta.get("name", "scan.csv"))).name, "scan.csv")
                    expected_size = int(meta.get("size", -1))
                    expected_crc = int(str(meta.get("crc32", "0")), 16)
                    if expected_size < 0 or expected_size > 64 * 1024 * 1024:
                        raise ValueError("invalid file size")
                    folder = self.hub.data_dir / _safe_component(device_id, "laser_meter")
                    folder.mkdir(parents=True, exist_ok=True)
                    final_path = folder / name
                    partial_path = folder / (name + ".part")
                    stream = partial_path.open("wb")
                    incoming = IncomingFile(final_path, partial_path, stream, expected_size, expected_crc)
                elif kind == TYPE_FILE_CHUNK:
                    if incoming is None:
                        raise ValueError("file chunk without metadata")
                    incoming.stream.write(body)
                    incoming.received += len(body)
                    incoming.crc32 = zlib.crc32(body, incoming.crc32)
                    if incoming.received > incoming.expected_size:
                        raise ValueError("received more bytes than declared")
                elif kind == TYPE_FILE_END:
                    if incoming is None:
                        raise ValueError("file end without metadata")
                    incoming.stream.flush()
                    os.fsync(incoming.stream.fileno())
                    incoming.stream.close()
                    actual_crc = incoming.crc32 & 0xFFFFFFFF
                    valid = (incoming.received == incoming.expected_size and
                             actual_crc == incoming.expected_crc32)
                    if not valid:
                        incoming.partial_path.unlink(missing_ok=True)
                        reply = {"ok": False, "error": "size_or_crc_mismatch"}
                    else:
                        incoming.partial_path.replace(incoming.final_path)
                        self.hub.set_file(incoming.final_path, incoming.received, actual_crc)
                        reply = {"ok": True, "type": "file", "name": incoming.final_path.name,
                                 "size": incoming.received, "crc32": f"{actual_crc:08x}"}
                    incoming = None
                    self.request.sendall((json.dumps(reply, separators=(",", ":")) + "\n").encode())
                else:
                    raise ValueError(f"unknown packet type: {kind}")
        except (ConnectionError, OSError, ValueError, json.JSONDecodeError) as exc:
            self.hub.set_event({"type": "pc_link_error", "message": str(exc),
                                "timestamp_ms": int(time.time() * 1000)})
        finally:
            if incoming is not None:
                try:
                    incoming.stream.close()
                finally:
                    incoming.partial_path.unlink(missing_ok=True)
            self.hub.disconnected(peer)


class DeviceTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class VisionRunner:
    """调用 vision_worker.py(当前或显式指定的 Python 环境)做 AI 检测 + 几何提取。

    结果以侧车文件 <图片>.vision.json 缓存,重复请求零开销;worker 缺失时
    抛出带说明的异常,不影响服务端其它功能。
    """

    IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".webp"}

    def __init__(self, python: str | None = None, worker: Path | None = None,
                 timeout: float = 90.0) -> None:
        # 解析 worker 运行方式,优先级:
        # 1. LASER_VISION_PYTHON 环境变量(显式指定 python.exe 或 vision_worker.exe)
        # 2. PyInstaller 打包场景:exe 同目录的 vision_worker.exe
        # 3. 开发环境使用当前 Python + vision_worker.py
        self.worker = worker  # worker 为 None 时使用打包进 exe 的 worker
        self.python = python or os.environ.get("LASER_VISION_PYTHON")
        if not self.python and getattr(sys, "frozen", False):
            for candidate in (
                Path(sys.executable).with_name("vision") / "vision_worker.exe",
                Path(sys.executable).with_name("vision_worker.exe"),
            ):
                if candidate.is_file():
                    self.python = str(candidate)
                    self.worker = None
                    break
        if not self.python:
            # 开发模式：当前 Python 环境需安装 numpy、opencv-python、onnxruntime。
            self.python = sys.executable
            if self.worker is None:
                self.worker = Path(__file__).with_name("vision_worker.py")
        self.timeout = timeout
        self._lock = threading.Lock()
        # 侧车缓存版本:模型/参数升级后 +1,旧缓存自动失效重新分析
        self.cache_version = 3

    def analyze(self, image_path: Path) -> dict:
        if self.worker is not None and not self.worker.is_file():
            raise RuntimeError(f"vision_worker.py 不存在: {self.worker}")
        cache_path = image_path.with_name(image_path.name + ".vision.json")
        if cache_path.is_file():
            try:
                payload = json.loads(cache_path.read_text(encoding="utf-8"))
                # 缓存版本不一致(模型升级/图片被同名覆盖)则视为失效
                if payload.get("cache_version") == self.cache_version:
                    return payload
            except (OSError, json.JSONDecodeError):
                pass  # 缓存损坏则重新分析
        with self._lock:
            if cache_path.is_file():
                try:
                    payload = json.loads(cache_path.read_text(encoding="utf-8"))
                    if payload.get("cache_version") == self.cache_version:
                        return payload
                except (OSError, json.JSONDecodeError):
                    pass
            tmp_path = cache_path.with_suffix(".json.tmp")
            try:
                # worker 已打包进 exe 时只传 exe;开发模式传 python + worker.py
                command = [self.python] + (
                    [] if self.worker is None or self.worker.name.endswith(".exe")
                    else [str(self.worker)]
                ) + [str(image_path), "--out", str(tmp_path)]
                result = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    timeout=self.timeout,
                )
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError(f"vision worker 超时({self.timeout:.0f}s)") from exc
            except OSError as exc:
                raise RuntimeError(f"无法启动 vision worker: {exc}") from exc
            if result.returncode != 0:
                detail = (result.stderr or result.stdout or "").strip()[-500:]
                raise RuntimeError(f"vision worker 失败: {detail}")
            if not tmp_path.is_file():
                raise RuntimeError("vision worker 未生成输出文件")
            try:
                payload = json.loads(tmp_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise RuntimeError("vision worker 输出不是有效 JSON") from exc
            if not payload.get("ok"):
                raise RuntimeError(f"vision worker 返回失败: {payload.get('error')}")
            payload["cache_version"] = self.cache_version
            try:
                tmp_path.write_text(json.dumps(payload, ensure_ascii=False),
                                    encoding="utf-8")
            except OSError:
                pass  # 缓存写失败不影响本次返回
            tmp_path.replace(cache_path)
        return payload


vision_runner = VisionRunner()


class WebHandler(BaseHTTPRequestHandler):
    hub: DeviceHub
    static_dir: Path
    pairing_host: str
    pairing_port: int

    def log_message(self, fmt: str, *args: object) -> None:
        _safe_print(f"[HTTP] {self.address_string()} {fmt % args}")

    def _json(self, payload: dict, status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _serve_file(self, path: Path, download: bool = False, cache_seconds: int | None = None) -> None:
        try:
            data = path.read_bytes()
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        if cache_seconds is not None:
            # 缩略图等静态产物允许浏览器缓存,避免每次重建网格都重新下载 → 频闪
            self.send_header("Cache-Control", f"public, max-age={cache_seconds}")
        else:
            self.send_header("Cache-Control", "no-store")
        if download:
            self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
        self.end_headers()
        self.wfile.write(data)

    def _video(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        sequence = -1
        try:
            while True:
                with self.hub.frame_ready:
                    self.hub.frame_ready.wait_for(
                        lambda: self.hub.frame_sequence != sequence, timeout=2.0)
                    if self.hub.frame_sequence == sequence or not self.hub.latest_jpeg:
                        continue
                    sequence = self.hub.frame_sequence
                    jpeg = self.hub.latest_jpeg
                header = (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                          str(len(jpeg)).encode() + b"\r\n\r\n")
                self.wfile.write(header)
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def do_GET(self) -> None:  # noqa: N802
        try:
            self._route_get()
        except (BrokenPipeError, ConnectionResetError):
            return  # 浏览器断开,无需响应
        except Exception as exc:  # 顶层兜底:任何内部异常都以 JSON 返回,避免连接 reset
            _safe_print(f"[HTTP] GET {self.path} unhandled: {exc!r}")
            try:
                self._json({"ok": False, "error": f"internal error: {exc}"},
                           HTTPStatus.INTERNAL_SERVER_ERROR)
            except OSError:
                pass

    def _route_get(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            self._serve_file(self.static_dir / "index.html")
        elif parsed.path == "/api/status":
            self._json(self.hub.snapshot())
        elif parsed.path == "/video.mjpg":
            self._video()
        elif parsed.path == "/api/files":
            items = []
            for path in sorted(self.hub.data_dir.rglob("*")):
                if path.is_file() and not path.name.endswith(".part"):
                    stat = path.stat()
                    items.append({"path": str(path.relative_to(self.hub.data_dir)).replace("\\", "/"),
                                  "size": stat.st_size,
                                  "modified_ns": stat.st_mtime_ns})
            self._json({"ok": True, "items": items})
        elif parsed.path == "/api/scan":
            query = urllib.parse.parse_qs(parsed.query)
            relative = query.get("path", [""])[0]
            candidate = (self.hub.data_dir / relative).resolve()
            root = self.hub.data_dir.resolve()
            if (not relative or candidate == root or root not in candidate.parents or
                    candidate.suffix.lower() != ".csv"):
                self._json({"ok": False, "error": "invalid scan path"}, HTTPStatus.BAD_REQUEST)
            else:
                try:
                    result = analyze_scan(candidate)
                    result.update({"ok": True, "path": relative.replace("\\", "/")})
                    self._json(result)
                except FileNotFoundError:
                    self._json({"ok": False, "error": "scan file not found"}, HTTPStatus.NOT_FOUND)
                except ScanFormatError as exc:
                    self._json({"ok": False, "error": str(exc)}, HTTPStatus.UNPROCESSABLE_ENTITY)
        elif parsed.path == "/api/file":
            query = urllib.parse.parse_qs(parsed.query)
            relative = query.get("path", [""])[0]
            candidate = (self.hub.data_dir / relative).resolve()
            root = self.hub.data_dir.resolve()
            if candidate != root and root not in candidate.parents:
                self.send_error(HTTPStatus.BAD_REQUEST)
            else:
                self._serve_file(candidate, download=True)
        elif parsed.path == "/api/calib-photos":
            photo_dir = _calib_root() / "laser_spot"
            records_csv = photo_dir / "records.csv"
            ranges = {}
            if records_csv.exists():
                for line in records_csv.read_text(encoding="utf-8", errors="replace").splitlines():
                    fields = [f.strip() for f in line.split(",")]
                    if len(fields) < 10 or fields[0] != "SAVE" or fields[9] in ("-", ""):
                        continue
                    try:
                        recorded_mm = float(fields[3])
                        reference = int(fields[4])
                    except ValueError:
                        continue
                    offset = {0: 126.5, 1: -2.0, 2: 18.0}.get(reference, 0.0)
                    ranges[Path(fields[9]).name] = (recorded_mm - offset) / 1000.0
            items = []
            for path in sorted(photo_dir.glob("*.jpg")):
                if path.name.endswith(".thumb.jpg"):
                    continue
                items.append({"name": path.name, "range_m": ranges.get(path.name)})
            self._json({"ok": True, "items": items})
        elif parsed.path == "/api/calib-marks":
            marks_path = _calib_root() / "laser_spot_marks.json"
            if parsed.query:
                query = urllib.parse.parse_qs(parsed.query)
                if query.get("action", [""])[0] == "get":
                    marks = json.loads(marks_path.read_text(encoding="utf-8")) if marks_path.exists() else {}
                    self._json({"ok": True, "marks": marks})
            else:
                try:
                    body = json.loads(self._read_body())
                    marks_path.write_text(json.dumps(body, indent=2), encoding="utf-8")
                    self._json({"ok": True, "saved": len(body)})
                except (ValueError, OSError) as exc:
                    self._json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
        elif parsed.path == "/api/photo-annotation":
            self._route_photo_annotation(parsed)
        elif parsed.path == "/api/camera-intrinsics":
            calib_file = _calib_root() / "camera_intrinsics.json"
            try:
                payload = json.loads(calib_file.read_text(encoding="utf-8"))
                payload.update({"ok": True})
                self._json(payload)
            except FileNotFoundError:
                self._json({"ok": False, "error": "camera_intrinsics.json not found"},
                           HTTPStatus.NOT_FOUND)
        elif parsed.path == "/api/photos":
            items = []
            for path in self.hub.data_dir.rglob("*"):
                # 排除缩略图文件(.thumb.jpg 是服务端生成的缓存,再列出会引发
                # 网页对缩略图再生成缩略图的无限递归,直至文件爆炸、接口超时)
                if (path.is_file() and path.suffix.lower() in VisionRunner.IMAGE_SUFFIXES
                        and not path.name.endswith(".part")
                        and not path.name.endswith(".thumb.jpg")):
                    stat = path.stat()
                    cache = path.with_name(path.name + ".vision.json")
                    items.append({
                        "path": str(path.relative_to(self.hub.data_dir)).replace("\\", "/"),
                        "size": stat.st_size,
                        "modified_ns": stat.st_mtime_ns,
                        "has_vision": cache.is_file(),
                    })
            # 最新照片在前(此前按路径字典序,最新照片沉底导致网页自动分析旧照片)
            items.sort(key=lambda item: item["modified_ns"], reverse=True)
            self._json({"ok": True, "items": items})
        elif parsed.path == "/api/vision":
            query = urllib.parse.parse_qs(parsed.query)
            relative = query.get("path", [""])[0]
            candidate = (self.hub.data_dir / relative).resolve()
            root = self.hub.data_dir.resolve()
            if (not relative or candidate == root or root not in candidate.parents or
                    candidate.suffix.lower() not in VisionRunner.IMAGE_SUFFIXES):
                self._json({"ok": False, "error": "invalid image path"}, HTTPStatus.BAD_REQUEST)
            elif not candidate.is_file():
                self._json({"ok": False, "error": "image not found"}, HTTPStatus.NOT_FOUND)
            else:
                try:
                    # refresh=1 时丢弃侧车缓存重新分析(模型升级/参数调整后使用)
                    if query.get("refresh", [""])[0] == "1":
                        cache = candidate.with_name(candidate.name + ".vision.json")
                        if cache.is_file():
                            cache.unlink()
                    result = self.vision_runner.analyze(candidate)
                    result = dict(result)
                    result["path"] = relative.replace("\\", "/")
                    self._json(result)
                except RuntimeError as exc:
                    self._json({"ok": False, "error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
        elif parsed.path == "/api/thumbnail":
            query = urllib.parse.parse_qs(parsed.query)
            relative = query.get("path", [""])[0]
            candidate = (self.hub.data_dir / relative).resolve()
            root = self.hub.data_dir.resolve()
            if (not relative or candidate == root or root not in candidate.parents or
                    candidate.suffix.lower() not in VisionRunner.IMAGE_SUFFIXES):
                self._json({"ok": False, "error": "invalid image path"}, HTTPStatus.BAD_REQUEST)
            elif not candidate.is_file():
                self.send_error(HTTPStatus.NOT_FOUND)
            else:
                # 请求的本身就是缩略图(递归请求或旧缓存名)时直接返回,禁止再套娃生成
                if candidate.name.endswith(".thumb.jpg"):
                    self._serve_file(candidate)
                    return
                thumb_path = candidate.with_name(candidate.name + ".thumb.jpg")
                if (not thumb_path.is_file() or
                        thumb_path.stat().st_mtime_ns < candidate.stat().st_mtime_ns):
                    try:
                        from PIL import Image
                        with Image.open(candidate) as image:
                            image.thumbnail((320, 320))
                            image.convert("RGB").save(thumb_path, "JPEG", quality=72)
                    except Exception as exc:
                        self._json({"ok": False, "error": f"thumbnail failed: {exc}"},
                                   HTTPStatus.INTERNAL_SERVER_ERROR)
                        return
                self._serve_file(thumb_path, cache_seconds=600)
        elif parsed.path == "/api/device-files":
            self._proxy_device_json("files", parsed)
        elif parsed.path == "/api/device-file":
            self._proxy_device_file(parsed)
        else:
            # 通用静态文件:仅允许 static_dir 内的文件(标定页、照片等)
            candidate = (self.static_dir / parsed.path.lstrip("/")).resolve()
            root = self.static_dir.resolve()
            if candidate != root and root in candidate.parents and candidate.is_file():
                self._serve_file(candidate, cache_seconds=600)
                return
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        try:
            self._route_post()
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as exc:
            _safe_print(f"[HTTP] POST {self.path} unhandled: {exc!r}")
            try:
                self._json({"ok": False, "error": f"internal error: {exc}"},
                           HTTPStatus.INTERNAL_SERVER_ERROR)
            except OSError:
                pass

    def _route_post(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/calib-marks":
            marks_path = _calib_root() / "laser_spot_marks.json"
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 1 << 20:
                    raise ValueError("invalid request size")
                body = json.loads(self.rfile.read(length).decode("utf-8"))
                marks_path.write_text(json.dumps(body, indent=2), encoding="utf-8")
                self._json({"ok": True, "saved": len(body)})
            except (ValueError, OSError) as exc:
                self._json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        if parsed.path == "/api/vision-edit":
            self._handle_vision_edit()
            return
        if parsed.path == "/api/device-fs":
            self._proxy_device_fs(parsed)
            return
        if parsed.path != "/api/bind":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > MAX_BIND_BODY:
                raise ValueError("invalid request size")
            body = json.loads(self.rfile.read(length).decode("utf-8"))
            ssid = str(body.get("ssid", "")).strip()
            password = str(body.get("password", ""))
            pc_name = str(body.get("pc_name", socket.gethostname())).strip()[:48]
            if not 1 <= len(ssid.encode("utf-8")) <= 32:
                raise ValueError("热点名称必须为1到32字节")
            if not 8 <= len(password) <= 63:
                raise ValueError("热点密码必须为8到63个字符")
            payload = {"op": "bind", "ssid": ssid, "password": password,
                       "pc_name": pc_name, "server_port": self.server.device_port}
            with socket.create_connection((self.pairing_host, self.pairing_port), timeout=5) as client:
                client.sendall((json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n").encode())
                client.settimeout(5)
                reply = b""
                while b"\n" not in reply and len(reply) < 4096:
                    block = client.recv(1024)
                    if not block:
                        break
                    reply += block
            result = json.loads(reply.split(b"\n", 1)[0].decode("utf-8"))
            self._json(result, 200 if result.get("ok") else 502)
        except (ValueError, OSError, json.JSONDecodeError) as exc:
            self._json({"ok": False, "error": str(exc)}, 400)

    # ---------- 设备文件管理代理 ----------
    # 设备在 STA 模式下运行仅含文件接口的 HTTP 服务(端口 80);
    # PC 服务做同源代理,浏览器不直接跨域访问设备。

    @staticmethod
    def _valid_device_path(relative: str) -> bool:
        if not relative.startswith("/") or "\x00" in relative:
            return False
        for segment in relative.split("/"):
            if segment in ("..", "."):
                return False
        return True

    def _device_host(self) -> str:
        """返回设备文件服务的基础 URL(从 HELLO 中 http_port 取端口,缺省 80)。"""
        if not self.hub.device_connected:
            return ""
        peer = self.hub.peer or ""
        host = peer.rsplit(":", 1)[0] if ":" in peer else ""
        if not host:
            return ""
        try:
            port = int((self.hub.hello or {}).get("http_port", 80))
        except (TypeError, ValueError):
            port = 80
        return f"http://{host}:{port}"

    def _proxy_device_json(self, api: str, parsed) -> None:
        host = self._device_host()
        if not host:
            self._json({"ok": False, "error": "设备未连接,无法访问设备文件服务"},
                       HTTPStatus.SERVICE_UNAVAILABLE)
            return
        query = urllib.parse.parse_qs(parsed.query)
        relative = query.get("path", ["/"])[0]
        if not self._valid_device_path(relative):
            self._json({"ok": False, "error": "invalid device path"}, HTTPStatus.BAD_REQUEST)
            return
        url = f"{self._device_host()}/api/{api}?path=" + urllib.parse.quote(relative)
        try:
            with urllib.request.urlopen(url, timeout=8) as response:
                body = response.read(2 * 1024 * 1024)
                self.send_response(response.status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
        except urllib.error.HTTPError as exc:
            body = exc.read(2 * 1024 * 1024)
            self.send_response(exc.code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except OSError as exc:
            self._json({"ok": False, "error": f"设备文件服务不可达: {exc}"},
                       HTTPStatus.BAD_GATEWAY)

    def _proxy_device_file(self, parsed) -> None:
        host = self._device_host()
        if not host:
            self._json({"ok": False, "error": "设备未连接"}, HTTPStatus.SERVICE_UNAVAILABLE)
            return
        query = urllib.parse.parse_qs(parsed.query)
        relative = query.get("path", [""])[0]
        if not self._valid_device_path(relative):
            self._json({"ok": False, "error": "invalid device path"}, HTTPStatus.BAD_REQUEST)
            return
        url = f"{self._device_host()}/api/file?path=" + urllib.parse.quote(relative)
        if query.get("download", [""])[0] == "1":
            url += "&download=1"
        try:
            with urllib.request.urlopen(url, timeout=30) as response:
                data = response.read()
                content_type = response.headers.get_content_type() or "application/octet-stream"
                self.send_response(response.status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                if query.get("download", [""])[0] == "1":
                    name = relative.rsplit("/", 1)[-1] or "download"
                    self.send_header("Content-Disposition",
                                     f'attachment; filename="{name}"')
                self.end_headers()
                self.wfile.write(data)
        except urllib.error.HTTPError as exc:
            body = exc.read(1 * 1024 * 1024)
            self.send_response(exc.code)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except OSError as exc:
            self._json({"ok": False, "error": f"设备文件服务不可达: {exc}"},
                       HTTPStatus.BAD_GATEWAY)

    def _proxy_device_fs(self, parsed) -> None:
        host = self._device_host()
        if not host:
            self._json({"ok": False, "error": "设备未连接"}, HTTPStatus.SERVICE_UNAVAILABLE)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length) if length > 0 else b""
        except OSError as exc:
            self._json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        query = urllib.parse.parse_qs(parsed.query)
        action = query.get("action", [""])[0]
        relative = query.get("path", [""])[0]
        if action not in {"delete", "mkdir", "rename", "save"} or not self._valid_device_path(relative):
            self._json({"ok": False, "error": "invalid action or device path"}, HTTPStatus.BAD_REQUEST)
            return
        url = f"{self._device_host()}/api/fs?action={action}&path=" + urllib.parse.quote(relative)
        dest = query.get("dest", [""])[0]
        if dest and self._valid_device_path(dest):
            url += "&dest=" + urllib.parse.quote(dest)
        try:
            request = urllib.request.Request(url, data=body, method="POST")
            with urllib.request.urlopen(request, timeout=30) as response:
                reply = response.read(1 * 1024 * 1024)
                self.send_response(response.status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(reply)))
                self.end_headers()
                self.wfile.write(reply)
        except urllib.error.HTTPError as exc:
            reply = exc.read(1 * 1024 * 1024)
            self.send_response(exc.code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(reply)))
            self.end_headers()
            self.wfile.write(reply)
        except OSError as exc:
            self._json({"ok": False, "error": f"设备文件服务不可达: {exc}"},
                       HTTPStatus.BAD_GATEWAY)

    def _route_photo_annotation(self, parsed) -> None:
        """照片激光点标注:测量记录(距离) + 内参/外参 -> 光斑像素坐标。

        激光点在机身系的位置只取决于距离(与姿态无关),照片由相机拍摄,
        相机与激光刚性固定,因此投影不需要任何姿态信息。
        """
        global _annotation_records, _annotation_records_mtime, _annotation_records_fetched
        query = urllib.parse.parse_qs(parsed.query)
        relative = query.get("path", [""])[0]
        candidate = (self.hub.data_dir / relative).resolve()
        root = self.hub.data_dir.resolve()
        if (not relative or candidate == root or root not in candidate.parents or
                candidate.suffix.lower() not in VisionRunner.IMAGE_SUFFIXES or
                not candidate.is_file()):
            self._json({"ok": False, "error": "invalid image path"}, HTTPStatus.BAD_REQUEST)
            return

        # 测量记录:优先 PC 缓存;设备在线时定期拉取最新(60s 节流)
        now = time.monotonic()
        if self.hub.device_connected and now - _annotation_records_fetched > 60.0:
            _annotation_records_fetched = now
            try:
                host = self._device_host()
                if host:
                    url = host + "/api/file?path=" + urllib.parse.quote("/measurement_records_v3.csv")
                    with urllib.request.urlopen(url, timeout=8) as response:
                        body = response.read(4 * 1024 * 1024)
                    target = self.hub.data_dir / "measurement_records_v3.csv"
                    try:
                        target.write_bytes(body)
                    except OSError:
                        pass
            except (OSError, ValueError):
                pass  # 设备离线/无记录文件时用缓存
        records_changed = False
        for csv in self.hub.data_dir.rglob("measurement_records_v3.csv"):
            mtime = csv.stat().st_mtime_ns
            if mtime != _annotation_records_mtime:
                _annotation_records = _parse_records_csv(csv)
                _annotation_records_mtime = mtime
                records_changed = True
            break
        range_entry = _annotation_records.get(candidate.name)
        if range_entry is None:
            self._json({"ok": False, "error": "照片没有对应的测量记录(距离未知)"}, HTTPStatus.NOT_FOUND)
            return
        range_m, reference = range_entry

        if not _load_annotation_intrinsics() or not _load_annotation_extrinsics():
            self._json({"ok": False, "error": "相机标定数据缺失,请先完成标定"},
                       HTTPStatus.SERVICE_UNAVAILABLE)
            return

        try:
            from PIL import Image
            with Image.open(candidate) as image:
                width, height = image.size
        except OSError as exc:
            self._json({"ok": False, "error": f"图片读取失败: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        spot = _project_laser_spot(range_m, width, height)
        if spot is None:
            self._json({"ok": False, "error": "激光点投影失败(相机方向异常)"},
                       HTTPStatus.UNPROCESSABLE_ENTITY)
            return
        self._json({
            "ok": True,
            "x": round(spot[0], 2),
            "y": round(spot[1], 2),
            "range_m": round(range_m, 3),
            "distance_mm": round(range_m * 1000.0),
            "reference": reference,
            "reference_name": _REFERENCE_NAMES.get(reference, "未知"),
            "width": width,
            "height": height,
            "records_changed": records_changed,
        })

    def _handle_vision_edit(self) -> None:
        """保存用户在照片上的角点手工调整(user_edits 写入侧车缓存)。"""
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 256 * 1024:
                raise ValueError("invalid request size")
            body = json.loads(self.rfile.read(length).decode("utf-8"))
            relative = str(body.get("path", "")).strip()
            user_edits = body.get("user_edits") or {}
            if not isinstance(user_edits, dict) or not user_edits:
                raise ValueError("user_edits must be a non-empty object")
            candidate = (self.hub.data_dir / relative).resolve()
            root = self.hub.data_dir.resolve()
            if (not relative or candidate == root or root not in candidate.parents or
                    candidate.suffix.lower() not in VisionRunner.IMAGE_SUFFIXES):
                self._json({"ok": False, "error": "invalid image path"}, HTTPStatus.BAD_REQUEST)
                return
            if not candidate.is_file():
                self._json({"ok": False, "error": "image not found"}, HTTPStatus.NOT_FOUND)
                return
            cache_path = candidate.with_name(candidate.name + ".vision.json")
            if not cache_path.is_file():
                self._json({"ok": False, "error": "vision result missing"}, HTTPStatus.NOT_FOUND)
                return
            try:
                payload = json.loads(cache_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise ValueError(f"corrupt vision cache: {exc}") from exc
            merged = dict(payload.get("user_edits") or {})
            merged.update(user_edits)
            payload["user_edits"] = merged
            tmp_path = cache_path.with_suffix(".json.tmp")
            tmp_path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
            tmp_path.replace(cache_path)
            self._json({"ok": True, "user_edits": merged})
        except (ValueError, OSError, json.JSONDecodeError) as exc:
            self._json({"ok": False, "error": str(exc)}, 400)

    def do_DELETE(self) -> None:  # noqa: N802
        try:
            self._route_delete()
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as exc:
            _safe_print(f"[HTTP] DELETE {self.path} unhandled: {exc!r}")
            try:
                self._json({"ok": False, "error": f"internal error: {exc}"},
                           HTTPStatus.INTERNAL_SERVER_ERROR)
            except OSError:
                pass

    def _route_delete(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path not in ("/api/photos", "/api/scan"):
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        query = urllib.parse.parse_qs(parsed.query)
        relative = query.get("path", [""])[0]
        candidate = (self.hub.data_dir / relative).resolve()
        root = self.hub.data_dir.resolve()
        if parsed.path == "/api/photos":
            if (not relative or candidate == root or root not in candidate.parents or
                    candidate.suffix.lower() not in VisionRunner.IMAGE_SUFFIXES):
                self._json({"ok": False, "error": "invalid image path"}, HTTPStatus.BAD_REQUEST)
                return
        else:  # /api/scan:仅允许删除 CSV 扫描文件
            if (not relative or candidate == root or root not in candidate.parents or
                    candidate.suffix.lower() != ".csv"):
                self._json({"ok": False, "error": "invalid scan path"}, HTTPStatus.BAD_REQUEST)
                return
        if not candidate.is_file():
            self._json({"ok": False, "error": "file not found"}, HTTPStatus.NOT_FOUND)
            return
        try:
            candidate.unlink()
            if parsed.path == "/api/photos":
                cache = candidate.with_name(candidate.name + ".vision.json")
                if cache.is_file():
                    cache.unlink()
                thumb = candidate.with_name(candidate.name + ".thumb.jpg")
                if thumb.is_file():
                    thumb.unlink()
            self._json({"ok": True})
        except OSError as exc:
            self._json({"ok": False, "error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)


def build_servers(host: str, http_port: int, device_port: int, pairing_host: str,
                  pairing_port: int, data_dir: Path, static_dir: Path,
                  vision_runner: VisionRunner | None = None):
    hub = DeviceHub(data_dir=data_dir)
    stream_handler = type("BoundDeviceStreamHandler", (DeviceStreamHandler,), {"hub": hub})
    device_server = DeviceTCPServer((host, device_port), stream_handler)
    web_handler = type("BoundWebHandler", (WebHandler,), {
        "hub": hub,
        "static_dir": static_dir,
        "pairing_host": pairing_host,
        "pairing_port": pairing_port,
        "vision_runner": vision_runner if vision_runner is not None else globals()["vision_runner"],
    })
    web_server = ThreadingHTTPServer((host, http_port), web_handler)
    web_server.device_port = device_port
    return hub, device_server, web_server


def main() -> int:
    parser = argparse.ArgumentParser(description="Laser meter PC receiver and room-planner web UI")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument("--device-port", type=int, default=8765)
    parser.add_argument("--pairing-host", default="192.168.4.1")
    parser.add_argument("--pairing-port", type=int, default=8766)
    frozen = getattr(sys, "frozen", False)
    exe_dir = Path(sys.executable).resolve().parent if frozen else Path(__file__).resolve().parent
    parser.add_argument("--data-dir", type=Path,
                        default=exe_dir / "data")
    args = parser.parse_args()
    if frozen:
        # 打包后静态资源在 _internal/static(PyInstaller --add-data)
        static_dir = Path(sys._MEIPASS) / "static"
        if not static_dir.is_dir():
            static_dir = exe_dir / "static"
    else:
        static_dir = Path(__file__).with_name("static")
    args.data_dir.mkdir(parents=True, exist_ok=True)
    _hub, device_server, web_server = build_servers(
        args.host, args.http_port, args.device_port, args.pairing_host,
        args.pairing_port, args.data_dir, static_dir)
    device_thread = threading.Thread(target=device_server.serve_forever, name="device-receiver", daemon=True)
    device_thread.start()
    url = f"http://127.0.0.1:{args.http_port}/"
    _safe_print(f"PC网页: {url}")
    _safe_print(f"设备接收端口: TCP {args.device_port}")
    _safe_print(f"首次绑定目标: {args.pairing_host}:{args.pairing_port}")
    if frozen:
        # 打包版自动打开浏览器(延迟等端口就绪)
        threading.Timer(1.2, lambda: _open_browser(url)).start()
    try:
        web_server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        web_server.shutdown()
        device_server.shutdown()
        web_server.server_close()
        device_server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
