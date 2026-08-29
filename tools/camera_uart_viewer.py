#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
import argparse
import http.server
import re
import socketserver
import threading
import time

import serial


latest_jpeg = None
latest_seq = -1
latest_counter = 0
latest_lock = threading.Lock()
stop_event = threading.Event()


def read_line(ser):
    return ser.readline().decode("utf-8", errors="replace").strip()


def read_exact(ser, length):
    chunks = []
    total = 0
    deadline = time.monotonic() + max(2.0, length / 6000.0 + 2.0)
    while total < length and time.monotonic() < deadline:
        chunk = ser.read(length - total)
        if not chunk:
            continue
        chunks.append(chunk)
        total += len(chunk)
    return b"".join(chunks)


def serial_worker(port, baud, frames_per_request):
    global latest_jpeg, latest_seq, latest_counter
    header_re = re.compile(r"#JPG seq=(\d+) len=(\d+) dt_ms=(\d+)")
    with serial.Serial(port, baud, timeout=2) as ser:
        time.sleep(0.4)
        ser.reset_input_buffer()
        while not stop_event.is_set():
            ser.write(f"stream_camera_uart {frames_per_request}\n".encode("ascii"))
            ser.flush()
            while not stop_event.is_set():
                line = read_line(ser)
                if not line:
                    continue
                if line.startswith("#CAMUART DONE"):
                    break
                match = header_re.match(line)
                if not match:
                    continue
                seq = int(match.group(1))
                length = int(match.group(2))
                jpg = read_exact(ser, length)
                if len(jpg) != length:
                    continue
                read_line(ser)  # #END seq=N
                with latest_lock:
                    latest_jpeg = jpg
                    latest_seq = seq
                    latest_counter += 1
                print(f"frame seq={seq} len={length}")


class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(
                b"<html><head><title>ESP32 Camera UART</title></head>"
                b"<body style='margin:0;background:#111;color:#eee;font-family:sans-serif'>"
                b"<div style='padding:10px'>ESP32-S3 Camera UART Stream</div>"
                b"<img src='/stream.mjpg' style='width:100%;max-width:960px;image-rendering:auto'>"
                b"</body></html>"
            )
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        last_sent = -1
        while not stop_event.is_set():
            with latest_lock:
                jpg = latest_jpeg
                seq = latest_seq
                counter = latest_counter
            if jpg is not None and counter != last_sent:
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    last_sent = counter
                except (BrokenPipeError, ConnectionResetError):
                    break
            time.sleep(0.03)

    def log_message(self, fmt, *args):
        return


def main():
    parser = argparse.ArgumentParser(description="View ESP32 camera frames sent over UART.")
    parser.add_argument("--port", default="COM18", help="serial port, e.g. COM18")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--frames", type=int, default=120, help="frames per firmware request")
    parser.add_argument("--http-port", type=int, default=8080)
    args = parser.parse_args()

    t = threading.Thread(target=serial_worker, args=(args.port, args.baud, args.frames), daemon=True)
    t.start()
    with socketserver.ThreadingTCPServer(("127.0.0.1", args.http_port), Handler) as httpd:
        print(f"Open http://127.0.0.1:{args.http_port}/")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            stop_event.set()


if __name__ == "__main__":
    main()
