"""Live ToF range page for lining up the sensor and the flag on the arm.

    python wired/tools/range_view.py COM68 [--log readings.csv]
    then open http://localhost:8765

Reads the R,<mm>[,<drive%>] lines from tof_range_stream or sweep_and_sense and
pushes them to the page as server-sent events, logging them to CSV if asked.
Only one program can hold the COM port, so close the Serial Monitor first.
"""

import http.server
import argparse
import json
import os
import socketserver
import threading
import time

import serial

PORT_HTTP = 8765
HERE = os.path.dirname(os.path.abspath(__file__))

readings = []          # (t, mm, drive) since the last push, mm = -1 when blind, drive None if not sent
lock = threading.Lock()


def read_sensor(port, log_path):
    while True:
        try:
            with serial.Serial(port, 115200, timeout=1) as s:
                while True:
                    line = s.readline().decode(errors="ignore").strip()
                    if line.startswith("R,"):
                        parts = line[2:].split(",")
                        try:
                            mm = int(parts[0])
                            drive = int(parts[1]) if len(parts) > 1 else None
                        except ValueError:
                            continue
                        t = time.time()
                        with lock:
                            readings.append((t, mm, drive))
                        if log_path:
                            with open(log_path, "a") as f:
                                f.write(f"{t:.3f},{mm},{'' if drive is None else drive}\n")
        except serial.SerialException as e:
            print("serial:", e, "- retrying")
            time.sleep(1)


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    with lock:
                        batch = readings[:]
                        readings.clear()
                    if batch:
                        self.wfile.write(f"data: {json.dumps(batch)}\n\n".encode())
                        self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return
        else:
            with open(os.path.join(HERE, "range_view.html"), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(body)


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM68")
    ap.add_argument("--log", help="append every reading to this CSV: time,mm,drive")
    args = ap.parse_args()
    port = args.port
    threading.Thread(target=read_sensor, args=(port, args.log), daemon=True).start()
    print(f"reading {port}, page at http://localhost:{PORT_HTTP}")
    Server(("127.0.0.1", PORT_HTTP), Handler).serve_forever()
