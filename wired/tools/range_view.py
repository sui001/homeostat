"""Live ToF range page for lining up the sensor and the flag on the arm.

    python wired/tools/range_view.py COM68
    then open http://localhost:8765

Reads the R,<mm> lines from tof_range_stream and pushes them to the page as
server-sent events. Only one program can hold the COM port, so close the
Serial Monitor first.
"""

import http.server
import json
import os
import socketserver
import sys
import threading
import time

import serial

PORT_HTTP = 8765
HERE = os.path.dirname(os.path.abspath(__file__))

readings = []          # (t, mm) since the last push, mm = -1 when blind
lock = threading.Lock()


def read_sensor(port):
    while True:
        try:
            with serial.Serial(port, 115200, timeout=1) as s:
                while True:
                    line = s.readline().decode(errors="ignore").strip()
                    if line.startswith("R,"):
                        try:
                            mm = int(line[2:])
                        except ValueError:
                            continue
                        with lock:
                            readings.append((time.time(), mm))
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
    port = sys.argv[1] if len(sys.argv) > 1 else "COM68"
    threading.Thread(target=read_sensor, args=(port,), daemon=True).start()
    print(f"reading {port}, page at http://localhost:{PORT_HTTP}")
    Server(("127.0.0.1", PORT_HTTP), Handler).serve_forever()
