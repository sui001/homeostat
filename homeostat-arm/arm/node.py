"""The pot node that runs unit TL: a USB-serial bridge for the editor, as fader.py is for the fader.

The node (an ESP32-DevKitC with a motorised pot, homeostat/node/) runs TL of PT's model itself and
keeps the clock; the sim page runs TR, BR and BL and steps them to the node's time. A background
thread finds the node (a CP210x or CH340 USB-serial bridge, or the port in NODE_PORT), keeps its
latest status line, and passes on the page's needles. It lets go when the port vanishes (unplugged,
or being flashed) and picks it up again after. The sim talks to it through the editor's /api/node.

    python node.py            print what the node says, for a quick look
"""
import json, os, threading, time

try:
    import serial
    from serial.tools import list_ports
except ImportError:                  # armster may not have pyserial: the editor still runs
    serial = None

USB_UART_VIDS = (0x10C4, 0x1A86)     # Silicon Labs CP210x (the DevKitC), WCH CH340
_lock = threading.Lock()
_state = dict(connected=False, port=None, version=None, t=None, tl=None, q=None, held=None, needle=None,
              mode=None, speed=None, link=None, at=0.0, note=None, notes=[], stepped_at=None,
              error=None if serial else "pyserial is not installed here")
_out = []                            # lines to send
_started = [False]


def find_port():
    if os.environ.get("NODE_PORT"):
        return os.environ["NODE_PORT"]
    ports = [p.device for p in list_ports.comports() if p.vid in USB_UART_VIDS]
    return ports[0] if len(ports) == 1 else None


def _run():
    while True:
        port = find_port()
        if not port:
            with _lock:
                _state.update(connected=False, port=None, error="no node found (plug it in, or set NODE_PORT)")
            time.sleep(2); continue
        try:
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = port, 115200, 0.2
            s.dtr = False; s.rts = False     # opening must not reset the DevKitC (its auto-reset uses DTR/RTS)
            s.open()
            with s:
                with _lock:
                    _state.update(connected=True, port=port, error=None)
                s.write(b"V\n")
                buf = b""
                while True:
                    with _lock:
                        out, _out[:] = list(_out), []
                    for line in out:
                        s.write((line + "\n").encode())
                    buf += s.read(s.in_waiting or 1)
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        _take(raw.decode(errors="replace").strip())
        except Exception as e:           # unplugged, flashed, or busy: try again shortly
            with _lock:
                _state.update(connected=False, error="lost the node on %s: %s" % (port, e))
            time.sleep(1)


def _take(line):
    if not line:
        return
    if line.startswith("{"):
        try:
            d = json.loads(line)
        except ValueError:
            return
        with _lock:
            if "version" in d:
                _state["version"] = d["version"]
            else:
                if d.get("stepped"):
                    _state["stepped_at"] = d.get("t")
                _state.update(d, at=time.time())
        return
    if line.startswith(("pot raw", "unit t", "status raw")):
        return                           # the bench sketch's debug stream, not news
    with _lock:
        _state["note"] = line            # "hand on ...", "TL reorganised ...", "FAULT: ...", boot line
        _state["notes"] = (_state["notes"] + [time.strftime("%H:%M:%S ") + line])[-12:]


def start():
    if serial and not _started[0]:
        _started[0] = True
        threading.Thread(target=_run, daemon=True).start()


DRIVER_STALE = 1.5                   # s: a driving page that has gone quiet this long gives the node up
_driver = dict(id=None, at=0.0)


def send(who, claim=False, needles=None, cmd=None):
    """From the page `who`: its TR, BR, BL at its model time (needles = [t, a, b, c]) and/or a
    command line ('link P S G T', 'speed X', 'd0 X'). One page drives at a time, as each page runs
    its own homeostat: the page that last claimed it, or any page once the driver has gone quiet.
    True if `who` is the driver."""
    now = time.time()
    with _lock:
        if not (claim or _driver["id"] is None or who == _driver["id"] or now - _driver["at"] > DRIVER_STALE):
            return False
        _driver.update(id=who, at=now)
        if cmd:
            _out.append(cmd.strip()[:80])
        if needles is not None:
            t, a, b, c = [float(v) for v in needles]
            line = "N %.3f %.2f %.2f %.2f" % (t, a, b, c)
            _out[:] = [l for l in _out if not l.startswith("N ")] + [line]   # only the newest matters
        return True


def state():
    with _lock:
        s = dict(_state, notes=list(_state["notes"]))
        s["driver"] = _driver["id"]
        s["driver_age"] = round(time.time() - _driver["at"], 2) if _driver["at"] else None
    s["age"] = round(time.time() - s["at"], 2) if s["at"] else None
    if s["connected"] and s["age"] is not None and s["age"] > 1.0:
        s["error"] = "the node has gone quiet (not linked yet, or stopped)"
    return s


if __name__ == "__main__":
    start()
    while True:
        print(state()); time.sleep(0.5)
