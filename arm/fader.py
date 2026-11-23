"""The motorised fader that plays unit TL: a USB-serial bridge for the editor.

A background thread finds the fader (an ESP32-S3 SuperMini: Espressif's USB id 303A, or the port
in FADER_PORT), reads its status lines and keeps the latest, and passes on where TL should be.
It lets go when the port vanishes (unplugged, or being flashed) and picks it up again after.
The sim and the /fader page talk to it through the editor's /api/fader.

    python fader.py            print what the fader says, for a quick look
"""
import json, os, threading, time

try:
    import serial
    from serial.tools import list_ports
except ImportError:                  # armster may not have pyserial: the editor still runs
    serial = None

ESPRESSIF_VID = 0x303A
_lock = threading.Lock()
_state = dict(connected=False, port=None, version=None, deg=None, pos=None, tgt=None, mode=None,
              at=0.0, note=None, notes=[], error=None if serial else "pyserial is not installed here")
_out = []                            # lines to send
_started = [False]


def find_port():
    if os.environ.get("FADER_PORT"):
        return os.environ["FADER_PORT"]
    ports = [p.device for p in list_ports.comports() if p.vid == ESPRESSIF_VID]
    return ports[0] if len(ports) == 1 else None


def _run():
    while True:
        port = find_port()
        if not port:
            with _lock:
                _state.update(connected=False, port=None, error="no fader found (plug it in, or set FADER_PORT)")
            time.sleep(2); continue
        try:
            with serial.Serial(port, 115200, timeout=0.2) as s:
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
                _state.update(connected=False, error="lost the fader on %s: %s" % (port, e))
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
                _state.update(d, at=time.time())
        return
    with _lock:
        _state["note"] = line            # "hand on: letting go", "FAULT: ...", boot line, tuning echo
        _state["notes"] = (_state["notes"] + [time.strftime("%H:%M:%S ") + line])[-12:]


def start():
    if serial and not _started[0]:
        _started[0] = True
        threading.Thread(target=_run, daemon=True).start()


DRIVER_STALE = 1.5                   # s: a driving page that has gone quiet this long gives the fader up
_driver = dict(id=None, at=0.0)


def follow(deg, who=None, claim=False):
    """Ask the fader to follow TL at this deflection (degrees, -50..+50), for the page `who`.
    One page drives at a time, as each page runs its own homeostat: the page that last claimed it
    (ticked its box), or any page once the driver has gone quiet. deg None just keeps the claim
    alive (a page whose TL is being held by the hand at the fader). True if `who` is the driver."""
    now = time.time()
    with _lock:
        if not (claim or _driver["id"] is None or who == _driver["id"] or now - _driver["at"] > DRIVER_STALE):
            return False
        _driver.update(id=who, at=now)
        if deg is not None:
            deg = max(-50.0, min(50.0, float(deg)))
            _out[:] = [l for l in _out if not l.startswith("T ")] + ["T %.2f" % deg]   # only the newest matters
        return True


def state():
    with _lock:
        s = dict(_state, notes=list(_state["notes"]))
        s["driver"] = _driver["id"]
        s["driver_age"] = round(time.time() - _driver["at"], 2) if _driver["at"] else None
    s["age"] = round(time.time() - s["at"], 2) if s["at"] else None
    if s["connected"] and s["age"] is not None and s["age"] > 1.0:
        s["error"] = "the fader has gone quiet"
    return s


if __name__ == "__main__":
    start()
    while True:
        print(state()); time.sleep(0.5)
