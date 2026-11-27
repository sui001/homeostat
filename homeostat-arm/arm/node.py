"""The pot nodes on USB: a serial bridge for the editor, as fader.py is for the fader.

Each node (an ESP32-DevKitC with a motorised pot, homeostat/node/) runs one unit of PT's model and
says which: 0 TL, 1 TR, 2 BR, 3 BL (firmware v1.11 keeps it in flash; older firmware is TL). One
linked node keeps the clock; the sim page runs whatever units have no node and steps them to that
clock, and passes every node all four needles. Any number of nodes may be plugged in at once: a
thread per USB-serial port (CP210x or CH340, or the ports in NODE_PORT, comma separated) keeps that
node's latest status line and sends it what the page says. A port lets go when it vanishes
(unplugged, or being flashed) and is picked up again after. The sim talks to it through the
editor's /api/node.

    python node.py            print what the nodes say, for a quick look
"""
import json, os, threading, time

try:
    import serial
    from serial.tools import list_ports
except ImportError:                  # armster may not have pyserial: the editor still runs
    serial = None

USB_UART_VIDS = (0x10C4, 0x1A86)     # Silicon Labs CP210x (the DevKitC), WCH CH340
NAMES = ("TL", "TR", "BR", "BL")
_lock = threading.Lock()
_nodes = {}                          # port -> its state, and the lines waiting for it
_started = [False]
_error = [None if serial else "pyserial is not installed here"]


def _fresh(port):
    return dict(port=port, connected=False, u=None, unit=None, version=None, t=None, x=None, q=None,
                held=None, needle=None, mode=None, speed=None, link=None, master=None, at=0.0,
                note=None, notes=[], stepped_at=None, error=None, out=[])


def find_ports():
    if os.environ.get("NODE_PORT"):
        return [p.strip() for p in os.environ["NODE_PORT"].split(",") if p.strip()]
    return [p.device for p in list_ports.comports() if p.vid in USB_UART_VIDS]


def _run(port):
    st = _nodes[port]
    try:
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = port, 115200, 0.2
        s.dtr = False; s.rts = False         # opening must not reset the DevKitC (its auto-reset uses DTR/RTS)
        s.open()
        with s:
            with _lock:
                st.update(connected=True, error=None)
            s.write(b"V\n")
            buf = b""
            while True:                      # until the port goes (a read or write then raises)
                with _lock:
                    out, st["out"] = st["out"], []
                for line in out:
                    s.write((line + "\n").encode())
                buf += s.read(s.in_waiting or 1)
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    _take(st, raw.decode(errors="replace").strip())
    except Exception as e:               # unplugged, flashed, or busy: the watcher tries again shortly
        with _lock:
            st["error"] = "lost the node on %s: %s" % (port, e)
    with _lock:
        st.update(connected=False, running=False)


def _watch():
    while True:
        ports = find_ports()
        with _lock:
            for p in ports:
                st = _nodes.get(p)
                if st is None or not st.get("running"):
                    if st is None:
                        st = _nodes[p] = _fresh(p)
                    st["running"] = True
                    threading.Thread(target=_run, args=(p,), daemon=True).start()
            for p in list(_nodes):
                if p not in ports and not _nodes[p].get("running"):
                    del _nodes[p]                    # unplugged: forget it
        time.sleep(2)


def _set_unit(st, u):
    st["u"] = u
    st["unit"] = NAMES[u] if isinstance(u, int) and 0 <= u < 4 else None


def _take(st, line):
    if not line:
        return
    if line.startswith("{"):
        try:
            d = json.loads(line)
        except ValueError:
            return
        with _lock:
            if "version" in d:
                st["version"] = d["version"]
                _set_unit(st, d.get("u", 0))         # before v1.11 a node was always TL
            else:
                if "tl" in d:                        # v1.9/v1.10: TL only
                    d["x"] = d.pop("tl"); d.setdefault("u", 0); d.setdefault("master", True)
                if d.get("stepped"):
                    st["stepped_at"] = d.get("t")
                _set_unit(st, d.get("u", st["u"]))
                st.update(d, at=time.time())
        return
    if line.startswith(("pot raw", "unit t", "status raw")):
        return                               # the bench sketch's debug stream, not news
    with _lock:
        st["note"] = line                    # "hand on ...", "TR reorganised ...", "FAULT: ...", boot line
        st["notes"] = (st["notes"] + [time.strftime("%H:%M:%S ") + line])[-12:]


def start():
    if serial and not _started[0]:
        _started[0] = True
        threading.Thread(target=_watch, daemon=True).start()


DRIVER_STALE = 1.5                   # s: a driving page that has gone quiet this long gives the nodes up
_driver = dict(id=None, at=0.0)


def send(who, claim=False, needles=None, cmd=None):
    """From the page `who`: all four needles at the clock's time (needles = [t, x0, x1, x2, x3]) for
    every node, and/or a command line. A command 'U:text' goes to the node that is unit U, plain
    'text' to every node. One page drives at a time, as each page runs its own homeostat: the page
    that last claimed them, or any page once the driver has gone quiet. True if `who` is the driver."""
    now = time.time()
    with _lock:
        if not (claim or _driver["id"] is None or who == _driver["id"] or now - _driver["at"] > DRIVER_STALE):
            return False
        _driver.update(id=who, at=now)
        for st in _nodes.values():
            if not st["connected"]:
                continue
            if cmd:
                c = cmd.strip()
                if len(c) > 2 and c[1] == ":" and c[0].isdigit():
                    if st["u"] != int(c[0]):
                        c = None
                    else:
                        c = c[2:]
                if c:
                    st["out"].append(c[:80])
            if needles is not None:
                t, *x = [float(v) for v in needles]
                line = "N %.3f %s" % (t, " ".join("%.2f" % v for v in x))
                st["out"] = [l for l in st["out"] if not l.startswith("N ")] + [line]   # only the newest matters
        return True


def state():
    now = time.time()
    with _lock:
        nodes = []
        for st in _nodes.values():
            s = {k: v for k, v in st.items() if k not in ("out", "running")}
            s["notes"] = list(st["notes"])
            s["age"] = round(now - s["at"], 2) if s["at"] else None
            if s["connected"] and s["age"] is not None and s["age"] > 1.0 and not s["error"]:
                s["error"] = "gone quiet (not linked yet, or stopped)"
            nodes.append(s)
        nodes.sort(key=lambda s: (s["u"] if s["u"] is not None else 9, s["port"]))
        r = dict(nodes=nodes, driver=_driver["id"],
                 driver_age=round(now - _driver["at"], 2) if _driver["at"] else None,
                 error=_error[0] or (None if nodes else "no node found (plug one in, or set NODE_PORT)"))
    return r


if __name__ == "__main__":
    start()
    while True:
        for n in state()["nodes"]:
            print(n["port"], n["unit"], n["version"], n["mode"], n["t"], n["x"], n["error"])
        time.sleep(0.5)
