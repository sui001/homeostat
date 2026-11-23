"""Which version of this code is running: deploy.sh writes VERSION next to the scripts (armster has no git);
on the laptop it comes from git. Shown on the pages, stored in every plan and printed at the top of runs."""
import json, os, subprocess
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))


def get():
    f = os.path.join(HERE, "VERSION")
    if os.path.exists(f):
        try:
            return json.load(open(f))
        except ValueError:
            pass
    try:
        git = lambda *a: subprocess.run(["git"] + list(a), cwd=HERE, capture_output=True, text=True, timeout=5).stdout.strip()
        c = git("rev-parse", "--short", "HEAD")
        if c:
            return dict(commit=c, date=git("log", "-1", "--format=%cI"), changes=len(git("status", "--porcelain", ".").splitlines()))
    except Exception:
        pass
    return dict(commit="unknown")


def label(v=None):
    v = v or get()
    s = v.get("commit", "unknown")
    try:
        s += " · " + datetime.fromisoformat(v["date"]).strftime("%-d %b %H:%M" if os.name != "nt" else "%d %b %H:%M")
    except Exception:
        pass
    if v.get("changes"):
        s += " +%d uncommitted" % v["changes"]
    return s


if __name__ == "__main__":
    print(label())
