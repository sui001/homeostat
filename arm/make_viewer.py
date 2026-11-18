#!/usr/bin/env python3
"""Turn plan.json (or plan_checked.json) into a self-contained viewer page: print_sim.html."""
import json, sys
import numpy as np
import gen3, plan

src = sys.argv[1] if len(sys.argv) > 1 else "plan.json"
dst = sys.argv[2] if len(sys.argv) > 2 else "print_sim.html"
d = json.load(open(src))
if d["params"].get("long_nozzle"): plan.HOTEND[:] = plan.HOTEND_LONG
frames = []
for w in d["waypoints"]:
    if not w["ok"]:
        frames.append(None); continue
    q = w.get("arm_ik") or w["joints"]
    F = gen3.frames(q, plan.TOOL)
    pts = [[0, 0, 0]] + [f[:3, 3].round(4).tolist() for f in F]
    frames.append(dict(p=pts, ax=F[-1][:3, 2].round(4).tolist(), h=min(w["hotend_mm"]), a=min(w["arm_mm"]),
                       tilt=w["tilt"], q=[round(v, 1) for v in q]))
data = dict(strand=d["strand"], frames=frames, params=d["params"], hotend=plan.HOTEND, arm=plan.ARM,
            check=d.get("arm_check"))
html = open("viewer_template.html", encoding="utf-8").read().replace("/*DATA*/null", json.dumps(data))
open(dst, "w", encoding="utf-8").write(html)
print("wrote", dst)
