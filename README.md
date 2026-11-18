# Homeostat strand on the Kinova

Free-form extruding the 3-D path of PT's 4-D homeostat model with a Kinova Gen3 (6-DOF), without the
arm or hot end running into strand it has already printed. Everything is in `arm/`.

## Layout editor

```sh
python arm/editor.py        # then open http://localhost:8792
```

Place, turn, tilt and scale the path inside the arm's reach, like a slicer's build plate. The green
ring is where the nozzle can get to at all. The path is coloured as you go: printable, blocked by strand
already printed, out of reach, or outside the painting rig's working sweep. The quick check is
optimistic; **Plan** runs the real planner, and **Plan and check on the arm** also runs the no-motion
check on armster. Results land in `arm/plans/` (not in git) with a viewer page each.

## The scripts

| File | What it does |
|---|---|
| `plan.py` | Look-ahead planner: clear nozzle directions per point, a smooth sequence, IK |
| `arm_sim.py` | Runs on armster. Checks a plan with the arm's own IK and trajectory validation. No motion |
| `air_run.py` | Runs on armster. Traces a plan on the real arm, raised, no extrusion. Needs `KINOVA_CONFIRM=yes` |
| `editor.py`, `editor.html`, `editor_core.py` | The layout editor |
| `make_viewer.py`, `viewer_template.html` | One-page 3-D viewer for a plan |
| `gen3.py`, `wrist.py` | Arm kinematics (calibrated against the arm) and quick wrist estimates |
| `homeostat_path.py` | The homeostat model, ported from the page, and its 3-D path |

## Numbers to keep in mind

- The arm's tool transform is 0.200 m and stays that way (the painting rig uses it). The tip is
  `gen3.NOZZLE` (0.230 m, the gripper's fingertips for now) and poses go through `gen3.arm_tcp()`.
- The print surface is taken as z = 0.16 m in the arm's frame. The painting rig dabs there.
- Hot end sizes in `plan.HOTEND` are still guesses.
