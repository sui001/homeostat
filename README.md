# Homeostat strand on the Kinova

Free-form extruding the 3-D path of PT's 4-D homeostat model with a Kinova Gen3 (6-DOF), without the
arm or hot end running into strand it has already printed. Everything is in `arm/`.

## Layout editor

Runs on armster as the `homeostat-editor` service: open **http://armster:8792** from anything on the
tailnet. It listens on armster's Tailscale address only, not on the arm's wired network or the Wi-Fi.

```sh
./deploy.sh                 # copy arm/ to armster and (re)start the service
python arm/editor.py        # or run it on a laptop: http://localhost:8792 (arm check goes over ssh)
```

Place, turn, tilt and scale the path inside the arm's reach, like a slicer's build plate: drag the
path to move it, the orange ring turns it, the red and teal rings tilt it, the corner squares scale it,
Shift-dragging the path raises or lowers it, and dragging empty space turns the view. The green dome is where the nozzle can get to at all. The
path is coloured as you go: printable, blocked by strand already printed, out of reach, or outside the
painting rig's working sweep. The quick check is optimistic; **Plan** runs the real planner, and
**Plan and check on the arm** also runs the no-motion arm check, which
asks the arm's own IK about every 10th point (and around any big joint step); `ARM_CHECK_EVERY=1` checks them all. Results land in `arm/plans/` (on
armster: `~/homeostat-arm/arm/plans`, not in git) with a viewer page each.

### Air-run, Park and the live arm

On armster the editor can also move the arm, after a confirm on the page: **Air-run** follows a
checked plan raised above its height (1x to 4x; big plans run in parts of up to 900 waypoints), and
**Park** goes Home. Both leave a print the safe way (`exit_path.py`): back out 50 mm along the nozzle,
rise 100 mm above the print, then Home, every pose checked against the strand printed so far before
anything moves. A direct swing to Home from the spiral would put the hot end's duct 6 mm into it.
While anything moves, Stop is pinned to the top of the page (Esc or Space too), and the arm is drawn
live from its joint angles.

### Light drawing

Tick **Light drawing** to plan a path for a light carried by the arm, photographed in one long
exposure. **Raise off the table** (or Shift-drag) lifts the drawing into the air; nothing holds it up. The Speed menu shows how
long the drawing takes at each speed (worked out from the plan's joints the way the run times it; a curvy
path tops out below the speed asked for). Nothing is left behind, so the planner only keeps to the arm's reach, its joint limits and the
table: paths can cross themselves and planning takes seconds instead of minutes. The run (**Draw with
light…**) goes down onto the start dark, waits while you open the shutter (three high beeps and a
countdown on the page), switches the light on along the path only, off at the end, waits while you close
the shutter (two low beeps), then leaves the safe way.

The light is switched on one of the wrist's four digital pins (GPIO 1-4 on the bracelet's expansion
connector, 3.3 V): drive the light through a small logic-level MOSFET or transistor, not from the pin
directly. With no pin chosen the light stays on throughout, approach and exit included.

### Attractors

The **Attractor** row under Path makes a classic 3-D strange attractor (Lorenz, Rössler, Thomas,
Aizawa, Halvorsen, Chen) and makes it the path, standing upright, with Light drawing on: **Run it for**
is the system's own time (longer = more loops, longer drawing), **Seed** nudges the start for a
different trajectory. At scale S it is about 2 S mm across. `attractors.py` does the maths.

## The scripts

| File | What it does |
|---|---|
| `plan.py` | Look-ahead planner: clear nozzle directions per point, a smooth sequence, IK |
| `arm_sim.py` | Runs on armster. Checks a plan with the arm's own IK and trajectory validation. No motion |
| `air_run.py` | Runs on armster. Traces a plan on the real arm, raised, no extrusion. Needs `KINOVA_CONFIRM=yes` |
| `editor.py`, `editor.html`, `editor_core.py` | The layout editor (serves the simulator at /sim too) |
| `homeostat.html` | The homeostat simulator; Send to the arm passes its path to the editor |
| `exit_path.py`, `safe_park.py` | The checked way out of a print, and Park built on it |
| `armmove.py` | Running joint waypoint lists, timing fitted to the arm, the live feed |
| `arm_stop.py` | Sends the arm Stop (the editor's Stop button) |
| `make_viewer.py`, `viewer_template.html` | One-page 3-D viewer for a plan |
| `gen3.py`, `wrist.py` | Arm kinematics (calibrated against the arm) and quick wrist estimates |
| `homeostat_path.py` | The homeostat model, ported from the page, and its 3-D path |
| `attractors.py` | Lorenz, Rössler, Thomas, Aizawa, Halvorsen and Chen attractors as paths |
| `armstream.py` | Light drawings as one continuous motion: joint speeds streamed to the arm |

## Numbers to keep in mind

- The arm's tool transform is 0.200 m and stays that way (the painting rig uses it). The tip is
  `gen3.NOZZLE` (0.230 m, the gripper's fingertips for now) and poses go through `gen3.arm_tcp()`.
- The print surface is taken as z = 0.16 m in the arm's frame. The painting rig dabs there.
- Hot end sizes in `plan.HOTEND` are still guesses.
