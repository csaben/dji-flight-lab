# Ideas (parked, not started)

## Drone-free splats: phone video + satellite imagery

*Noted 2026-11-07.*

Build the Gaussian splat from **handheld phone video plus satellite or aerial imagery only**, with no drone capture flight. Then plan and preview virtual paths in the sim for places the drone has never flown.

- Phone video gives ground-level detail: facades, under eaves, between houses.
- Satellite imagery gives the overhead view, roofs and footprints, and a GPS-aligned scale anchor. That could also replace the drone clip's weak ±11 m GPS fit.
- Open questions:
  - Can a reconstruction (COLMAP/GLOMAP, gsplat, or World Labs) fuse such different viewpoints?
  - How should orthographic satellite tiles be treated as cameras?
  - Will roof heights come out right without oblique aerial views?
- Payoff: scout and pre-plan new areas before ever bringing the drone, and get training scenes for RL far more cheaply.

## Race maker mode: custom FPV gate courses, time trials and ghosts

*Noted 2026-11-08.*

Fly around the splat in the sandbox's first-person view and drop **race gates**: rings with an adjustable diameter, placed and oriented at the drone's current pose, much like "Place in 3D" drops waypoints. Each saved course is an ordered list of gates.

- **RL training:** give the gate course to dji-rl as a new task, flying the gates in order as fast as possible without collisions, and train a racing policy on it.
- **Overlay:** render the gates on top of the normal first-person view, both in the sim and, later, on the real drone's live feed using its GPS and attitude, so you can see the course while flying.
- **Time trials:**
  - A start/finish gate, lap and split times, and a best-time leaderboard for each course.
  - Missed gates or collisions count as penalties.
- **Ghosts:**
  - Replay your best lap, or the trained agent's lap from RL training, as a translucent drone racing alongside you, like Mario Kart's time-trial ghosts.
  - Store every run as a trajectory (t, pose) so any run can become a ghost.
- **Open questions:**
  - How to render rings in the splat viewer: dji-rl's renderer, or composite them client-side using the camera pose from X-State.
  - Gate orientation when placing: use the drone's heading, or a separate "aim" control.
  - How safe it would be to run courses on the real drone at speed. Sim-only to start.
