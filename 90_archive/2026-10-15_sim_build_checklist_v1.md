# Build checklist: the CaP robot in Viam sim

Written Thu 15 Oct 2026 (planning on Windows). Built on Linux with Claude Code.
Time: about two and a half days (Thu afternoon, Fri, Sat). Sun stays for the lab computer + week summary.

## The goal in one line
The CaP robot (KUKA KR6 on the lab cart, suction tool, sensors) and the lab world live
in Viam's own sim. A Python script drives the cart to the table, picks a box with the
suction tool and places it, using Viam services, and our own DoCommand verbs only where
Viam has nothing.

## Decisions taken today (also in decisions.md #11-#14)
- **Viam-only sim.** No MuJoCo, no ROS2 in this build. Viam fake/sim parts + our own small modules.
- **Code lives in Viam_Lab** (`20_sim/`), moves into CaP_Thesis with the rest after the sprint.
- **Viam services first.** If Viam has it (arm API, motion service, base API, frame system),
  we use it. If it's missing, we add it as a DoCommand verb in our module (the thesis primitives).
- **Scope: everything** - arm, cart, vacuum, cameras, lidar, IMU, odometry, world obstacles.
- **Numbers come from CaP_Thesis, never typed by hand.** One script reads the MJCF and the
  `params.py` files and writes the Viam numbers. Thesis = meters, Viam = **millimeters**.

## Read this before starting: what a Viam-only sim can and can't do
| Part | In Viam-only sim | Verified? |
|---|---|---|
| Arm | Joints move, kinematics + motion planning + collision checks work | Yes (Viam 101) |
| Cart (base) | Fake base has no attributes; unclear if it tracks its own pose | **No - spike S2** |
| Vacuum | No suction physics. Our gripper module keeps a "holding" flag + geometry | Plan |
| Cameras | Fake camera = same gradient image every time (+ a point cloud) | Yes (docs) |
| Lidar, IMU | Fake parts: wired, framed, readable, but fixed data | Partly - spike S5 |
| World | Static obstacles from frame configs, box/sphere/capsule/mesh, add 20-50 mm margin | Yes (docs) |

So: by Saturday the **wiring, frames, geometry, planning and program flow** are real. Sensor
**data** is placeholder. Real sensor data = Phase 10 (stretch, bridge to the thesis MuJoCo sim).

## Who does what
- **[S]** Shahar does it (decides, runs, checks, writes in the app).
- **[CC]** Claude Code on Linux does the heavy work. Shahar reads, runs and questions it.
- **[S-write]** Shahar writes this piece himself to check he understood. Hint first, solution only if asked.
- **GATE** = stop. Claude Code shows the check, Shahar runs it himself and says "pass". Then commit
  with the gate number in the message. No next phase before that.

---

## Open questions - answer before Phase 1
- [ ] Q1 Which Linux computer? (stav: Ubuntu 22.04, RTX 3060 / or the strong lab computer). Claude Code runs there?
- [ ] Q2 Real hours: Thu afternoon __ h, Fri __ h, Sat __ h. Is Sun still the lab-computer day?
- [ ] Q3 Fake sensor data is OK for this build (real data = Phase 10 stretch)? Y / N
- [ ] Q4 Did the lab day happen / will it? Until it does, **the sim numbers are the truth** for this build.
- [ ] Q5 New Viam machine for this: name `cap-sim`, in which org/location?
- [ ] Q6 Module stays a **local module** (not in the Registry) for now, so no public namespace is used. OK? (see decision #9)

---

## Phase 0 - Linux setup (about 1 h)
- [ ] [S] Clone `Viam_Lab` on Linux. Clone `CaP_Thesis` next to it (read only, we never write to it here).
- [ ] [S] In app.viam.com: create machine `cap-sim`. Install viam-server on Linux from its setup page.
- [ ] [S] Machine shows **Live** in the app. Paste `systemctl status viam-agent` (or viam-server) output.
- [ ] [CC] Python venv in `Viam_Lab/.venv-linux` with `viam-sdk` (same version as Windows, 0.84.0), numpy.
- [ ] [CC] `.gitignore`: `viam.json`, `.env`, `*.secret*`, venvs, `__pycache__`. Check nothing secret is staged.
- [ ] [CC] `20_sim/.env.example` (address + API key names, no values). Real `.env` stays local.
- [ ] [S] `viam login` works in the Linux terminal (CLI already learned in Viam 102).
- [ ] [CC] `20_sim/scripts/connect_check.py`: connect, print resource names. [S] runs it, pastes output.
- [ ] **GATE 0:** script connects from Linux **and** from Windows to `cap-sim`. Commit "Gate 0".

## Phase 1 - Spikes: check what Viam really does (max 2 h, time-boxed)
Each spike = a tiny config or script, a result line in today's log, and a decision.
- [ ] S1 Arm kinematics: does the built-in `fake` arm (or the `simulated` arm, model `"simulated"`, attributes
      `speed`, `acceleration`, `model-path`) take **our own** kinematics file? Or do we need a small custom arm module
      that serves `get_kinematics`? -> decision #15
- [ ] S2 Fake base: does it track its pose, and does the arm on top move with it in the frame system / 3D scene?
      If not -> we write our own `sim-base` (Phase 4B). -> decision #16
- [ ] S3 Static obstacles: which component type carries them in config (docs page "Configure workspace
      obstacles"). Do **mesh** obstacles show in the 3D scene? -> decision #17
- [ ] S4 Gripper: fake gripper vs our own `vacuum` gripper module (we want `is_holding` + a held-object geometry). -> #18
- [ ] S5 Lidar: in Viam a 2D lidar is usually a **camera that returns a point cloud**. Check the fake camera point
      cloud is enough for wiring. IMU = fake movement sensor. Odometry = movement sensor (wheeled odometry model needs
      encoders/motors - check if it fits a fake base). -> #19
- [ ] S6 Base motion planning: does the motion service plan **base** moves without SLAM (MoveOnMap needs a map)?
      If not, cart moves use the base API (move_straight / spin) + our `drive_to` DoCommand. -> #20
- [ ] **GATE 1:** spike table filled in the log, decisions #15-#20 written. Plan below adjusted if needed. Commit.

## Phase 2 - Numbers from the thesis (one source of truth)
- [ ] [CC] `20_sim/tools/extract_numbers.py`: reads CaP_Thesis (path from an env var, `CAP_THESIS_DIR`),
      writes `20_sim/generated/robot_numbers.json`. Never edited by hand. Header says where every number came from.
      - cart: frame 1000 x 700, frame top 400, wheels 300 dia, 500 apart, casters 120 (`sim/cart_kr6/cart.xml`)
      - arm mount on cart: xyz + rotation (`sim/cart_kr6/arm_mount.xml`)
      - KR6 link chain, joint axes, joint limits (`arm_kr6_body.xml`; cross-check `arm_kinematics.py` ArmChain)
      - vacuum tool: flange -> cup (TCP offset), tool box size (`sim/tools/vacuum/`)
      - sensors: cart + wrist RGBD, lidar, IMU mounts and models (`sim/sensors/*/params.py`, `mount_*.xml`)
      - lab world: room, table, cabinet, columns (`sim/worlds/lab/*.xml`, `sim/cart_kr6/worlds/world_lab.xml`)
- [ ] [CC] Meters -> mm, MuJoCo quaternions -> Viam orientation (orientation vector, degrees). One helper, tested.
- [ ] [S] Read the JSON next to the MJCF: spot-check 5 numbers by eye.
- [ ] **GATE 2:** `robot_numbers.json` committed; spot-check passed.

## Phase 3 - KR6 arm in Viam
- [ ] [CC] KR6 kinematics file in `20_sim/kinematics/` (URDF or Viam SVA JSON, per S1). Joint limits included.
      Link collision geometry = boxes/capsules (Viam SVA supports box, sphere, capsule, cylinder... check per S1).
- [ ] [CC] `20_sim/tools/check_fk.py`: same 20 random joint sets -> TCP from Viam vs TCP from MuJoCo
      (run in the thesis env). Pass = under 1 mm and 0.5 deg.
- [ ] [S-write] The arm block in the machine config: name `kr6`, model, attributes, **frame** with parent = cart.
- [ ] [S] 3D scene: arm stands on the cart in the right place, right way round.
- [ ] [CC] `20_sim/scripts/arm_check.py`: home -> 3 joint targets -> `get_end_position` -> `motion.move` to 3 poses.
- [ ] **GATE 3:** FK check passes, arm_check runs clean, 3D scene looks right. Commit.

## Phase 4 - The cart (base + odometry)
- [ ] [S-write] Cart frame block: `cart` frame in world, box geometry 1000 x 700 x 400 (+ wheels as cylinders if needed).
- [ ] 4A (if S2 says the fake base is enough): fake base `cart`, kinematics/width from the numbers.
- [ ] 4B (if not): [CC] module `20_sim/modules/cap_sim/` with model `sim-base`: diff-drive math
      (wheel 300 mm, track 500 mm), integrates pose over time, `move_straight`, `spin`, `set_velocity`,
      `stop`, `is_moving`, `get_properties` (width, turning radius). Speed/accel limits as attributes.
- [ ] [CC] Odometry movement sensor `odom` (from the base pose): position, linear + angular velocity, orientation.
- [ ] [CC] `20_sim/scripts/base_check.py`: drive a 1 m square. Pass = odometry back at start within 1 mm / 0.1 deg
      (ideal model, so it should close almost exactly).
- [ ] **GATE 4:** square closes; arm frame moves with the cart (or the limit is written down). Commit.

## Phase 5 - Vacuum tool
- [ ] [CC] Gripper `vacuum` (per S4): `grab` = suction on, `open` = off, `is_holding`, `stop`, `get_geometries`.
      DoCommand: `{"command": "seal"}` returns 0..1 (sim: 1 when holding).
- [ ] [S-write] The `grab` method body (small and key).
- [ ] [S-write] Tool frame block: parent `kr6`, TCP offset from the numbers, tool box geometry.
- [ ] [CC] Held box: when holding, add the box to the WorldState so the planner moves it with the tool
      (course recipe `50_reference/how_to_model_held_object.md`).
- [ ] **GATE 5:** pick a box from a pose, move, place, `is_holding` right at each step. Commit.

## Phase 6 - Sensors (wired + framed, fake data)
- [ ] [CC] Cameras `cam_cart`, `cam_wrist`: fake camera, width/height from the D435 numbers, `model: true` for intrinsics.
- [ ] [S-write] Wrist camera frame (parent = tool or `kr6`), cart camera frame (parent = cart).
- [ ] [CC] Lidar `lidar` (per S5): camera with point cloud, frame on the cart.
- [ ] [CC] IMU `imu`: fake movement sensor, frame on the cart.
- [ ] [CC] `20_sim/scripts/sensors_check.py`: read every sensor once, print shapes/values, save one image.
- [ ] [S] 3D scene: every sensor frame sits where the thesis model puts it.
- [ ] **GATE 6:** all sensors readable, frames right. Commit.

## Phase 7 - Lab world obstacles
- [ ] [CC] From `robot_numbers.json`: room walls, table, cabinet, columns as obstacles (per S3), +30 mm margin.
- [ ] [CC] Put the world in a **fragment** `cap-lab-world` (so the lab computer and the real robot reuse it).
- [ ] [CC] Named places as data, not code: table pick spot, place spot, home (`20_sim/configs/locations.json`).
- [ ] [CC] `20_sim/scripts/world_check.py`: plan a move that would go through the table -> planner goes
      around or refuses. Also a pose inside the cabinet -> clear error.
- [ ] **GATE 7:** world shows in the 3D scene, both checks behave. Commit.

## Phase 8 - Primitives: Viam first, DoCommand for the gaps
- [ ] [CC] `20_sim/primitive_map.md`: every thesis primitive (from CaP_Thesis `docs/Research_API_Inventory_2026-09-03.md`
      and the primitive survey) -> the Viam call that covers it, or "DoCommand". First draft:

| Thesis primitive | Viam | Gap -> DoCommand verb |
|---|---|---|
| move_joints | arm `move_to_joint_positions` | - |
| move_to_pose | motion `move` | - |
| move_linear | motion `move` + linear constraint | check (spike in this phase) |
| move_relative | motion `move` from current pose | small helper |
| go_to_named_pose | - | `go_to_named_pose` |
| approach (standoff, then straight in) | - | `approach` |
| suction pick / place | gripper + motion | `pick`, `place` |
| change_tool | - | `change_tool` (stretch) |
| stop_arm / stop_cart | `stop` | - |
| estop (latched, both) | - | `estop`, `release_estop` |
| drive_straight / rotate_in_place | base `move_straight` / `spin` | - |
| drive_arc / drive_velocity | base `set_velocity` | - |
| drive_to_pose / go_to_location | motion on base if S6 allows | else `drive_to`, `go_to_location` |
| queries (tcp pose, odom, seal) | `get_end_position`, movement sensor, gripper | - |
| needs_clarification | - | error answer shape in DoCommand |

- [ ] [S] Approve the map (Shahar decides).
- [ ] [CC] Generic service `cap` in the module: holds the gap verbs. Dependencies: kr6, cart, vacuum, motion, odom.
      Attributes for every number that changes between setups (speeds, standoff_mm, locations file).
      `validate_config` with clear messages (Viam 102 page 9 pattern). Every loop bounded.
- [ ] [S-write] One verb end to end: `go_to_named_pose`.
- [ ] [CC] Unit tests for the pure math (diff-drive, approach line, frames) - run without viam-server.
- [ ] **GATE 8:** every verb called once from a script; bad input -> clear error, no crash. Commit.

## Phase 9 - The demo program + hygiene
- [ ] [CC] `20_sim/scripts/demo_pick_place.py`: home -> drive to table -> approach -> pick -> drive to place spot
      -> place -> home. Prints each step, stops clean on any error.
- [ ] [S] Run it 5 times. Record a 3D-scene screen capture for the log.
- [ ] [CC] Full machine config exported to `20_sim/configs/cap-sim.json` (no secrets) + fragment JSON.
- [ ] [CC] README in `20_sim/` and each subfolder: what is here, how to run, in plain words.
- [ ] [S] Fill today's log + decisions; update `plan.md`.
- [ ] **GATE 9:** demo 5/5, config in git, a fresh clone + README is enough to rebuild. Commit "Gate 9".

## Phase 10 - Stretch (only if time is left)
- [ ] Real sensor data: bridge module that reads the thesis MuJoCo sim's camera/lidar/IMU topics (ROS2) and
      serves them as Viam cameras/sensors. Same names, so scripts don't change.
- [ ] Arm/base driven by the MuJoCo sim through ROS2 (sim_node stays the only owner of the model).
- [ ] Cloud build of the module for linux/amd64 (+ arm64 if the cart computer is a Pi) - private.
- [ ] Not doing: viam-isaac-sim (needs a 10 GB+ Isaac install and a strong RTX card, no gripper support yet).

## Sunday (already planned)
- [ ] Same config + fragment on the strong lab computer. Week summary. Next steps toward the real robot.

---

## Kick-off prompt for Claude Code (paste on Linux)
```
Read Viam_Lab/CLAUDE.md, 00_admin/plan.md, 00_admin/sim_build_checklist.md and the latest log in 40_logs.
We are building the CaP robot in Viam's own sim, phase by phase, as in sim_build_checklist.md.
Rules: stop at every GATE and give me the exact command to run and what "pass" looks like.
I run all commands and paste the output. Final code first, then a block-by-block explanation.
Pieces marked [S-write] are mine: give a hint, not the code, unless I ask.
Check every Viam fact in docs.viam.com and say when it's not verified.
Numbers come only from CaP_Thesis (CAP_THESIS_DIR), never typed by hand. Viam units are mm.
Never delete files (archive with date + reason). No secrets in git. I commit and push.
Start with Phase 0.
```
