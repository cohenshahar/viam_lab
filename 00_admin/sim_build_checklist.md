# Build checklist: lab cart + KR6 in Viam sim, then the real cart (v2)

Written Thu 15 Oct 2026 after Q&A with Shahar. Old draft: `90_archive/2026-10-15_sim_build_checklist_v1.md`.

## Targets
- **Sat night:** the full demo runs in Viam sim: the cart drives to the table, the arm approaches,
  suction-picks a box, the cart drives on, the arm places it, everything goes home. Viam services first,
  our DoCommand verbs only for the gaps.
- **Sunday:** our Python script drives the **real cart** (no arm) through Viam, running on the cart's
  Raspberry Pi 4: forward/back 0.5 m + spin 90 deg both ways, slowly. Anything more is a bonus.

## Settled (decisions #11-#20)
| Topic | Decision |
|---|---|
| Sim kind | Viam's own sim: fake/sim parts + our small modules. No MuJoCo/ROS2 bridge this week |
| Mode | **Efficiency first.** Claude Code writes everything; Shahar runs and approves at gates |
| Code | `Viam_Lab/20_sim/` (sim) and `Viam_Lab/30_robot/` (real cart) |
| Viam services | Use Viam where it exists; thesis primitives Viam lacks become DoCommand verbs. The motion service moves the arm only (it can't drive a base) |
| Scope (sim) | Arm, cart, vacuum, cameras, lidar, IMU, odometry, lab world obstacles. Sensor data is fake, and that's OK this week |
| Org / machines | Org **Robotics_BGU**. Locations `Lab-floor` (machine **capbot**, the cart Pi), `sim` (machine **capbot-sim**, Shahar's PC), `arm-table` (fixed-arm rig). Shared fragment **lab-world** |
| Same names | Both machines use the same component names (`cart`, `left`, `right`, `imu`, ...) so the same script runs on both |
| Numbers | Come from CaP_Thesis + today's lab measurements via one script, never typed by hand. Viam = **mm** |
| Module | Local module, not in the Registry (keeps the public namespace out of lab code) |
| Computers | Today: stav (lab, Ubuntu). Fri/Sat: Shahar's PC (Ubuntu). Sun: the cart Pi + Shahar's PC |

## Time plan
| When | What |
|---|---|
| Thu (lab) | `30_robot/lab_day_checklist.md` (Pi access, survey, SD backup, wiring, measurements). If time: Phase 0 on stav |
| Fri | Phases 0-5 on the home PC |
| Sat | Phases 6-9, then Phase 10 (Sunday script run in sim) |
| Sat (if time) / next week | Phase V (CAD look). Spikes V1-V3 can run any time; the build waits until GATE 9 |
| Sun | Phases 11-13 at the lab with the real cart |

## Thursday status
Written and checked on the Windows PC (no viam-server): numbers + kinematics generator (FK check PASS, 50 poses, 0.000 mm),
both machine configs, the sim module (tests PASS), all check scripts, `cart_sunday.py`, the primitive map.
Items marked *(written Thu, run Fri)* still need their gate run against viam-server.

## How we work
- **[CC]** Claude Code does it. **[S]** Shahar does it (app clicks, running, approving, hardware).
- **GATE** = Claude Code stops and gives the exact command + what "pass" looks like. Shahar runs it, pastes output,
  says pass. Commit with the gate number. Keep gates short: one command each where possible.
- Unverified Viam facts are marked **(verify)**. Claude Code checks docs.viam.com before relying on them.

---

## Phase 0 - Setup (~45 min)
- [x] [S] Org **Robotics_BGU**, locations `Lab-floor` / `arm-table` / `sim`, machines **capbot** and **capbot-sim** created (Thu 11:41).
- [x] [S] On the PC (Ubuntu): install viam-server for **capbot-sim** from its setup page. Machine shows Live.
- [x] [S] Clone Viam_Lab and CaP_Thesis (read only) side by side. Set `CAP_THESIS_DIR` in `~/.bashrc`.
- [x] [S] venv `.venv-linux` with `viam-sdk==0.84.0`, numpy, mujoco 3.8.1 (commands in `20_sim/README.md`). Done Thu evening on the Ubuntu PC (Python 3.10.12); build/check_fk PASS/make_configs re-run there.
- [x] [CC] `.gitignore`: `viam.json`, `.env`, `*.img`, venvs, `__pycache__`. `20_sim/.env.example` with key names only.
- [ ] [S] `viam login` on the PC; `viam organizations list` shows Robotics_BGU.
- [x] [CC] `20_sim/scripts/connect_check.py` (reads address + API key from `.env`, prints resource names). *(written Thu, run Fri)*
- [x] **GATE 0:** connect_check prints the machine's resources. Commit. PASS Thu 14:47 (1 resource: motion builtin).

## Phase 1 - Spikes (mostly answered on Thu from docs + source, see `50_reference/viam_facts_2026-10-15.md`)
Settled on paper (decision #21):
- [x] Arm = `rdk:builtin:simulated` with `model-path` to our SVA JSON, `simulate-time: true`.
- [x] Fake base / fake movement sensor / fake gripper are useless for us -> own `vacuum`; cart = Viam's wheeled base on fake motors + our `odom` (decision #28).
- [x] Motion service can't drive a base -> cart moves from our code; motion service for the arm only.
- [x] ~~Obstacles = fragment `lab-world`~~ -> (decision #29) room obstacles sent with each arm move as WorldState, moved into the cart frame from odom.
Confirmed on the PC (Thu 15 Oct):
- [x] S1 The simulated arm loads `kr6.json` (and `cart_planar.json`) from an absolute path. 7 joint poses match our FK, 0.0000 mm.
- [x] S2 Cart trick worked (0.000 mm) **but is dropped**, because of S3.
- [x] S3 **Planner must not move the cart:** FAILED with the cart trick: Viam's motion service turns every goal into a world goal,
      so the planner used cart-pose's x / y / yaw (slid 124.6 mm). Fix = decision #29: Viam's model, world frame = the cart,
      `odom` for the room position, `arm_move()` sends the room per move. Re-check in GATE 3.
- [x] S4 Fragment obstacles showed in the 3D scene, then dropped for capbot-sim (decision #29).
- [x] **GATE 1:** results in the log (40_logs/2026-10-15.md).

## Phase 2 - Numbers + generated files (done Thu from the Windows PC; re-run on any thesis change)
- [x] [CC] `20_sim/tools/build_viam_files.py` compiles the thesis MuJoCo model (MuJoCo 3.8.1, absolute path) and writes,
      all generated, never edited by hand: `20_sim/generated/robot_numbers.json`, `20_sim/kinematics/kr6.json`,
      (decision #29: `cart_planar.json` and `fragment_lab-world.json` no longer generated; room boxes with margin are in
      `robot_numbers.json` -> `world.obstacles[*].planning_box_mm`; cart body box -> `viam_frames.cart_geometry`).
- [ ] [CC] **Lab overrides:** `30_robot/measured_2026-10-15.json` (today's tape numbers) wins over the sim value for the
      real cart config; the JSON says which won.
- [ ] **GATE 2:** Shahar spot-checks 5 numbers in `robot_numbers.json` against the thesis files.

## Phase 3 - KR6 arm
- [x] [CC] `kr6.json` (SVA): links/joints from the MuJoCo model, joint limits, a box per link from the mesh bounds,
      a last fixed link that turns the flange so **+Z points out of the flange** (Viam's habit).
- [x] [CC] `20_sim/tools/check_fk.py`: our JSON's forward kinematics vs MuJoCo at 50 random joint sets. Pass < 0.1 mm / 0.01 deg.
- [x] [CC] Arm `kr6` in capbot-sim: `rdk:builtin:simulated`, `model-path` absolute, frame parent `cart`, mount from the numbers (`make_configs.py`).
- [x] [CC] `20_sim/scripts/arm_check.py`: named poses -> `get_end_position` vs our kinematics -> cart trick (S2) -> `motion.move` in `cart` (S3). *(written Thu, run Fri)*
- [x] **GATE 3:** arm_check PASS (Thu 15 Oct, after decision #29): 7 joint poses 0.0000 mm; cart = world; 3 targets reached
      (0.000 mm) with the cart at 2 room places; odom unchanged. Commit.

## Phase 4 - Cart (same name `cart` as the real one)
- [x] ~~`cart-pose` fake arm (cart trick)~~ archived, decision #29 (`90_archive/2026-10-15_cart_pose_trick/`).
- [x] [CC] (v2, decision #28) Like the real cart: `cart` = `rdk:builtin:wheeled` base on motors `left` / `right`
      (`rdk:builtin:fake` motors, each with a fake encoder `left-enc` / `right-enc`, real cart's `max_rpm`). Viam's wheeled base
      does distance/angle -> wheel revolutions + rpm from `width_mm` / `wheel_circumference_mm` (whole mm).
      Our module part `odom` (`capbot:sim:wheel-odom`, generic) reads the wheel turns at 20 Hz and does diff-drive odometry
      (the cart's place in the room; not in the frame tree, decision #29). DoCommand on `odom`: `pose` -> x_mm, y_mm, yaw_deg, left_rev, right_rev;
      `set_pose` to teleport. `test_module.py` passes (wheel math copied from the RDK source, 1 m square closes).
      Old `capbot:sim:sim-base`: `90_archive/2026-10-15_sim_base/`. *(written Thu, run Fri)*
- [x] [CC] Frames (decision #29): `cart` parent `world` with the cart body box as its geometry; everything on the cart
      (kr6, lidar, cameras, imu) parent `cart`.
- [x] [CC] `20_sim/scripts/base_check.py`: 1 m square; odom back at start within 1 mm / 0.1 deg; wheel turns match the
      wheeled-base math within 0.01 rev. *(written Thu, run Fri)*
- [x] **GATE 4:** square closes; wheel turns right. Commit. *(live PASS Sat 17: 0.22 mm, wheel turns exact)*

## Phase 5 - Vacuum tool
- [x] [CC] Gripper `vacuum` (`capbot:sim:vacuum`): `grab` = suction on, `open` = off, `is_holding`, `stop`; tool box served as a no-joint kinematics model.
      DoCommand `{"command":"seal"}` -> 0..1. Tests pass. *(written Thu, run Fri)*
- [x] [CC] Tool frame on `kr6`: TCP 491 mm out of the flange + tool box, from the numbers.
- [ ] [CC] Held box moves with the tool in planning (WorldState, as in `50_reference/how_to_model_held_object.md`).
- [ ] **GATE 5:** pick a box at a pose, move it, place it; `is_holding` right at every step. Commit.

## Phase 6 - Sensors (wired, framed, fake data)
- [x] [CC] Cameras `cam-front`, `cam-rear`, `cam-wrist` (fake, `model: true`), frames from the numbers (turned to Viam's camera habit).
- [x] [CC] `lidar` (fake camera = point cloud), `imu` (fake movement sensor), frames on the cart from the numbers.
- [x] [CC] `20_sim/scripts/sensors_check.py`: read every sensor once, print shapes/values, save one image. *(written Thu, run Fri)*
- [ ] **GATE 6:** all readable; frames right in the 3D scene. Commit. *(sensors_check PASS offline Sat 17)*

## Phase 7 - Lab world
- [x] [CC] Room boxes generated from the thesis lab (4 pillars, table, cabinet, floor, +30 mm margin) into `robot_numbers.json`;
      `capbot_common.arm_move()` sends them with each arm move, moved into the cart frame from odom (decision #29).
      **The thesis lab is a sketch** (`sim/worlds/lab/README.md`: no tape has been near the room). Origin = centre of
      the 9 x 6.5 m floor, +X along the 9 m side (thesis convention, kept). Lab measurements go into the thesis first, then re-generate.
- [x] ~~[S] fragment `lab-world` on capbot-sim~~ dropped (decision #29).
- [ ] [CC] Named places as data: `20_sim/configs/locations.json` (home, table pick spot, place spot).
- [x] [CC] `20_sim/scripts/world_check.py`: above the table -> planned; inside the table / cabinet -> refused. *(written Thu, run Fri)*
- [x] **GATE 7:** world_check behaves (above the table planned; inside table / cabinet refused). Commit. *(live PASS Sat 17, side approach; refusals are real collisions)*

## Phase 8 - Primitives: Viam first, DoCommand for gaps
- [x] [CC] `20_sim/primitive_map.md` (draft written Thu) from CaP_Thesis `docs/Research_API_Inventory_2026-09-03.md` + primitive survey. Draft:

| Thesis primitive | Viam | Gap -> DoCommand verb |
|---|---|---|
| move_joints | arm `move_to_joint_positions` | - |
| move_to_pose | motion `move` | - |
| move_linear | motion `move` + linear constraint | check |
| move_relative | motion `move` from current pose | small helper |
| go_to_named_pose | - | `go_to_named_pose` |
| approach (standoff, then straight in) | - | `approach` |
| suction pick / place | gripper + motion | `pick`, `place` |
| stop_arm / stop_cart | `stop` | - |
| estop (latched, arm + cart) | - | `estop`, `release_estop` |
| drive_straight / rotate_in_place | base `move_straight` / `spin` | - |
| drive_arc / drive_velocity | base `set_velocity` | - |
| drive_to_pose / go_to_location | motion on base if S7 allows | else `drive_to`, `go_to_location` |
| queries (tcp pose, odom, seal) | `get_end_position`, movement sensor, gripper | - |
| change_tool | - | skip this week |

- [ ] [S] Approve the map (5 min).
- [ ] [CC] Generic service `cap` in the local module with the gap verbs. Dependencies: kr6, cart, vacuum, motion, odom.
      Every changeable number is an attribute with its unit in the name; `validate_config` with clear messages; every loop bounded.
- [ ] [CC] pytest for the pure math (diff-drive, approach line, frame helpers), runs without viam-server.
- [ ] **GATE 8:** every verb called once; bad input -> clear error, no crash. Commit.

## Phase 9 - The demo
- [ ] [CC] `20_sim/scripts/demo_pick_place.py`: home -> drive to table -> approach -> pick -> drive -> place -> home.
      Prints each step; on any error: stop everything, report the step.
- [ ] [S] Run 5 times; screen-capture one run in the 3D scene for the log.
- [ ] [CC] Export configs to `20_sim/configs/` (capbot-sim.json, fragments; no secrets). READMEs in plain words.
- [ ] **GATE 9:** demo 5/5; a fresh clone + README rebuilds it. Commit "Gate 9".

## Phase V - CAD look in the 3D scene (decision #30)
Goal: the 3D scene shows the real KR6, cart, tool, room and picked box at CAD level, in the thesis colours.
The planner keeps the simple boxes (fast and safe); the meshes are for looking only. No MuJoCo physics or rendering
(that stays the later bridge, decision #11).

How Viam draws things (checked 15 Oct in the docs + RDK source, main):
- **Arm API `get_3d_models`** returns one mesh per link (GLB). Viam's fake arm ships GLBs for UR5e / UR20 / xArm / lite6 only
  (rdk `components/arm/fake/3d_models`); the built-in simulated arm has nothing for a KR6 -> we serve our own.
- **World state store service** (Python SDK 0.84 has it: `viam.services.worldstatestore`) publishes visual-only shapes:
  box / sphere / mesh (PLY; "convert an STL file to PLY first"), colours, see-through; re-sending the same UUID moves it.
  Docs: visualization/visuals-and-collisions, publish-visuals-from-a-module, reference/geometry-constructors.
- URDF collision meshes (STL / PLY) would also draw, but then the planner checks every triangle (slower) -> not our route.
- No URDF needed: the thesis MuJoCo model already places every STL on its link, with colours. STLs: CaP_Thesis
  `sim/rig/assets/urdf/meshes/` (KR6 links) and `sim/cart_kr6/meshes/` (cart). (No .urdf file is in CaP_Thesis.)

Spikes (30-45 min, before building):
- [x] V1 `get_3d_models`: which key does the 3D scene match (link id in `kr6.json`?) and in which frame / units is the mesh
      drawn? Test: a small module arm serving one coloured cube GLB on `link_2`. **(verify)** -> answered from source (Sat 17): key = kinematics link id, drawn at the link box centre, metres.
- [x] V2 World state store from a Python module: one coloured PLY box parented to `cart`; re-send with a new pose -> it moves. **(verify)** -> PLY/STL, metres, vertex colours; stream must open with an empty message (viam-server drops the first).
- [x] V3 Is GLB or PLY lighter for the scene with ~1 MB of KR6 STL? Do the meshes need thinning (decimation)? **(verify)** -> GLB per arm link (32k tris total) and PLY parts (< 1 MB each); no thinning needed.

Build:
- [x] [CC] `build_viam_files.py` also exports, from the compiled thesis MuJoCo model: each KR6 link's meshes in that link's
      Viam frame, the cart meshes in the `cart` frame, the vacuum tool, with the MuJoCo colours ->
      `20_sim/generated/visuals/` (generated, never edited by hand).
- [x] [CC] Module arm `capbot:sim:kr6` in place of `rdk:builtin:simulated`: same `kr6.json` for planning (boxes), timed joint
      moves like `simulate-time`, `get_3d_models` = the link meshes. `arm_check` must still PASS.
- [x] [CC] Module service `capbot:sim:scene` (world state store): cart body meshes; the room (pillars, table, cabinet, floor)
      placed around the cart from `odom`, kept live as the cart drives; the picked box (rides on the tool while held). (done except the picked box: comes with GATE 5)
- [x] [S] Look: 3D scene checked by Shahar.
- [x] **GATE V:** *(Sat 17: Shahar OK; GATE 3/4/7 live PASS)* arm, cart, tool, room and box look like the thesis model; GATE 3 / 4 / 7 checks still PASS. Commit.

## Phase 10 - Sunday script, proven in sim (Sat evening)
- [x] [CC] `30_robot/scripts/cart_sunday.py` (written Thu) - one script, two machines (pick with `--machine sim|real`, address/key from `.env`):
      1. connect, print resources; 2. `--motor-test`: each motor `set_power(+p)` 2 s, then `-p` 2 s, stop (p from `--power`, default 0.25);
      3. `--drive`: `move_straight(500 mm, slow)`, `move_straight(-500 mm)`, `spin(90)`, `spin(-90)`; pause + prompt between steps;
      4. every command in `try/finally: stop()`; a speed cap and a max time per step; Ctrl+C = stop.
- [ ] [CC] For real: before driving, set both **gear pins to low gear** through the board API (verify how: board `gpio_pin_by_name`). Which level = low gear comes from Sunday's test.
- [ ] [S] Run it fully against capbot-sim.
- [ ] **GATE 10:** clean run in sim. Commit. (Sunday only changes `--machine`.)

---

## Sunday - the real cart (no arm)

### Phase 11 - Pi ready (morning, ~1 h)
- [ ] [S] SD image backup exists and its size is right (from Thursday). If not, make it now, **before anything else**.
- [ ] Path A (spare card + reader): Raspberry Pi Imager -> **Raspberry Pi OS Lite 64-bit**; in the Imager settings: hostname `lab-cart`,
      user, SSH on, Wi-Fi (or none if hotspot/Ethernet). Old card goes into a labelled bag.
- [ ] Path B (only the old card): if the survey says 64-bit + recent OS: keep it; stop and **disable** (not delete) the old cart control
      service(s) found in the survey; note exactly what was disabled in the log so it can be turned back on.
      If it's 32-bit: reflash the same card to 64-bit (the backup image is the way back).
- [ ] [S] Turn on I2C; turn **off** the serial console and serial port (pins 8/10 are used for the left motor). `sudo raspi-config` -> Interface Options.
- [ ] [S] Internet works: `curl -I https://app.viam.com` shows a code.
- [ ] **GATE 11:** `ssh` into the Pi; `getconf LONG_BIT` = 64; internet OK; `i2cdetect -y 1` shows 0x68 (if the IMU is wired).

### Phase 12 - capbot config (motors with wheels in the AIR)
- [ ] [S] Install viam-server on the Pi from the **capbot** setup page. Machine Live.
- [ ] [CC] capbot config (+ fragment lab-world):
      - board `board`: `viam:raspberry-pi:rpi4`; pins are named by **physical header number** (checked). PWM = pigpio software PWM.
      - motor `left`: gpio, `pins: {pwm: "12", dir: "10"}`; motor `right`: `pins: {pwm: "33", dir: "35"}`;
        `min_power_pct: 0.25` (dead band from the report), `max_power_pct: 0.40` (safety cap for day one; report max useful ~0.70),
        `max_rpm`: fit on Sunday so `power = rpm/max_rpm` matches reality (report: ~148 rpm low gear at full speed),
        `pwm_freq`: same as the old code (survey) else 800 (Viam default). `move_straight` accuracy = `max_rpm` accuracy (open loop, no encoders).
      - base `cart`: wheeled, `left: ["left"]`, `right: ["right"]`, `width_mm` + `wheel_circumference_mm` from the **measured** values.
      - (bonus) `imu`: MPU6050 movement sensor on I2C bus 1 (verify model name).
- [ ] [S] Safety set-up: drive wheels lifted; second person at the main switch; motor power on last.
- [ ] [S] Motor test in the app (motor card), then `cart_sunday.py --machine real --motor-test --power 0.25`.
      Each wheel turns, right direction? Wrong way -> `dir_flip: true` for that motor. Note which gear-pin level is low gear.
- [ ] **GATE 12:** both wheels spin forward/backward on command and stop on `stop()`, wheels in the air.

### Phase 13 - Drive on the floor
- [ ] [S] Clear area, cart on the floor, person at the switch.
- [ ] [S] `cart_sunday.py --machine real --drive`. Measure the real distance and angle with a tape and floor marks.
- [ ] [CC] Correct `wheel_circumference_mm` / `spin_slip_factor` from the measurements (open loop, no wheel encoders expected).
- [ ] **GATE 13 (Sunday done):** our script drives 0.5 m forward/back and spins 90 deg both ways on the real cart. Commit + log + video.
- [ ] Bonus (surprise): IMU live in Viam; lidar live (if USB and a Viam module exists for its model); keyboard teleop script.

## Week wrap
- [ ] Log, decisions, `plan.md` updated. Next: the KR6 on Viam (real controller), then merge Viam_Lab into CaP_Thesis.

---

## Kick-off prompt for Claude Code (paste on the Ubuntu PC)
```
Read Viam_Lab/CLAUDE.md, 00_admin/plan.md, 00_admin/sim_build_checklist.md, 30_robot/lab_day_checklist.md
and the latest log in 40_logs. This week is efficiency first: you write the code and configs; I run, check
and approve at every GATE. Work phase by phase as in sim_build_checklist.md, starting with Phase 0.
At each GATE: stop, give me the exact command and what "pass" looks like; I paste the output.
Check Viam facts in docs.viam.com and say when something is not verified. Numbers only from CaP_Thesis
(CAP_THESIS_DIR) and 30_robot/measured_*.json, never typed by hand; Viam units are mm.
Never delete files (archive with date + reason). No secrets in git. I commit and push.
Tick the checklist boxes as we go and keep today's log in 40_logs up to date.
```
