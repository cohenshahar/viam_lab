# 20_sim: capbot in Viam's own sim

capbot = the lab cart with the KUKA KR6 and the suction tool. Viam machine **capbot-sim**
(org Robotics_BGU, location `sim`). Real cart = machine **capbot** (`30_robot/`).

## What is where
| Path | What | Edited by hand? |
|---|---|---|
| `tools/build_viam_files.py` | reads the thesis MuJoCo model, writes every number file below | yes (code) |
| `tools/check_fk.py` | our KR6 file vs MuJoCo, 50 random poses | yes (code) |
| `tools/make_configs.py` | writes the machine configs from the numbers | yes (code) |
| `tools/viam_frames.py` | Viam orientation math + forward kinematics | yes (code) |
| `generated/robot_numbers.json` | every robot / room number, mm and degrees, with its source (the room's obstacles too) | **no, generated** |
| `kinematics/kr6.json` | KR6 kinematics for Viam (SVA) | **no, generated** |
| `configs/capbot-sim.json` | the capbot-sim machine config | **no, generated** |
| `generated/visuals/` | CAD meshes for the 3D scene (KR6 link GLBs; tool / cart / room PLYs, metres) | **no, generated** |
| `configs/locations.json` | named cart places, the crates, the pick station and the pallet slots (room frame) | **no, generated** |
| `configs/lab_changes.json` | Shahar's changes to the thesis lab for the sim (table height, crate pallet) | yes |
| `modules/capbot_sim/` | local module: `capbot:sim:kr6` (arm + CAD meshes), `capbot:sim:motor` (drive motors), `capbot:sim:wheel-odom` (`odom`), `capbot:sim:vacuum` (tool), `capbot:sim:scene` (3D scene shapes), `capbot:sim:cap` (the thesis verbs) | yes (code) |
| `scripts/capbot_common.py` | connect, numbers, and `arm_move()`: every arm move goes through it | yes (code) |
| `scripts/*_check.py` | one check per gate | yes (code) |
| `primitive_map.md` | thesis primitives -> Viam calls / our verbs | yes |

## How the sim cart works (and why)
We follow Viam's own model for a mobile base with an arm
([docs](https://docs.viam.com/motion-planning/frame-system/mobile-base-arm/), decision #29):
**Viam's world frame is the cart** (centre of the drive axle, on the floor). The frame tree never moves:
`world -> cart -> kr6 -> vacuum`, plus cameras, lidar, imu on `cart`, and `cam-wrist` on `kr6`.

- **Driving:** `cart` is Viam's **wheeled base** on motors `left` and `right` (our `capbot:sim:motor`, with the
  real cart's `max_rpm`; Viam's fake encoder never counts, decision #31). The wheeled base turns "500 mm" or "90 deg" into revolutions and rpm for each
  wheel, the same math as on the real cart.
- **Where the cart is in the room:** our `odom` part reads how far each wheel turned and works out x, y, yaw
  in the room (diff-drive odometry). Ask it with `{"command": "pose"}`; teleport with `set_pose` (sim only).
  This is not in the frame tree; on the real robot, real odometry or SLAM takes its place.
- **Arm moves:** always through `capbot_common.arm_move()`. It reads odom, moves the room's obstacles
  (from `robot_numbers.json`) into the cart frame, and sends them with the request as WorldState. The planner
  plans the 6 KR6 joints only; it cannot move the cart. Targets are given in the cart frame (`"world"`);
  `room_point_in_cart()` turns a room point into one.
- **3D scene (CAD look, decision #30):** the KR6 serves its link meshes itself (`get_3d_models`), drawn in place of
  the link boxes; the `scene` service draws the cart, the tool and the room (placed around the cart from odom, so
  the room turns and slides around the fixed cart as it drives). Meshes come from the thesis MuJoCo model.
  The planner still uses the plain boxes. In the 3D scene settings, "model" hides the boxes, "colliders+model" shows both.

Why not hang the cart on a fake arm so it drives across the room in the 3D scene? We tried (archived in
`90_archive/2026-10-15_cart_pose_trick/`): the motion service turns every goal into a world goal, so the
planner used the cart's x / y / yaw as extra arm joints and slid the cart to reach.

## Setup on the Ubuntu PC (Friday, once)
```bash
cd ~/robotics/viam_lab                          # the clone on this PC
python3 -m venv .venv-linux                     # Python 3.10.12 on this PC (Ubuntu 22.04)
.venv-linux/bin/pip install viam-sdk==0.84.0 numpy mujoco==3.8.1
chmod +x 20_sim/modules/capbot_sim/run.sh
echo 'export CAP_THESIS_DIR=/home/shahar/robotics/CaP_Thesis' >> ~/.bashrc && source ~/.bashrc
cp 20_sim/.env.example 20_sim/.env              # then fill in SIM_ADDRESS / keys from the app
```
Ubuntu's `~/.bashrc` stops early in non-interactive shells (scripts, Claude Code's terminal), so
`CAP_THESIS_DIR` may be missing there. Set it on the same line instead:
`CAP_THESIS_DIR=/home/shahar/robotics/CaP_Thesis .venv-linux/bin/python 20_sim/tools/build_viam_files.py`.
Re-make the files **on this PC** (the sim config holds absolute paths of this PC):
```bash
.venv-linux/bin/python 20_sim/tools/build_viam_files.py
.venv-linux/bin/python 20_sim/tools/check_fk.py          # must say PASS
.venv-linux/bin/python 20_sim/tools/make_configs.py
```
In the app: capbot-sim -> CONFIGURE -> JSON -> paste all of `configs/capbot-sim.json` -> Save.
After a change to the module's code: `sudo systemctl restart viam-agent` (viam-server keeps the old module running).

## Checks (one per gate)
```bash
cd 20_sim/scripts
../../.venv-linux/bin/python connect_check.py --machine sim   # GATE 0
../../.venv-linux/bin/python arm_check.py                     # GATE 3 (+ spike S3)
../../.venv-linux/bin/python base_check.py                    # GATE 4
../../.venv-linux/bin/python sensors_check.py                 # GATE 6
../../.venv-linux/bin/python world_check.py                   # GATE 7
../../.venv-linux/bin/python pick_place_check.py              # GATE 5
../../.venv-linux/bin/python verbs_check.py                   # GATE 8
../../.venv-linux/bin/python demo_pick_place.py --runs 5      # GATE 9 (the demo; watch the 3D SCENE)
../../.venv-linux/bin/python sensor_snapshot.py               # what the sensors report (saved to out/)
```

## The demo and the verbs
`cap` (a generic service) holds the thesis primitives Viam lacks, as DoCommand verbs (`20_sim/primitive_map.md`):
`await cap.do_command({"command": "pick", "box": "box1"})`. From a script: `capbot_common.cap(robot, "pick", box="box1")`.
Verbs answer `{"ok": true, ...}` or `{"error": ..., "needs_clarification": true|false}`.
The demo, like the Viam course palletizer: 8 crates (120 mm cubes) come one at a time to the pick station on the
lab table (top lowered to 0.55 m, `configs/lab_changes.json`). The cart parks side-on (crate 480 mm to its left), the
arm picks the crate from above, the cart drives to the crate pallet (a 200 mm stand in the back-left of the lab) and
the arm lowers the crate into the next slot from above: 2 x 2 grid, 2 layers. While carried, the crate rides on
`vacuum` in planning (a WorldState transform) and in the 3D scene.
No collisions: the arm planner checks every arm move; before every cart move the arm is tucked in and `drive_to`
checks the cart's swept path against the room and the crates; the demo also runs `collision_audit` (own kinematics)
after every step. `demo_pick_place.py --crates 2` for a short run.
Tests (no viam-server needed):
```bash
.venv-linux/bin/python 20_sim/modules/capbot_sim/tests/test_cap_math.py
.venv-linux/bin/python 20_sim/modules/capbot_sim/tests/test_cad.py
.venv-linux/bin/python 20_sim/modules/capbot_sim/tests/test_sim_motor.py
.venv-linux/bin/python 20_sim/scripts/test_room_frame.py
.venv-linux/bin/python 20_sim/modules/capbot_sim/tests/test_drive_math.py
.venv-linux/bin/python 20_sim/modules/capbot_sim/tests/test_module.py
```

Offline test against a private viam-server (no cloud, no keys), useful before pasting a config:
add `"network": {"bind_address": "localhost:8081"}` to a copy of `capbot-sim.json`, start
`/opt/viam/bin/viam-server -config that-copy.json`, then run any check with `CAPBOT_LOCAL=localhost:8081`.

## Known limits
- The thesis lab room is a sketch, not measured (`CaP_Thesis/sim/worlds/lab/README.md`).
- The wrist camera sits on link 5 in the thesis; in Viam it hangs on the arm's end, so it turns with joint a6.
- Sensors return fake data this week.
- The room is not drawn in the 3D scene (it is sent with each arm move).
- The fake motors skip `min_power_pct` / `max_power_pct`; the real cart's slowest speeds will be faster than asked.
