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
| `generated/robot_numbers.json` | every robot / room number, mm and degrees, with its source | **no, generated** |
| `kinematics/kr6.json`, `kinematics/cart_planar.json` | Viam kinematics (SVA) | **no, generated** |
| `configs/fragment_lab-world.json` | room obstacles for the `lab-world` fragment | **no, generated** |
| `configs/capbot-sim.json` | the capbot-sim machine config | **no, generated** |
| `modules/capbot_sim/` | local module: `capbot:sim:sim-base` (cart), `capbot:sim:vacuum` (tool) | yes (code) |
| `scripts/*_check.py` | one check per gate | yes (code) |
| `primitive_map.md` | thesis primitives -> Viam calls / our verbs | yes |

## How the sim cart works (and why)
Viam's fake base does nothing, and a base frame never moves in Viam's frame system. So:
`cart-pose` is a fake **arm** with 3 joints (x mm, y mm, yaw deg; `cart_planar.json`). Our `cart` base
keeps the pose and writes it into those joints 20 times a second. Everything on the cart hangs under it:
`world -> cart-pose -> cart -> kr6 -> vacuum`, plus cameras, lidar, imu. Drive the cart and the arm rides along.
The motion service plans the **arm** only; cart moves come from the base API (and our `drive_to` later).

## Setup on the Ubuntu PC (Friday, once)
```bash
cd ~/Viam_Lab                                   # wherever you cloned it
python3 -m venv .venv-linux
.venv-linux/bin/pip install viam-sdk==0.84.0 numpy mujoco==3.8.1
chmod +x 20_sim/modules/capbot_sim/run.sh
echo 'export CAP_THESIS_DIR=~/CaP_Thesis' >> ~/.bashrc && source ~/.bashrc   # your clone path
cp 20_sim/.env.example 20_sim/.env              # then fill in SIM_ADDRESS / keys from the app
```
Re-make the files **on this PC** (the sim config holds absolute paths of this PC):
```bash
.venv-linux/bin/python 20_sim/tools/build_viam_files.py
.venv-linux/bin/python 20_sim/tools/check_fk.py          # must say PASS
.venv-linux/bin/python 20_sim/tools/make_configs.py
```
In the app:
1. FRAGMENTS -> Create fragment `lab-world` -> paste `configs/fragment_lab-world.json` -> Save.
2. capbot-sim -> CONFIGURE -> JSON -> paste `configs/capbot-sim.json` (drop the `_note` line) -> Save.
3. capbot-sim -> CONFIGURE -> add fragment `lab-world`.

## Checks (one per gate)
```bash
cd 20_sim/scripts
../../.venv-linux/bin/python connect_check.py --machine sim   # GATE 0
../../.venv-linux/bin/python arm_check.py                     # GATE 3 (+ spikes S2, S3)
../../.venv-linux/bin/python base_check.py                    # GATE 4
../../.venv-linux/bin/python sensors_check.py                 # GATE 6
../../.venv-linux/bin/python world_check.py                   # GATE 7
```
Module tests (no viam-server needed):
```bash
.venv-linux/bin/python 20_sim/modules/capbot_sim/tests/test_drive_math.py
.venv-linux/bin/python 20_sim/modules/capbot_sim/tests/test_sim_base.py
```

## Known limits
- The thesis lab room is a sketch, not measured (`CaP_Thesis/sim/worlds/lab/README.md`).
- The wrist camera sits on link 5 in the thesis; in Viam it hangs on the arm's end, so it turns with joint a6.
- Sensors return fake data this week.
- Not yet tested against a running viam-server: the arm loading `kr6.json` (S1), the cart trick (S2),
  the planner leaving the cart alone (S3), fragment obstacles (S4), the vacuum's no-joint kinematics.
