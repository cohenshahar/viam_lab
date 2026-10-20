# Show the two sims (Ubuntu runs them, any computer watches)

For a meeting or a report: the strong Ubuntu PC runs both sim machines, and you watch them in the browser
on any computer (for example the Windows PC). Setup on the Ubuntu PC was done once already (`README.md`).

| Machine | What | Started by |
|---|---|---|
| **capbot-sim** (location `sim`) | the cart + KR6 + suction tool in the lab room | viam-agent (a service, starts at boot) |
| **armtable-sim** (location `arm-table`) | the arm arena: KR6 on the 2 x 2 m table, one box | `scripts/run_armtable_sim.sh` (only while its terminal is open) |

## 1. Ubuntu PC: update and start (terminal 1)
```bash
cd ~/robotics/viam_lab && git pull
sudo systemctl restart viam-agent              # capbot-sim
systemctl status viam-agent --no-pager | head -5   # should say "active (running)"
cd 20_sim/scripts && ./run_armtable_sim.sh     # armtable-sim; leave this terminal open
```
If `git pull` changed files in `20_sim/modules/`, the restart above already loads the new module code.
If it changed `configs/capbot-sim.json` or `configs/armtable-sim.json`, paste that file into the machine's
CONFIGURE -> JSON in the app and Save.

## 2. Watching computer: open the app
1. Browser -> app.viam.com, same Viam account, org **Robotics_BGU**.
2. Open **capbot-sim** and **armtable-sim** in two tabs. Both should show as live (online).
3. In each one open the **3D SCENE** tab.

The page connects to the Ubuntu PC over the internet, so both computers need internet (same network not needed).
The scene may lag a little behind the real moves.

## 3. Ubuntu PC: run the demos (terminal 2)
```bash
cd ~/robotics/viam_lab/20_sim/scripts
../../.venv-linux/bin/python connect_check.py --machine sim     # quick check: connects, lists parts
../../.venv-linux/bin/python connect_check.py --machine arena
```
Short runs first (check that the 3D scenes move on the watching computer):
```bash
../../.venv-linux/bin/python arena_demo.py --cycles 1           # about 25 s
../../.venv-linux/bin/python demo_pick_place.py --crates 2      # about 2-3 min
```
Full runs for the meeting:
```bash
../../.venv-linux/bin/python arena_demo.py --cycles 3           # about 1.5 min
../../.venv-linux/bin/python demo_pick_place.py --crates 8      # about 9.5 min
```
Each demo ends with PASS and a clean collision audit after every step.

## 4. If something fails
- Machine not live in the app: check terminal 1 (armtable-sim) or `systemctl status viam-agent` (capbot-sim).
- `Missing 20_sim/.env`: the keys file is only on the Ubuntu PC (`~/robotics/viam_lab/20_sim/.env`, never in git).
- Backup: the videos `media/demo_pallet_2026-10-18_4x.mp4` and `media/demo_arena_2026-10-18_4x.mp4`.

## 5. Stop
- armtable-sim: Ctrl+C in terminal 1.
- capbot-sim keeps running as a service (it is fine to leave it). To stop it: `sudo systemctl stop viam-agent`.
