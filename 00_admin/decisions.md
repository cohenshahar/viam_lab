# Decision log

| # | Date | Decision | Why |
| --- | --- | --- | --- |
| 1 | 2026-10-13 | Project lives in C:\Users\cohen\Documents\Viam_Lab | Folder already set aside for the Viam course |
| 2 | 2026-10-13 | New private GitHub repo Viam_Lab as the bridge between computers; merge into CaP_Thesis at the end | Same flow as the thesis |
| 3 | 2026-10-13 | Sim target: KUKA KR6 on the lab cart | This is the CaP robot |
| 4 | 2026-10-13 | Track progress with one log file per day | Simple, and easy to read back |
| 5 | 2026-10-13 | Learning mode: I write the code myself, Claude hints and reviews | The goal is to learn, not only to get it done |
| 6 | 2026-10-13 | Project language is English | Shahar's choice |
| 7 | 2026-10-13 | Concept first, then Claude does the heavy coding; I read, run and review it (replaces #5) | Saves time in a 6-day sprint; the learning stays in the concepts and the review |
| 8 | 2026-10-14 | Plan change: Viam 101 done on Wed; Thu becomes the lab day at the physical robot; Python SDK work moves into Viam 102 and Fri/Sat | No lab access Fri/Sat, so everything that needs the real robot must be recorded on Thu |
| 9 | 2026-10-14 | Viam org public namespace is `shahartutorial`; course module is `shahartutorial:palletizer-102` | Namespace is public and permanent; fine for the course. Lab/thesis modules may need a cleaner name later (would mean a separate org) |
| 10 | 2026-10-15 | Viam 102 course repo (palletizer-102) on GitHub is public; lab and thesis module repos are private | The course repo is a tutorial; lab code stays private |
| 11 | 2026-10-15 | Build the CaP robot in Viam's own sim (fake/sim parts + own modules); no MuJoCo/ROS2 bridge in this build | Learn Viam's frame system, planning and APIs first; the bridge is a stretch phase |
| 12 | 2026-10-15 | Code for the sim build lives in Viam_Lab/20_sim | Moves into CaP_Thesis with the rest after the sprint |
| 13 | 2026-10-15 | Viam services first; thesis primitives Viam lacks become DoCommand verbs in our module | Use what exists, add only the gaps |
| 14 | 2026-10-15 | Scope: arm, cart, vacuum, cameras, lidar, IMU, odometry, world obstacles. Build steps in 00_admin/sim_build_checklist.md | Full robot + environment; sensor data is placeholder in Viam-only sim |
| 15 | 2026-10-15 | For the sim build: efficiency first. Claude Code writes the code and configs; Shahar runs and approves at gates (replaces #7 for this build) | Two and a half days to a full demo + real cart |
| 16 | 2026-10-15 | Viam org robotics_bgu; machines lab_cart_sim (Shahar's PC, Ubuntu) and lab_cart (cart Pi), one shared fragment, same component names | The same script runs in sim and on the real cart |
| 17 | 2026-10-15 | Sat target: full sim demo (drive to table, suction pick, drive, place, home) | Shahar's choice |
| 18 | 2026-10-15 | Sunday target: our Python script drives the real cart (no arm) via viam-server on the cart Pi: 0.5 m forward/back, spin 90 deg both ways | First real hardware step; everything more is a bonus |
| 19 | 2026-10-15 | Fake sensor data is fine this week | Wiring, frames and program flow matter now; real data later |
| 20 | 2026-10-15 | Before any install on the cart Pi, image its SD card; old card/system kept so we can go back | Only one card for now; the old project's system must not be lost |
| 21 | 2026-10-15 | Names: org Robotics_BGU; locations Lab-floor, arm-table, sim; machines capbot (cart Pi) and capbot-sim; hyphens in names; pillars are obstacles (pillar-front-left ...), not locations | Shahar's choice; hyphens are safe in web addresses |
| 22 | 2026-10-15 | Sim cart = our module base `capbot:sim:sim-base` that writes its pose into a fake arm `cart-pose` (x, y, yaw joints), so the arm rides with the cart | Viam's fake base does nothing, base frames never move, and the motion service can't drive a base (checked in the RDK source) |
| 23 | 2026-10-15 | KR6 = `rdk:builtin:simulated` arm with our generated `kr6.json`; vacuum = our module gripper `capbot:sim:vacuum` | Fake gripper can't hold; the simulated arm moves over time |
| 24 | 2026-10-15 | All Viam numbers are generated from the thesis MuJoCo model (`build_viam_files.py`, MuJoCo 3.8.1) and the configs from those numbers (`make_configs.py`); lab measurements go in `30_robot/measured_*.json` | One source of truth; no hand-typed numbers |
| 25 | 2026-10-15 | World origin = the thesis convention: centre of the lab floor, +X along the 9 m side | Same frame in the thesis sim and in Viam |
| 26 | 2026-10-15 | Real cart day one: power 0.30 (cap 0.27-0.32), low gear, max_rpm fitted to the report table at that power; Viam drives open loop (no encoders) | Slow and predictable first drive; distance accuracy is measured, not assumed |
