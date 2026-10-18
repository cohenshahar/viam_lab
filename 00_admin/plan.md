# Week plan — Viam learning sprint (13–18 Oct 2026)

Budget: ~3 h/day, ~18 h total. Hours are a guide, not a rule. If a topic is more
interesting, go deeper and move the rest.

| Day | Focus | Done when |
| --- | --- | --- |
| Tue 13 | Setup: folder, GitHub repo, Viam account, start Viam 101 | Repo pushed, first course module started, log written |
| Wed 14 | Viam 101 (finished, all 19 steps) + start Viam 102 | Notes per module in 10_course |
| Thu 15 | **Lab day** at the physical robot: record every part, measure, photograph; add its blocks to the lab robot machine in Viam | 30_robot has a full hardware record; lab robot machine has a block for each part |
| Fri 16 | KR6 in Viam (from home): kinematics file, frame system, motion service; continue Viam 102 | KR6 (or a stand-in arm) moves to a target pose in sim |
| Sat 17 | Mobile base: fake base + arm on top of the cart frame (from home) | Script drives the base and reaches with the arm |
| Sun 18 | Strong lab computer: viam-server on Ubuntu, same config; week summary | Same machine config runs on Ubuntu; summary + next steps written |

## Sim build (Thu afternoon to Sat)
Step-by-step checklist with gates: `00_admin/sim_build_checklist.md` (decisions #11-#20).
Thu = lab day (`30_robot/lab_day_checklist.md`), Fri/Sat = sim on the home PC (full demo by Sat night),
Sun = real cart Pi: our script drives the cart (no arm).

## After the sprint: proposal for the meeting with Prof. Shapira (19 Oct 2026)
Status: proposal, nothing decided. Shown as a page: "Viam in the CaP Thesis" (claude.ai artifact, private to Shahar).
| When | What | Done when |
| --- | --- | --- |
| 19-25 Oct | KR6 on the arm-table through Viam (viam-soleng/viam-kuka over TCP/KRL + our KR6 kinematics) | one script moves the real KR6 to 3 poses and back; same script runs on the sim |
| 26 Oct-1 Nov | Less perfect sim: truth vs belief, wheel slip, odometry drift, sim IMU + lidar with noise, crates off position | pallet demo passes with noise on, or fails in a measurable way |
| 2-8 Nov | New workstation (RTX 5080 16 GB, Ryzen 9): Isaac Sim + viam-isaac-sim module vs a MuJoCo bridge | one Viam arm move runs in the chosen physics sim; one-page comparison |
| 9-22 Nov | CaP through Viam: the language model writes programs against the `cap` commands; tasks T1-T3 in sim, T2 on the real arm | results in the Stage A JSON format; one task on hardware |
| In parallel | Real cart on Viam (Pi, motors, measured 0.5 m / 90 deg drives) | the sim's script drives the real cart |
Decisions for the meeting: ROS 2 vs Viam as the primitive layer; Isaac Sim vs MuJoCo bridge; perception (fixed spots / AprilTags / D435 + vision service); lab facts into the thesis model.
New workstation (Gigabyte Aorus Master laptop, RTX 5080 Laptop 16 GB, Ryzen 9, 32 GB RAM, 1 TB, dual boot Ubuntu + Windows) vs Isaac Sim 6.0: meets every requirement (RAM at the minimum, disk at "Ideal"); laptop GPU a little slower than the desktop card. Run Isaac Sim on Ubuntu 22.04 with the validated 580 driver (50-series crashes reported on 590/595); NVIDIA card selected as the active GPU.

## After the sprint (original notes)
- Move this folder into the CaP thesis folder and merge the repo.
- Next stage: real hardware (cart motors, KR6 controller, robot computer).

## Open questions (move to decisions.md when settled)
- Which Viam program / access level do we get? (asked Avery and Oliver, 13 Oct)
- How do we load KR6 kinematics into Viam (URDF vs Viam JSON kinematics)?
- Can Viam work with MuJoCo, or only its own sim? Does it matter for the thesis?
