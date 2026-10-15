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

## After the sprint
- Move this folder into the CaP thesis folder and merge the repo.
- Next stage: real hardware (cart motors, KR6 controller, robot computer).

## Open questions (move to decisions.md when settled)
- Which Viam program / access level do we get? (asked Avery and Oliver, 13 Oct)
- How do we load KR6 kinematics into Viam (URDF vs Viam JSON kinematics)?
- Can Viam work with MuJoCo, or only its own sim? Does it matter for the thesis?
