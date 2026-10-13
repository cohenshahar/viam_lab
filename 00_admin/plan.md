# Week plan — Viam learning sprint (13–18 Oct 2026)

Budget: ~3 h/day, ~18 h total. Hours are a guide, not a rule. If a topic is more
interesting, go deeper and move the rest.

| Day | Focus | Done when |
| --- | --- | --- |
| Tue 13 | Setup: folder, GitHub repo, Viam account, start Viam 101 | Repo pushed, first course module started, log written |
| Wed 14 | Viam 101: app, machines, config, components, services | Notes per module in 10_course |
| Thu 15 | Finish Viam 101 + Python SDK from this computer (connect, read state, move a fake arm) | First own script in 20_sim/scripts runs against a sim machine |
| Fri 16 | KR6 in Viam: kinematics file, frame system, motion service | KR6 (or a stand-in arm) moves to a target pose in sim |
| Sat 17 | Mobile base: fake base + arm on top of the cart frame | Script drives the base and reaches with the arm |
| Sun 18 | Strong lab computer: viam-server on Ubuntu, same config; week summary | Same machine config runs on Ubuntu; summary + next steps written |

## After the sprint
- Move this folder into the CaP thesis folder and merge the repo.
- Next stage: real hardware (cart motors, KR6 controller, robot computer).

## Open questions (move to decisions.md when settled)
- Which Viam program / access level do we get? (asked Avery and Oliver, 13 Oct)
- How do we load KR6 kinematics into Viam (URDF vs Viam JSON kinematics)?
- Can Viam work with MuJoCo, or only its own sim? Does it matter for the thesis?
