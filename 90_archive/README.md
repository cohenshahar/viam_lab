# 90_archive
Old or dropped work. Add a line per file: date, what, why.

- `2026-10-15_lab_day_checklist_v1.md` - first lab-day checklist (14 Oct). Replaced on 15 Oct by a version focused on the cart Pi, because Sunday's goal became driving the real cart from Viam.
- `2026-10-15_sim_build_checklist_v1.md` - first draft of the build checklist (learning mode, open questions). Replaced the same day by v2 after Shahar's answers: efficiency first, two machines + shared fragment, Sunday on the real cart Pi.
- `2026-10-15_sim_base/` - our own sim base `capbot:sim:sim-base` and its test. Replaced the same day by Viam's wheeled base + fake motors `left`/`right` + our odometry part `odom` (decision #28), so the motor math is part of the sim.
- `2026-10-15_cart_pose_trick/` - `cart_planar.json` (fake arm cart-pose) and `fragment_lab-world.json`. Dropped the same day (decision #29): the arm planner moved the cart-pose joints; we follow Viam's model, world frame = the cart, room placed around it from odometry per arm move.
