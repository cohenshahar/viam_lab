# Archived 15 Oct 2026: our own sim base (`capbot:sim:sim-base`)

**Why:** decision #28. The sim cart now uses Viam's own wheeled base (`rdk:builtin:wheeled`) driving
two fake motors `left` / `right` with fake encoders, like the real cart. The wheeled base does the
distance/angle -> wheel revolutions math itself; our module only does the odometry
(`20_sim/modules/capbot_sim/src/wheel_odom.py`, model `capbot:sim:wheel-odom`, component `odom`).

The old base skipped the motors: it turned "drive 500 mm" straight into a pose, so the motor math
was never part of the sim. The `pose` / `set_pose` DoCommand moved from `cart` to `odom`.
