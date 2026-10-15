# Archived 15 Oct 2026: the cart-pose trick and the lab-world fragment

**Why:** decision #29. GATE 3 spike S3 failed: Viam's motion service turns every goal into a world goal
(rdk `services/motion/builtin/builtin.go`), so the planner treated cart-pose's x / y / yaw "joints" as part of
the arm and slid the cart (up to 124.6 mm) to reach targets. Viam's own model for a mobile base with an arm
(docs.viam.com/motion-planning/frame-system/mobile-base-arm) is: world frame = the base centre, the base
frame never moves, and the base's place in the room comes from odometry / SLAM.

- `cart_planar.json`: kinematics of the fake arm `cart-pose` (x, y prismatic, yaw revolute, cart body box).
  The cart body box now sits on the `cart` frame itself.
- `fragment_lab-world.json`: room obstacles as fake components with parent `world`. With world = the cart,
  fixed room obstacles in the config would be wrong as soon as the cart drives. The room now lives in
  `20_sim/generated/robot_numbers.json` (world.obstacles), and `capbot_common.arm_move()` places it around
  the cart from the odometry before each arm move (WorldState obstacles).

The `lab-world` fragment in the Viam app (org Robotics_BGU) is no longer used by capbot-sim.
Explainer page made with Claude: "Who moves the cart in Viam?" (claude.ai artifact, 15 Oct 2026).
