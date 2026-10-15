# Viam facts checked on 15 Oct 2026

Read from the Viam docs and the RDK (Viam's robot software) source on GitHub, `main` branch, 15 Oct 2026.
**V** = verified in docs or source. **NV** = not verified, best reading. Re-check when a version changes.

## What changed our plan
1. **The motion service can't drive a base.** The builtin `MoveOnMap` / `MoveOnGlobe` return "not supported by builtin",
   and a base frame is fixed in the frame system. -> Cart moves come from our own code (base API + our verbs). (V)
2. **The fake base does nothing.** `move_straight`, `spin`, `set_velocity` are no-ops, no pose tracking, child frames
   (the arm) don't move with it. -> We write our own sim base. (V)
3. **The fake movement sensor returns fixed numbers** (a GPS point in New York, speed 5.4). Useless as odometry. (V)
4. **`wheeled-odometry` needs motors with encoders** and fails on a fake base. (V)

## Arm
- Fake arm `rdk:builtin:fake`: attribute `model-path` (or `arm-model`, not both). Files `.json` (Viam SVA) or `.urdf`. Jumps to targets instantly. (V)
- **Simulated arm `rdk:builtin:simulated`**: same `model-path`, plus `speed` (rad/s, default 1.0), `acceleration`,
  `path-tolerance`, **`simulate-time: true`** (needed, otherwise it never moves on its own). Moves over time. (V)
- `model-path` is read on the computer running viam-server; use an absolute path. (V for the read, NV for relative paths)

## Kinematics file (SVA JSON)
- Top: `name`, `kinematic_param_type: "SVA"`, `links[]`, `joints[]`. Units **mm and degrees**. (V)
- Link: `id`, `parent`, `translation{x,y,z}`, optional `orientation`, optional `geometry`. (V)
- Joint: `id`, `type` (`revolute` / `prismatic`), `parent`, `axis{x,y,z}`, `min`, `max`. (V)
- Geometry: `box` (x,y,z), `sphere` (r), `capsule` (r, l, l >= 2r), `cylinder`, with its own `translation` / `orientation`. (V)
- URDF also works, and URDF can use `.stl` / `.ply` meshes for collision. (V)

## Obstacles (fixed things in the room)
- Docs page "Configure workspace obstacles": a component whose `frame` has a `geometry`, parent `world`, mm, centre of the shape. (V)
- Simplest: `rdk:component:generic` with model `rdk:builtin:fake`, one shape each. (NV that the docs use exactly this; the fake generic model exists, V)
- Or the `erh:vmodutils:obstacle` module (several shapes per component). (V)
- No clearance added by the planner: make shapes 20-50 mm bigger. (V)

## Fragments
- Hold components, services, modules; machines list them under `fragments`, overrides with `fragment_mods` (`$set`, `$unset`). (V)
- Frames inside fragment components, and machine parts using a fragment part as frame parent. (NV, should work)

## Gripper, sensors (fake)
- Fake gripper: `grab` returns false, `is_holding` false. -> We write our own `vacuum`. (V)
- Fake camera: same gradient image every time + a point cloud. (V)

## Real cart (Sunday)
- Pi 4 board: module `viam:raspberry-pi`, model **`viam:raspberry-pi:rpi4`**. (V)
- **Pins are named by the physical header number** as text (`"12"`), or `"io18"` for the chip (BCM) number. (V)
- PWM is software PWM through the pigpio service, default **800 Hz**. (V)
- 64-bit OS recommended; module supports bullseye and bookworm. (V)
- GPIO motor `rdk:builtin:gpio`: `board`, `pins{pwm,dir}`, `max_rpm` (needed without encoder), `min_power_pct`,
  `max_power_pct` (0.06-1.0), `pwm_freq`, `dir_flip`. (V)
  - Power is **clipped** to max_power_pct, and any small non-zero power is **raised** to min_power_pct. (V)
  - Without encoder: `go_for` = power rpm/max_rpm for a computed time, then stop. Open loop. (V)
- Wheeled base `rdk:builtin:wheeled`: `left[]`, `right[]`, `width_mm`, `wheel_circumference_mm`, `spin_slip_factor`. (V)
  - `move_straight` / `spin` use `go_for` on each motor, so **distance accuracy depends on `max_rpm`**. (V)
  - `set_power` works with no `max_rpm`; `set_velocity` needs it. (V)
- MPU6050: module `viam:tdk-invensense`, model `viam:tdk-invensense:mpu6050`, `i2c_bus: "1"`. Gives turn rate and
  acceleration only, **no orientation**. Alternate-address attribute name unclear (NV).
- Python SDK: `pin = await board.gpio_pin_by_name("8")`, `await pin.set(True)`. (V)

## Sources
- RDK source: https://github.com/viamrobotics/rdk (components/arm/fake, components/arm/sim, components/base/fake,
  components/base/wheeled, components/motor/gpio, components/movementsensor/wheeledodometry,
  services/motion/builtin/builtin.go, referenceframe/model_json.go)
- https://docs.viam.com/motion-planning/obstacles/configure-workspace-obstacles/
- https://docs.viam.com/fleet/reuse-configuration/
- https://docs.viam.com/reference/components/motor/gpio/
- https://docs.viam.com/reference/components/base/wheeled/
- https://docs.viam.com/reference/device-setup/rpi-setup/
- https://github.com/viam-modules/raspberry-pi
- https://github.com/viam-modules/tdk-invensense
