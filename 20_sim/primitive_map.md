# Primitive map: thesis primitives -> Viam (approved by Shahar 17 Oct, verbs kept as listed; decision #33)

Source list: CaP_Thesis `docs/Research_Primitives_Survey_2026-09-24.md` (implementation status, 01/10/2026).
Rule (decision #13): if Viam has it, use Viam. If not, it becomes a DoCommand verb on our service `cap`.
"(V)" = checked in the Viam source or docs on 15 Oct; "(NV)" = not checked yet.

## Frames: the thesis and Viam use the names differently
| Thesis frame | Meaning in the thesis | Viam frame here |
|---|---|---|
| `world` | odometry frame, start pose = origin | `world` (in the sim: centre of the lab floor; the cart starts where `set_pose` puts it) |
| `cart` | **the arm base**, 403.9 mm above the floor, x forward, z up | `kr6_origin` (Viam's frame at the arm's base) (NV, name to check) |
| (none) | point on the floor under the drive axle | `cart` (our cart frame) |
| `tcp` | suction cup face | `vacuum` |
So a thesis call `move_to_pose(p, frame='cart')` becomes `motion.move("vacuum", PoseInFrame("kr6_origin", p))`.

## Arm and tool
| Thesis primitive | Viam | Gap -> verb on `cap` | Notes |
|---|---|---|---|
| `move_joints` | arm `move_to_joint_positions` (degrees) | - | thesis uses radians |
| `get_joint_positions` | arm `get_joint_positions` | - | |
| `get_tcp_pose(frame)` | `robot.get_pose("vacuum", frame)` | - | |
| `solve_ik` | - | `solve_ik` | Viam has no public IK call; plan-only `motion.move` dry run is not offered (NV). Low priority |
| `move_to_pose` | `motion.move("vacuum", PoseInFrame)` | - | |
| `move_linear` (`stop_force_n`, `max_travel_m`) | `motion.move` + `LinearConstraint` | `move_linear` for the force stop | no force sensor in Viam sim -> force stop not possible this week |
| `move_relative` | `motion.move` from the current pose | `move_relative` | small helper (current pose + offset) |
| `go_to_named_pose` | - | `go_to_named_pose` | table = thesis `arm_named_poses.json` (home_folded, home_arm_up, ready_high, ready_front) |
| `approach` | 2 x `motion.move` (standoff, then linear in) | `approach` | standoff along the tool axis |
| `suction_pick` | approach + gripper `grab` + retreat | `pick` | held box added to WorldState while carried |
| `suction_place` | approach + gripper `open` + retreat | `place` | |
| `vacuum_on` / `vacuum_off` | gripper `grab` / `open` | - | |
| `get_tool_seal` | gripper DoCommand `seal` | - | sim: 1.0 while holding |
| `stop_arm` | arm `stop` | - | |
| `get_mounted_tool`, `available_tools`, `change_tool` | - | skip this week | one tool only |
| `get_tool_wrench`, `zero_tool_wrench` | - | skip this week | no force sensor in Viam sim |

## Cart
| Thesis primitive | Viam | Gap -> verb | Notes |
|---|---|---|---|
| `drive_straight` | base `move_straight` | - | |
| `rotate_in_place` | base `spin` | - | |
| `drive_velocity` | base `set_velocity` | - | |
| `drive_arc` | base `set_velocity` for a time | `drive_arc` | small helper (radius + angle -> v, w, t) |
| `stop_cart` | base `stop` | - | thesis returns the stopping distance; Viam doesn't |
| `estop_cart` / `release_estop` | - | `estop`, `release_estop` | latch in `cap`: refuses every motion verb until released; stops arm + cart |
| `get_cart_odometry` | sim: cart DoCommand `pose`, or `robot.get_pose("cart","world")`; real: none yet (no encoders) | - | |
| `get_location_names` | - | `locations` | table = `20_sim/configs/locations.json` |
| `drive_to_pose` | **motion service can't drive a base** (V) | `drive_to` | turn, straight, turn (same recipe as the thesis `cart_utils`) |
| `go_to_location` | - | `go_to_location` | location table -> `drive_to` |

## Sensors
| Thesis primitive | Viam | Gap | Notes |
|---|---|---|---|
| `get_camera_frame`, `select_camera` | camera `get_images` on `cam-front` / `cam-rear` / `cam-wrist` | - | fake gradient image this week |
| `get_range_scan` | camera `get_point_cloud` on `lidar` | - | fake point cloud this week |
| `get_cart_imu` | movement sensor `imu` | - | fake values in sim; real MPU6050 has no orientation (V) |
| `pause(seconds)` | - | - | plain `asyncio.sleep` in the program |

## Infrastructure in the thesis that has no Viam twin this week
- `needs_clarification` answers -> every `cap` verb answers `{"error": ..., "needs_clarification": true/false}` instead of raising.
- Teleop priority / operator override -> not this week.
- Pose guard (thesis `pose_guard.py`) -> Viam's own collision checking with the kr6/cart/obstacle geometry.

## Built (17 Oct)
The `cap` service (`20_sim/modules/capbot_sim/src/cap.py`) has the verbs below; `verbs_check.py` calls each once.
Differences from the drafts above, found while building:
- Frames: Viam's world frame is the cart (decision #29), so poses are given in `"world"` (cart floor centre, x forward)
  or `"room"`; not `kr6_origin`.
- `pick` / `place` are side picks (the 491 mm tool pointing down cannot reach the top of the 1 m table).
- Odometry: `odom` DoCommand `pose` (not the cart's).
- Added `reset_boxes` (sim only: the demo box back to its start).
- `solve_ik`, `move_linear` force stop, `drive_to` obstacle check: not built.

## Viam functions that could replace or improve verbs (for Shahar to decide; nothing changed yet)
Checked in the Python SDK 0.84 (V) or the docs/source (V); none of these is used yet.
| Viam function | What it gives | Could help |
|---|---|---|
| Arm `move_through_joint_positions(..., MoveOptions(max_vel_degs_per_sec, max_acc_degs_per_sec2, max_tcp_speed))` | speed / acceleration limits per move (V) | thesis speed arguments on `move_joints`, `move_to_pose`, a slow `approach` |
| Motion `OrientationConstraint(orientation_tolerance_degs)` | keeps the tool orientation along the whole path (V) | carrying the box level; `move_linear` |
| Motion `PseudolinearConstraint(line_tolerance_factor, ...)` | a "roughly straight" path, easier to plan than strict linear (V) | `approach` when strict linear fails |
| Motion `get_plan`, `list_plan_statuses`, `stop_plan` | follow / stop a running plan (V; mainly for base moves in Viam) | thesis `stop_arm` with a reason, status display |
| `robot.transform_pose(pose, frame)` | any pose into any frame through the frame system (V) | `get_tcp_pose(frame)` without our own FK |
| Arm `move_through_joint_positions_streamed` | stream joint points, get progress back (V) | teleop / operator override later |
| Vision service (detections, 3D segmentation) | find objects in camera images (V, service exists) | finding the box with the real D435 instead of a fixed spot |
| Data capture (configured in the app) | records sensor / arm data to the cloud (V, docs) | thesis experiments: log every run |
| Sensor-controlled base (`rdk:builtin:sensor-controlled`) | closes the loop on spins / straights with an IMU (V, docs) | the real cart (no wheel encoders) |

## `cap` service verbs (built 17 Oct)
`status`, `go_to_named_pose`, `approach`, `pick`, `place`, `move_relative`, `drive_to`, `go_to_location`,
`locations`, `drive_arc`, `estop`, `release_estop`. Every loop bounded; every number an attribute with its unit.
