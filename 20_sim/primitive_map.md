# Primitive map: thesis primitives -> Viam (DRAFT, Shahar approves at Phase 8)

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

## `cap` service verbs (to build in Phase 8)
`status`, `go_to_named_pose`, `approach`, `pick`, `place`, `move_relative`, `drive_to`, `go_to_location`,
`locations`, `drive_arc`, `estop`, `release_estop`. Every loop bounded; every number an attribute with its unit.
