"""Write the Viam machine configs for capbot-sim (sim) and capbot (the real cart Pi).

Numbers come only from:
  20_sim/generated/robot_numbers.json   (made by build_viam_files.py from the thesis model)
  30_robot/measured_*.json              (lab tape numbers; win for the real cart when filled in)
  30_robot/cart_report_calibration.json (2024 report speed table, for the real motors)

Run ON THE COMPUTER THAT RUNS viam-server for capbot-sim, because the sim config
holds absolute paths to the kinematics files and the module on that computer:
    python3 20_sim/tools/make_configs.py
Then paste 20_sim/configs/capbot-sim.json into the app (CONFIGURE -> JSON), and
20_sim/configs/capbot.json into capbot on Sunday.
"""

import json
import math
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
NUMBERS = REPO / "20_sim/generated/robot_numbers.json"
CAL = REPO / "30_robot/cart_report_calibration.json"

# Real-cart safety settings for day one (decisions in sim_build_checklist.md, Phase 12)
OPERATING_POWER = 0.30     # the one power we drive at on Sunday, fraction of full PWM
MIN_POWER = 0.27           # just above the dead band (report: wheels start near 69/255 = 0.27)
MAX_POWER = 0.32           # hard cap in the motor config for the first day
PWM_FREQ_HZ = 800          # Viam default; replace with the old code's value if the survey finds one


def load(p):
    return json.loads(Path(p).read_text(encoding="utf-8"))


def latest_measured():
    files = sorted((REPO / "30_robot").glob("measured_*.json"))
    return (files[-1], load(files[-1])) if files else (None, {})


def frame(parent, t, orientation=None, geometry=None):
    """A Viam frame block. Viam's world frame is the cart centre on the floor (decision #29)."""
    f = {"parent": parent, "translation": {"x": t[0], "y": t[1], "z": t[2]}}
    f["orientation"] = orientation or {"type": "ov_degrees", "value": {"x": 0, "y": 0, "z": 1, "th": 0}}
    if geometry:
        f["geometry"] = geometry
    return f


def sim_config(n):
    kin = REPO / "20_sim/kinematics"
    visuals = REPO / "20_sim/generated/visuals"
    cart, s, vf = n["cart"], n["sensors"]["frames"], n["viam_frames"]
    # Viam's world frame = the cart (decision #29). The room is not in the config: capbot_common.arm_move()
    # places it around the cart from odom before each arm move.
    cam = lambda name, parent, fr: {
        "name": name, "api": "rdk:component:camera", "model": "rdk:builtin:fake",
        # no "model": with lens data the fake gradient image becomes a point cloud, drawn as a big plane in the 3D scene
        "attributes": {"width": 640, "height": 480},
        "frame": frame(parent, fr["translation_mm"], fr["orientation"]),
    }
    # Like the real cart: Viam's wheeled base drives motors left / right. Here they are our sim motors
    # (smooth turn count; Viam's fake encoder never counts, decision #31), with the real cart's max_rpm.
    # The wheeled base wants whole mm.
    width, circ = int(round(cart["track_width_mm"])), int(round(cart["drive_wheel_circumference_mm"]))
    max_rpm, _ = max_rpm_for(OPERATING_POWER)
    motor = lambda name: {"name": name, "api": "rdk:component:motor", "model": "capbot:sim:motor",
                          "attributes": {"max_rpm": max_rpm}}
    return {
        "modules": [{"type": "local", "name": "capbot-sim-module",
                     "executable_path": str(REPO / "20_sim/modules/capbot_sim/run.sh")}],
        "components": [
            motor("left"),
            motor("right"),
            {"name": "cart", "api": "rdk:component:base", "model": "rdk:builtin:wheeled",
             "attributes": {"left": ["left"], "right": ["right"], "width_mm": width,
                            "wheel_circumference_mm": circ, "spin_slip_factor": 1.0},
             "frame": frame(vf["cart_parent"], [0, 0, 0], geometry=vf["cart_geometry"])},
            {"name": "odom", "api": "rdk:component:generic", "model": "capbot:sim:wheel-odom",
             "attributes": {"left_motor": "left", "right_motor": "right",
                            "width_mm": width, "wheel_circumference_mm": circ, "tick_hz": 20}},
            # our KR6 model: same kr6.json (boxes for planning) + CAD meshes for the 3D scene (decision #30)
            {"name": "kr6", "api": "rdk:component:arm", "model": "capbot:sim:kr6",
             "attributes": {"kinematics_path": str(kin / "kr6.json"), "visuals_dir": str(visuals),
                            "speed_deg_per_sec": 57.3},
             "frame": frame(vf["kr6_parent"], vf["kr6_translation_mm"])},
            {"name": "vacuum", "api": "rdk:component:gripper", "model": "capbot:sim:vacuum",
             "attributes": {"tool_box_mm": {k: vf["vacuum_geometry"][k] for k in ("x", "y", "z", "translation")}},
             "frame": frame(vf["vacuum_parent"], vf["vacuum_translation_mm"])},
            cam("cam-front", "cart", s["rgbd_front"]),
            cam("cam-rear", "cart", s["rgbd_rear"]),
            cam("cam-wrist", "kr6", s["rgbd_wrist_on_flange"]),
            {"name": "lidar", "api": "rdk:component:camera", "model": "rdk:builtin:fake",
             "attributes": {"width": 320, "height": 240},
             "frame": frame("cart", s["lidar"]["translation_mm"], s["lidar"]["orientation"])},
            {"name": "imu", "api": "rdk:component:movement_sensor", "model": "rdk:builtin:fake",
             "attributes": {}, "frame": frame("cart", s["imu"]["translation_mm"], s["imu"]["orientation"])},
        ],
        # CAD look for the 3D scene: cart, tool and room drawn from the thesis meshes (decision #30)
        "services": [{"name": "scene", "api": "rdk:service:world_state_store", "model": "capbot:sim:scene",
                      "attributes": {"visuals_dir": str(visuals), "odom": "odom", "poll_hz": 10}},
                     # the thesis primitives Viam lacks, as DoCommand verbs (Phase 8, 20_sim/primitive_map.md)
                     {"name": "cap", "api": "rdk:service:generic", "model": "capbot:sim:cap",
                      "attributes": {"numbers_path": str(NUMBERS), "kinematics_path": str(kin / "kr6.json"),
                                     "locations_path": str(REPO / "20_sim/configs/locations.json"),
                                     "drive_speed_mm_per_sec": 250, "turn_speed_deg_per_sec": 45,
                                     "standoff_mm": 100, "lift_mm": 30, "retreat_mm": 150, "verb_timeout_sec": 120,
                                     "dock_mm": 250, "carry_tcp_mm": [300, 0, 900]}}],
    }


# Demo scene (decisions #33, #35, #36): crates picked from above at the table's pick station and stacked from above
# on the crate pallet. These are demo choices, not robot numbers.
DEMO_BOX_MM = 120          # cube edge
BOX_FROM_EDGE_MM = 30      # crate near face this far in from the table edge (close: top-down reach)
N_CRATES = 8               # crates in the queue; the table's pick station shows one at a time (like the course)
# Side-on (decision #37): the cart parks parallel to the table / pallet with the crate on its LEFT (+Y of the cart).
# The cart's side is 350 mm from its centre (its nose 518 mm), so the crate can be close to the shoulder.
CRATE_LEFT_MM = 480        # pick station crate centre this far to the left of the cart centre (~72% arm stretch)
PICK_FROM_LEFT_MM = 100    # pick station: this far in from the table's left end (-X)
TABLE_DOCK_MM = 850        # the cart turns this far before the pick spot and drives in along the table edge
                           # (spinning next to the table would sweep into it; checked by the drive check)
PALLET_LEFT_MM = 560       # crate pallet centre this far to the left of the cart centre (stand 30 mm clear)
PALLET_DOCK_MM = 800       # turn this far before the pallet spot, then drive in straight
HOME_DOCK_MM = 250


def locations(n):
    """Named cart poses and the demo box, in the room frame (mm, deg), from the room's table numbers."""
    t = n["world"]["obstacles"]["lab-table"]
    lo, hi = t["min_mm"], t["max_mm"]          # the real table (no margin)
    pick_x = lo[0] + PICK_FROM_LEFT_MM
    s = DEMO_BOX_MM
    box_y, box_z = lo[1] + BOX_FROM_EDGE_MM + s / 2, hi[2] + s / 2
    cart_y = box_y - CRATE_LEFT_MM
    out = {
        "_about": "GENERATED by 20_sim/tools/make_configs.py from robot_numbers.json (table, crate bin) + demo choices "
                  "(DEMO_BOX_MM, BOX_FROM_EDGE_MM, N_CRATES, CRATE_LEFT_MM, PICK_FROM_LEFT_MM, TABLE_DOCK_MM, PALLET_LEFT_MM, PALLET_DOCK_MM). Room frame, mm and degrees.",
        "places": {
            "home": {"x_mm": 0.0, "y_mm": 0.0, "yaw_deg": 0.0, "dock_mm": HOME_DOCK_MM},
            "table_pick": {"x_mm": pick_x, "y_mm": cart_y, "yaw_deg": 0.0, "dock_mm": TABLE_DOCK_MM},
        },
        # all crates start at the pick station; only the first is there, the next appears when one is set down
        "boxes": {f"box{i + 1}": {"size_mm": [s, s, s], "start": {"x_mm": pick_x, "y_mm": box_y, "z_mm": box_z,
                                                                   "yaw_deg": 0.0}} for i in range(N_CRATES)},
        "pick_station": {"x_mm": pick_x, "y_mm": box_y, "z_mm": box_z},
        "box_spots": {},
    }
    p = n["world"].get("crate_pallet")
    if p:   # Shahar's crate pallet (lab_changes.json): the cart faces it from its -Y side
        cx, cy = p["centre_xy_mm"]
        out["places"]["pallet"] = {"x_mm": cx, "y_mm": cy - PALLET_LEFT_MM, "yaw_deg": 0.0, "dock_mm": PALLET_DOCK_MM}
        pitch = s + p["gap_mm"]
        cols, rows = p["grid"]
        # slot order like the course: layer by layer; within a layer the back row first (any order works from above)
        slots = []
        for layer in range(p["layers"]):
            for row in reversed(range(rows)):
                for col in range(cols):
                    slots.append({"x_mm": cx + (col - (cols - 1) / 2) * pitch, "y_mm": cy + (row - (rows - 1) / 2) * pitch,
                                  "z_mm": p["top_mm"] + layer * s + s / 2, "yaw_deg": 0.0})
        out["box_spots"]["pallet"] = {"type": "pallet", "slots": slots,
                                      "note": "crate centres, filled in this order; 2 x 2 grid, 2 layers"}
    return out


def max_rpm_for(power):
    """Viam maps rpm -> power as power = rpm / max_rpm (no offset). Pick max_rpm so that
    the rpm the report measured at `power` gives back exactly that power."""
    cal = load(CAL)["low_gear_forward"]
    pwm = [p / load(CAL)["pwm_resolution"] for p in cal["pwm"]]
    rpm = cal["wheel_rpm"]
    for (p0, r0), (p1, r1) in zip(zip(pwm, rpm), zip(pwm[1:], rpm[1:])):
        if p0 <= power <= p1:
            r = r0 + (r1 - r0) * (power - p0) / (p1 - p0)
            return round(r / power, 1), round(r, 1)
    raise ValueError("power outside the report table")


def real_config(n):
    mpath, m = latest_measured()
    cart = n["cart"]
    used = {}

    def pick(key, sim_value):
        v = m.get(key)
        used[key] = "measured" if v is not None else "thesis sim"
        return v if v is not None else sim_value

    circ = pick("drive_wheel_circumference_mm", None)
    if circ is None:
        d = m.get("drive_wheel_diameter_mm")
        circ = math.pi * d if d else cart["drive_wheel_circumference_mm"]
        used["drive_wheel_circumference_mm"] = "measured diameter" if d else "thesis sim"
    width = pick("track_width_mm", cart["track_width_mm"])
    pins = m.get("pi_pins", {})
    max_rpm, rpm_at_op = max_rpm_for(OPERATING_POWER)
    speed_at_op = rpm_at_op * circ / 60.0
    motor = lambda name, pwm, rev: {
        "name": name, "api": "rdk:component:motor", "model": "rdk:builtin:gpio",
        "attributes": {"board": "board", "pins": {"pwm": pwm, "dir": rev}, "max_rpm": max_rpm,
                       "min_power_pct": MIN_POWER, "max_power_pct": MAX_POWER,
                       "pwm_freq": m.get("pi_pwm_freq_hz_old_code") or PWM_FREQ_HZ, "dir_flip": False},
    }
    cfg = {
        "modules": [{"type": "registry", "name": "viam_raspberry-pi", "module_id": "viam:raspberry-pi",
                     "version": "latest"}],
        "components": [
            {"name": "board", "api": "rdk:component:board", "model": "viam:raspberry-pi:rpi4", "attributes": {}},
            motor("left", pins.get("left_throttle_pwm", "12"), pins.get("left_reverse", "10")),
            motor("right", pins.get("right_throttle_pwm", "33"), pins.get("right_reverse", "35")),
            {"name": "cart", "api": "rdk:component:base", "model": "rdk:builtin:wheeled",
             "attributes": {"left": ["left"], "right": ["right"], "width_mm": int(round(width)),
                            "wheel_circumference_mm": int(round(circ)), "spin_slip_factor": 1.0},
             "frame": frame("world", [0, 0, 0])},
        ],
        "_note": "GENERATED by 20_sim/tools/make_configs.py.",
    }
    info = {
        "measured_file": str(mpath.relative_to(REPO)) if mpath else None,
        "value_source": used,
        "pins_confirmed_on_cart": bool(pins.get("confirmed_on_cart")),
        "gear_pins": {"left": pins.get("left_gear", "8"), "right": pins.get("right_gear", "37"),
                      "low_gear_level": m.get("low_gear_pin_level")},
        "operating_power": OPERATING_POWER,
        "max_rpm_reason": f"report: {rpm_at_op} rpm at power {OPERATING_POWER}; Viam power = rpm/max_rpm",
        "speed_at_operating_power_mm_s_free_wheel": round(speed_at_op, 1),
    }
    return cfg, info


def write(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    print("wrote", path.relative_to(REPO))


def main():
    n = load(NUMBERS)
    out = REPO / "20_sim/configs"
    write(out / "locations.json", locations(n))
    write(out / "capbot-sim.json", sim_config(n))
    real, info = real_config(n)
    (REPO / "30_robot/configs").mkdir(parents=True, exist_ok=True)
    write(REPO / "30_robot/configs/capbot.json", real)
    write(REPO / "30_robot/configs/capbot_build_info.json", info)
    print(json.dumps(info, indent=2))


if __name__ == "__main__":
    main()
