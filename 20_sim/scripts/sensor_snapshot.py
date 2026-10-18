"""Show what every sensor on capbot-sim reports right now (fake data this week).

Saves to 20_sim/scripts/out/: one image per camera (cam-*.jpg), the lidar point cloud (lidar.pcd)
and all the numbers (sensor_snapshot.txt). Open them in VS Code.

Run:  python3 20_sim/scripts/sensor_snapshot.py
In the app: capbot-sim -> CONTROL tab shows the camera streams and the IMU readings live.
"""

import asyncio
from pathlib import Path

import numpy as np
from viam.components.camera import Camera
from viam.components.movement_sensor import MovementSensor

from capbot_common import connect

OUT = Path(__file__).resolve().parent / "out"


def pcd_points(data):
    """x, y, z in mm from a PCD file (Viam writes metres; ascii or binary float32 x y z first)."""
    head_end = data.index(b"DATA ")
    header = data[:head_end].decode(errors="ignore")
    n = int(next(l.split()[1] for l in header.splitlines() if l.startswith("POINTS")))
    fields = next(l.split()[1:] for l in header.splitlines() if l.startswith("FIELDS"))
    sizes = [int(s) for s in next(l.split()[1:] for l in header.splitlines() if l.startswith("SIZE"))]
    line_end = data.index(b"\n", head_end)
    kind = data[head_end + 5:line_end].decode().strip()
    body = data[line_end + 1:]
    if kind == "ascii":
        rows = np.loadtxt(body.decode().splitlines()[:n], ndmin=2)
        return rows[:, :3] * 1000.0, n
    stride = sum(sizes)
    raw = np.frombuffer(body[:n * stride], dtype=np.uint8).reshape(n, stride)
    xyz = raw[:, :12].copy().view("<f4").reshape(n, 3)
    return xyz.astype(float) * 1000.0, n


async def main():
    OUT.mkdir(exist_ok=True)
    lines = []

    def say(s):
        print(s)
        lines.append(s)

    robot = await connect("sim")
    try:
        for name in ("cam-front", "cam-rear", "cam-wrist"):
            images, _ = await Camera.from_robot(robot, name).get_images()
            img = images[0]
            (OUT / f"{name}.jpg").write_bytes(img.data)
            say(f"{name:9s} image {img.mime_type}, {len(img.data)} bytes -> out/{name}.jpg")
        data, mime = await Camera.from_robot(robot, "lidar").get_point_cloud()
        (OUT / "lidar.pcd").write_bytes(data)
        try:
            xyz, n = pcd_points(data)
            say(f"lidar     {n} points ({mime}), in mm: x {xyz[:, 0].min():.1f}..{xyz[:, 0].max():.1f}, "
                f"y {xyz[:, 1].min():.1f}..{xyz[:, 1].max():.1f}, z {xyz[:, 2].min():.1f}..{xyz[:, 2].max():.1f} -> out/lidar.pcd")
        except Exception as e:  # format surprise: still saved
            say(f"lidar     {len(data)} bytes ({mime}) -> out/lidar.pcd (could not read points: {e})")
        imu = MovementSensor.from_robot(robot, "imu")
        props = await imu.get_properties()
        say("imu       supports: " + ", ".join(k for k, v in [
            ("orientation", props.orientation_supported), ("angular velocity", props.angular_velocity_supported),
            ("linear acceleration", props.linear_acceleration_supported), ("linear velocity", props.linear_velocity_supported),
            ("position", props.position_supported), ("compass heading", props.compass_heading_supported)] if v))
        for label, call in [("orientation", imu.get_orientation), ("angular velocity (deg/s)", imu.get_angular_velocity),
                            ("linear acceleration (m/s^2)", imu.get_linear_acceleration),
                            ("linear velocity (m/s)", imu.get_linear_velocity), ("compass heading (deg)", imu.get_compass_heading)]:
            try:
                v = await call()
                say(f"imu       {label}: {str(v).strip().replace(chr(10), ' ')}")
            except Exception as e:
                say(f"imu       {label}: not given ({str(e).splitlines()[0][:80]})")
        try:
            pos, alt = await imu.get_position()
            say(f"imu       position: lat {pos.latitude}, lng {pos.longitude}, alt {alt} m (fake GPS point)")
        except Exception as e:
            say(f"imu       position: not given ({str(e).splitlines()[0][:80]})")
    finally:
        await robot.close()
    (OUT / "sensor_snapshot.txt").write_text("\n".join(lines) + "\n")
    print("saved out/sensor_snapshot.txt")


if __name__ == "__main__":
    asyncio.run(main())
