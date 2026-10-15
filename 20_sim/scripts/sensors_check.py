"""GATE 6: read every sensor on capbot-sim once (fake data this week).

Run:  python3 20_sim/scripts/sensors_check.py
Pass: every sensor prints a line, one image is saved to 20_sim/scripts/out/, "sensors_check PASS".
"""

import asyncio
from pathlib import Path

from viam.components.camera import Camera
from viam.components.movement_sensor import MovementSensor

from capbot_common import connect

OUT = Path(__file__).resolve().parent / "out"


async def main():
    robot = await connect("sim")
    ok = True
    try:
        OUT.mkdir(exist_ok=True)
        for name in ("cam-front", "cam-rear", "cam-wrist"):
            try:
                images, _ = await Camera.from_robot(robot, name).get_images()
                img = images[0]
                (OUT / f"{name}.jpg").write_bytes(img.data)
                print(f"OK   {name}: {len(images)} image(s), {img.mime_type}, {len(img.data)} bytes")
            except Exception as e:
                ok = False
                print(f"FAIL {name}: {e}")
        try:
            data, mime = await Camera.from_robot(robot, "lidar").get_point_cloud()
            print(f"OK   lidar: point cloud {len(data)} bytes ({mime})")
        except Exception as e:
            ok = False
            print(f"FAIL lidar: {e}")
        try:
            imu = MovementSensor.from_robot(robot, "imu")
            print(f"OK   imu: angular velocity {await imu.get_angular_velocity()}".replace("\n", " "))
        except Exception as e:
            ok = False
            print(f"FAIL imu: {e}")
        for name in ("cam-front", "cam-wrist", "lidar", "imu"):
            p = (await robot.get_pose(name, "world")).pose
            print(f"     {name:9s} in world: x={p.x:8.1f} y={p.y:8.1f} z={p.z:8.1f}")
    finally:
        await robot.close()
    print("sensors_check PASS" if ok else "sensors_check FAIL")


if __name__ == "__main__":
    asyncio.run(main())
