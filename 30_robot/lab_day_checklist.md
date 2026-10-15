# Lab day checklist - Thu 15 Oct 2026 (v2, focused on the cart Pi)

Why today matters: the weekend is sim only, and **Sunday we drive the real cart from our Python script
through Viam, running on the cart's Raspberry Pi (no arm)**. Anything we don't find out today blocks Sunday.
Rule: photograph and measure; write "unknown" rather than a guess. Units: mm and degrees.
Old version: `90_archive/2026-10-15_lab_day_checklist_v1.md`.

## What we already know (from the 2024 cart project report, `CaP_Thesis/sim/cart_kr6/reference/cart_project_report_24-33.pdf`)
Still to be checked against the real cart today:
- Raspberry Pi 4 drives 2 BLDC hub motors (BLDC = brushless DC; motors built into the wheels), each through its own motor controller.
- Per motor: **throttle** = Pi PWM through a low-pass filter (an RC circuit that turns PWM into a steady voltage),
  **reverse** pin, **gear** pin. Motors spin from ~25% duty and stop getting faster at ~70% (64/255 to 178/255).
- Pi pins (physical numbers): left gear 8, left reverse 10, left throttle 12, left GND 14 |
  right throttle 33, right reverse 35, right gear 37, right GND 39 | MPU6050 IMU: 3.3V 1, SDA 3, SCL 5, GND 9.
- Speed calibration (free wheel): low gear max 15.56 rad/s, high gear 31.35 rad/s, reverse low max ~6.4 rad/s.
- The old code was a ROS C++ node (`mauro_prototype_control`) driven by a game pad.
- Note: pins 8/10 are also the Pi's serial port (UART), so the serial console must be off.
- Wheel size: the thesis sim (from the cart CAD) says drive wheels **358 mm**, track **478 mm**, frame 1000 x 700. That matches the
  report's speeds. The 13/08 note (300 mm wheels, 500 apart) looks wrong; measure to settle it.

## 1. Photos first (phone), before touching anything
- [ ] Whole cart, 4 sides + top
- [ ] The Pi, close up, with every cable in place (so it can all go back the same way)
- [ ] The low-pass filter board, both motor controllers (labels!), batteries, fuses, main switch
- [ ] Every name plate / label (motor controllers, batteries, lidar, IMU, arm, KR6 controller)
- [ ] Wiring between Pi -> filter board -> motor controllers

## 2. Get into the Pi (top priority)
- [ ] Ask the lab: who has the Pi login? Old project team / lab manager. Write the **username only** here, never the password.
- [ ] Way in: screen + keyboard (Pi 4 needs **micro-HDMI**), or the network (find its IP from the router, or `nmap -sn <lab subnet>/24` from stav).
- [ ] Power the Pi **only** (not the motors) if possible. Note how the Pi gets power (from the cart battery? a USB supply?).
- [ ] Copy `30_robot/tools/pi_survey.sh` to the Pi (USB stick or `scp`), run:
      `bash pi_survey.sh > pi_survey_$(hostname).txt 2>&1` and copy the txt back to `30_robot/`. It only reads, changes nothing.
- [ ] From the survey, the key answers: **64-bit?** (`getconf LONG_BIT` = 64), OS version, free disk,
      internet + app.viam.com reachable, I2C on with **0x68** visible, what old code starts at boot.

## 3. Back up the SD card (before ANY install, today or Sunday)
- [ ] If there is a USB card reader: Pi off, card out, on stav:
      `lsblk` (find the card, e.g. /dev/sdb) then `sudo dd if=/dev/sdX of=~/cart_pi_2026-10-15.img bs=4M status=progress`
- [ ] If no reader: over the network from stav (Pi running, nothing else using it):
      `ssh <user>@<pi-ip> "sudo dd if=/dev/mmcblk0 bs=4M" | dd of=~/cart_pi_2026-10-15.img bs=4M status=progress`
- [ ] Check the image size is about the card size. Keep it on stav (not in git; it's big and may hold passwords).
- [ ] If the OS is 32-bit: a 64-bit reinstall is needed on Sunday. Get a spare microSD (32 GB+) and a USB card reader before Sunday.

## 4. Check the wiring against the report
- [ ] Pins actually used on the Pi header match the list above? Mark any difference in a photo.
- [ ] Motor controller make/model; what each input wire is (throttle, reverse, gear, brake?, hall?)
- [ ] Is there any speed feedback wire back to the Pi (hall or encoder)? (If not: no wheel odometry, the cart drives open loop.)
- [ ] Battery: type, voltage, how it's charged. Charge it today for Sunday.
- [ ] Main power switch / fuse: where. Can motor power be cut while the Pi stays on?

## 5. Network for Sunday
- [ ] Can the Pi join a lab Wi-Fi without a browser login page? SSID name: ____
- [ ] If not: test a phone hotspot with the Pi (or plan it for Sunday). Ethernet port near the cart?
- [ ] The survey's line `app.viam.com HTTP ...` must show a code (200/30x), not "NOT reachable".

## 6. Safety plan for Sunday (first time Viam moves real wheels)
- [ ] How to lift the drive wheels off the floor (blocks / jack). Note: the KR6 is on the cart, so it's heavy.
- [ ] Who will be there, standing at the power switch.
- [ ] Is there an e-stop? If not, the main switch / battery disconnect is the e-stop. Mark it with tape.
- [ ] Optional, only if the old system runs and wheels are in the air: drive with the old game pad for 10 s, to prove the hardware works before Sunday.

## 7. Measurements (write them into `30_robot/measured_2026-10-15.json`)
- [ ] Frame outer length x width x frame-top height
- [ ] Drive wheel diameter (or roll one turn and measure the distance = circumference)
- [ ] Wheel track: centre of left tyre to centre of right tyre
- [ ] Swivel wheels: diameter and positions
- [ ] Lidar: make/model, position on the cart (x, y, z from the cart centre on the floor), how it connects (USB?)
- [ ] IMU: where it sits, which way its axes point
- [ ] Cameras on the cart, if any
- [ ] Arm base position on the cart + KR6 exact model from its plate (for the sim)
- [ ] The room (the thesis lab model is a sketch): the 4 pillars' positions and size, the floor area edges.
      Origin = centre of the floor, +X along the 9 m side; note which pillar is front-left (photo with arrows)
- [ ] Where the `arm-table` rig stands in the room (2 x 2 m table, cage to 1.95 m) and the lab table / cabinet, if they exist
- [ ] Suction tool: flange face -> cup face distance, tool size

## Before leaving
- [ ] Photos copied into `30_robot/photos/` (named like `2026-10-15_pi_header.jpg`)
- [ ] `30_robot/pi_survey_<hostname>.txt` saved
- [ ] SD image on stav, size checked
- [ ] Every "unknown" written as an open question in today's log
