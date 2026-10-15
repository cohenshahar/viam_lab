# Lab day checklist — Thu 15 Oct 2026

Goal: leave the lab with everything needed to model the robot in Viam on Fri/Sat **without** going back.
Rule: measure and photograph; don't guess. Write "unknown" rather than a guess.
Units: mm and degrees. Write where each measurement was taken from and to.

## 1. Photos (phone)
- [ ] Whole robot from 4 sides + top
- [ ] Every name plate / label (arm, controller, motors, drivers, computer, vacuum parts)
- [ ] Arm base mounting on the cart (bolts, plate)
- [ ] Tool flange + suction tool, side view with a ruler in the shot
- [ ] Wiring / connectors between computer, controller, drivers

## 2. KR6 arm
- [ ] Exact model from the name plate (KR6 ... variant)
- [ ] Controller model (from its label)
- [ ] How the arm is commanded today (software, interface, cable/network)
- [ ] Arm base position on the cart: offset from a cart reference point (x, y, z) and rotation
- [ ] Home / rest joint angles as shown on the pendant (if available)

## 3. Suction tool (end effector)
- [ ] Make/model of cups, vacuum source (generator/pump), valve
- [ ] **Flange face → cup face distance** (the TCP offset, tool center point) and any sideways offset
- [ ] Tool size for a collision box: width x length x height
- [ ] How suction is switched on/off (I/O, relay, from which device)

## 4. Cart (mobile base)
- [ ] Check the 13/08 measurements still hold (frame 1000 x 700, top 400, wheels)
- [ ] Choose and mark the cart reference point (origin) and axis directions; photo it
- [ ] Motor + driver models, how they are commanded
- [ ] Anything on the cart that sticks out (collision volumes)

## 5. Computers and network
- [ ] Which computer is on the robot, OS, how it connects (Wi-Fi / cable)
- [ ] Which computer talks to the KR6 controller
- [ ] IP addresses / hostnames (no passwords in this file)

## 6. Sensors and other parts
- [ ] Cameras (model, where mounted, offset from cart origin or flange)
- [ ] E-stop, any other I/O

## 7. In the Viam app (lab robot machine)
- [ ] One block per part, named clearly (arm, gripper, base, cameras)
- [ ] Frames filled in from the measurements above
- [ ] Note which blocks are placeholders until the real module/driver is known

## Before leaving the lab
- [ ] All photos copied off the phone into 30_robot/photos/
- [ ] Every "unknown" listed in today's log as an open question
