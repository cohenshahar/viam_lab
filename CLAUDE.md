# Viam Lab — project instructions

## What this project is
A one-week learning sprint (Tue 13 Oct to Sun 18 Oct 2026) to learn the Viam robotics
platform and model my lab robot in it, in simulation first, with real hardware as the
next step. When the sprint ends, this folder moves into the CaP thesis folder and repo.

Owner: Shahar (M.Sc. Mechatronics, BGU, Prof. Amir Shapiro's lab).
Viam contacts: Avery Rosen (Technical Chief of Staff), Oliver Mitchell.

## Goal
1. Finish the Viam 101 course (learn.viam.com).
2. Model the CaP robot in Viam sim: KUKA KR6 on the lab cart (diff-drive base).
3. Run the same setup from three places: this Windows computer (course, notes, SDK
   scripts), the strong lab computer (Ubuntu, viam-server), and later the robot computer.

## Time
About 3 hours a day, Tue to Sun. Log real hours every day in 40_logs.

## How we work (most important)
This is a LEARNING project. I want to do more of the work myself than usual.
- I write the configs and code myself. Claude explains, reviews and points to docs.
- When I'm stuck: first a hint, then a smaller hint, and a full solution only if I ask.
- Explain the "why", not only the "how". Expand every acronym once, with one line on
  what it actually is.
- I run all commands myself and paste the output. Claude reads it with me.
- When code is given: the final code first, then a block-by-block explanation.
- I work in VS Code. Never send me to notepad or another editor.
- Plain everyday words. No fancy vocabulary, in chat or in files.
- Everything in this project is in English.
- Check facts about Viam in the docs (docs.viam.com) before stating them. Say clearly
  when something is not verified.

## Rules
- Never delete a file. Old or dropped work moves to 90_archive with a date and a reason.
- Secrets (API keys, machine cloud credentials, viam.json) never go into git.
- GitHub (private repo Viam_Lab) is the bridge between the computers. Claude prepares
  files; I commit and push.
- One source of truth per subject: the plan lives in 00_admin/plan.md, decisions in
  00_admin/decisions.md, daily progress in 40_logs.

## Folder map
- 00_admin/      plan for the week, decision log
- 10_course/     my notes from Viam 101, one file per module
- 20_sim/        the simulated robot
  - configs/     machine config JSON (no secrets)
  - kinematics/  KR6 kinematics / URDF files for Viam
  - scripts/     Python SDK scripts I write
- 30_robot/      real hardware facts (cart, KR6, computers) for the next stage
- 40_logs/       one file per day: YYYY-MM-DD.md
- 50_reference/  links, glossary, useful docs pages
- 90_archive/    dropped or old work, never deleted

## Known facts (checked)
- Python venv in .venv (Python 3.12.3) already has viam-sdk 0.84.0 installed.
- Viam 101 is free, self-paced and runs fully in simulation.
- The official viam-kuka module lists the KR10 R900-2 only. KR6 is not listed, so the
  KR6 likely needs its own kinematics / URDF file. (To confirm.)
- Lab cart (measured 13/08/2026, mm): welded aluminium frame 1000 x 700, frame top 400,
  diff drive, two 300 mm drive wheels 500 apart on the centre axis, two 120 mm swivel
  wheels on the middle axis. Full record is in the CaP thesis folder.

## Start of every session
1. Read 00_admin/plan.md and the latest file in 40_logs.
2. Say what today's task is and what "done" looks like.
At the end: update today's log (done, learned, hours, open questions).
