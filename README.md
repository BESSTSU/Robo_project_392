# AmazingRobot

Autonomous farming robot stack (Pi + 2 ESP32) with OOP architecture.

## What is included
- Python application (`app/`) for mission orchestration, dual camera processing, web dashboard, and serial integration.
- ESP32 firmware templates:
  - `firmware/drive_esp32/main.cpp` (motor driver + dual encoder telemetry)
  - `firmware/stepper_esp32/main.cpp` (2 steppers + 2 limits + 2 servos)
- Setup and run scripts:
  - `setup_env.sh`
  - `run.sh`
  - `verify.sh`

## Current project state
- See `PROJECT_STATE.md` for the current intended workflow, tuning, and active design decisions.
- If conversation context is compacted, re-open `AGENTS.md` and `PROJECT_STATE.md` first.

## Quick start
```bash
cd /home/besstsu/Documents/AmazingRobot
bash setup_env.sh
bash run.sh
```

Open dashboard: `http://localhost:5000`

## Runtime behavior
Mission flow implemented as FSM:
1. Initialize actuators (plant axis to top limit, reset servos)
2. Search target (`FaceWoodenbox` priority over `Woodenbox`)
3. PID approach target
4. Sweep camera servo when bbox bottom reaches configurable horizon line
5. Read AprilTag 52h13 and decode AB/C/DE
6. Move forward by configurable `FRONT_TO_PLANT_POINT_CM` using encoder distance telemetry
7. Execute planting sequence
8. Continue until front and rear cameras both do not detect `Woodenbox`

## Important notes
- If YOLO or AprilTag libs are unavailable, the app still starts; status will show degraded mode.
- All tunable values are in `app/config.py`.
- This project assumes two serial devices:
  - Drive ESP32 (default `/dev/ttyUSB0`)
  - Stepper/Servo ESP32 (default `/dev/ttyUSB1`)
