# Stepper/Servo ESP32 Firmware

Features:
- 2 steppers (`FEED`, `PLANT`)
- 2 limit switches (top/bottom) for plant axis
- 2 servos (`SERVO CAM`, `SERVO PLANT`)

Serial protocol (examples):
- `INIT_HOME CAM:120 PLANT:10`
- `SERVO CAM 100`
- `SERVO PLANT 70`
- `FEED STEP 30`
- `PLANT DOWN_LIMIT`
- `PLANT UP_STEP 60`
- `PLANT DOWN_STEP 60`
- `PLANT UP_LIMIT`
- `ACT_STATUS`
- `ACT_STOP`
- `PROTO_VER`
- `HELP`
