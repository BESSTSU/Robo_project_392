# Drive ESP32 Firmware

Features:
- 4WD drive output (MDD10A style DIR+PWM)
- 2 encoder channels (left/right)
- odometry distance + RPM telemetry

Serial protocol:
- `VEL L:<int> R:<int>`
- `ODO RESET`
- `STOP`
- `STATUS`
- `PROTO_VER`
- `HELP`

Telemetry:
- `DRV_STAT L_RPM:<float> R_RPM:<float> DIST_CM:<float>`
