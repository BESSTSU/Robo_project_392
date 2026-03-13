# Drive ESP32 Firmware

Features:
- 4WD drive output (MDD10A style DIR+PWM)
- 2 encoder channels (left/right)
- odometry distance + RPM telemetry
- BNO055 IMU on I2C (`SDA=21`, `SCL=13`, addr `0x28`)
- I2C clock forced to `100kHz` for better noise tolerance on real wiring

Serial protocol:
- `VEL L:<int> R:<int>`
- `ODO RESET`
- `STOP`
- `STATUS`
- `IMU STATUS`
- `PROTO_VER`
- `HELP`

Telemetry:
- `DRV_STAT L_RPM:<float> R_RPM:<float> DIST_CM:<float> ... IMU:<0|1> HEAD_DEG:<float>`
