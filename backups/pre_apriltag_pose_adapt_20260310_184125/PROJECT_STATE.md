# Project State

Primary workspace: `/home/besstsu/Documents/AmazingRobot`

Use this file as the first context source when conversation state is compacted.

## Current system architecture
- Main runtime is Python, not ROS2.
- Autonomy uses physical front/rear cameras directly; during mission the dashboard front/rear panels stay fixed to physical front/rear. Camera swap is allowed for idle preview only and must not change mission behavior.
- `start_mission()` forces camera swap OFF so every mission begins with physical front=front and rear=rear.
- Entry point: `app/main.py`
- Runtime scripts:
  - `setup_env.sh`
  - `run.sh`
  - `verify.sh`
- Two ESP32 roles:
  - Drive ESP32: drive motors + encoders + IMU heading telemetry
  - Actuator ESP32: planting stepper/limits/servos

## Current important files
- Mission logic: `app/orchestration/mission_orchestrator.py`
- Planting logic: `app/controllers/planting_sequence.py`
- Drive serial client: `app/controllers/drive_client.py`
- Actuator serial client: `app/controllers/stepper_servo_client.py`
- AprilTag service: `app/services/apriltag_52h13.py`
- Dashboard: `app/web/dashboard.py`
- Config: `app/config.py`
- Drive firmware reference in this repo: `firmware/drive_esp32/main.cpp`

## Current intended automatic mission flow
1. `INITIALIZING`
   - verify drive/actuator serial mapping
   - home plant axis to top limit
   - set start servo positions
   - reset drive odometry
2. `SEARCHING_TARGET`
   - detect target from cameras
   - `FaceWoodenbox` is the preferred class
3. Early `READING_APRILTAG`
   - mission now starts in `READING_APRILTAG` immediately after initialization
   - if no AprilTag yet but front camera sees `FaceWoodenbox` or `woodenbox`, the robot stays stopped and keeps waiting for an acceptable tag reading
   - if no front woodenbox is available, fall back to `SEARCHING_TARGET`
   - if multiple tags are visible outside the plot, prefer the AprilTag nearest to `FaceWoodenbox`
   - only accept tag if:
     - the tag is near `FaceWoodenbox`
     - center is near image middle
     - yaw is not too side-facing
4. `TRIANGLE / COARSE ALIGN`
   - if AprilTag scan succeeds, run triangle align first
  - if triangle cannot run, report the failure reason and fall back to normal PID approach
5. `ALIGNING_ENTRY`
   - after triangle/coarse, go directly to entry align
   - do not return to long PID approach first
   - front camera only
6. `Pre-April adjust`
   - align bbox bottom to trigger line before accepting final front-of-plot reading
7. `MOVING_TO_PLANT_POINT`
   - use AprilTag-derived distances
8. `PLANTING`
   - plant first point
   - move by `C`
   - plant second point
9. Rear measuring loop
   - first rear measure move uses `DE + PLANT_TO_CAMBACK_POINT_CM`
   - later rear measure moves use `DE`

## Current design intent for triangle align
- Triangle align exists to make the robot approach the plot from a near-perpendicular, easier entry pose before PID entry.
- PID should do fine adjustment, not the large heading correction.
- Current triangle behavior:
  - derive `x,z` from AprilTag pose, or approximate from `bearing + dist`
  - if lateral offset is too small, skip triangle and let normal PID continue
  - `turn1`: rotate orthogonally by `-90` when `x>=0` or `+90` when `x<0`, then add tag yaw compensation if available
  - `move1`: drive the lateral leg only, using encoder distance `|x|`
  - `turn2`: rotate back by `-turn1` so the robot faces the plot again before PID entry
  - if triangle succeeds, go to `ALIGNING_ENTRY`
  - if triangle fails, do not fake success and do not insert AprilTag coarse-align in between; fall back to PID approach with explicit failure status

## Current AprilTag acceptance gate
- Purpose: reject side-view or unrelated tag readings.
- Config keys:
  - `APRILTAG_FACE_BOX_EXPAND_SCALE`
  - `APRILTAG_ACCEPT_CENTER_TOL_NORM`
  - `APRILTAG_ACCEPT_YAW_DEG`
- Current `.env` defaults:
  - `APRILTAG_FACE_BOX_EXPAND_SCALE=1.35`
  - `APRILTAG_ACCEPT_CENTER_TOL_NORM=0.22`
  - `APRILTAG_ACCEPT_YAW_DEG=22.0`
  - `APRILTAG_ALLOW_WOODENBOX_FALLBACK=1`
  - `APRILTAG_WOODENBOX_CENTER_TOL_NORM=0.14`
  - `APRILTAG_WOODENBOX_YAW_DEG=16.0`
  - `APRILTAG_WOODENBOX_MIN_SIDE_PX=16.0`
  - `APRILTAG_WOODENBOX_BEARING_DEG=18.0`
- If tag is not near `FaceWoodenbox`, mission should reject it and continue waiting when `APRILTAG_ALLOW_WOODENBOX_FALLBACK=0`.
- If tag is near `FaceWoodenbox` but still too side-facing, mission should reject it and continue waiting.

## Current triangle-align tuning keys
- `APRILTAG_TRIANGLE_ALIGN_ENABLED=1`
- `APRILTAG_TRIANGLE_STANDOFF_CM=20.0`
- `APRILTAG_TRIANGLE_MAX_MOVE_CM=120.0`
- `APRILTAG_TRIANGLE_MAX_TURN_DEG=140.0`
- `APRILTAG_TRIANGLE_MOVE_NORM=0.24`
- `APRILTAG_TRIANGLE_TURN_TIMEOUT_SEC=4.0`
- `APRILTAG_TRIANGLE_MOVE_TIMEOUT_SEC=8.0`
- `APRILTAG_TRIANGLE_TURN2_DEG=90.0`
- `APRILTAG_TRIANGLE_MIN_LATERAL_CM=3.0`
- `APRILTAG_TRIANGLE_FINAL_HEADING_OFFSET_DEG=0.0`

## Current triangle debug/log behavior
- Triangle stage can emit:
  - `Triangle plan(orthogonal): x=... z=... dist=... turn1=... move1=... turn2=...`
  - `triangle turn1: current=... target=... err=...`
  - `triangle move1: moved=... remain=...`
  - `triangle turn2: current=... target=... err=...`
- If triangle does not actually execute, status should say why, for example:
  - `Triangle align skipped: no IMU heading`
  - `Triangle align skipped: insufficient tag pose`
  - `Triangle align skipped: lateral offset small (...)`
  - `Triangle align skipped: move too large (...)`
  - `Triangle align skipped: turn too large (...)`
- Triangle failure should no longer be followed by a fake `Triangle/coarse complete` message.

## Current camera calibration state
- Front camera intrinsics were calibrated and copied into `.env`.
- Current values:
  - `APRILTAG_FX=678.151530`
  - `APRILTAG_FY=679.206763`
  - `APRILTAG_CX=313.821216`
  - `APRILTAG_CY=243.169972`
- AprilTag physical size:
  - `APRILTAG_SIZE_CM=4.2`

## Current IMU decision
- IMU is intended to be on the Drive ESP32, not on the Pi.
- Target sensor: `BNO055`
- Planned/implemented I2C pins for drive firmware:
  - `SDA = GPIO21`
  - `SCL = GPIO13`
  - address `0x28`
- Drive telemetry should include:
  - `IMU`
  - `HEAD_DEG`
  - `CAL_SYS`
  - `CAL_GYRO`
  - `CAL_ACCEL`
  - `CAL_MAG`

## Current planting behavior notes
- Final close should use `SERVO_PLANT_START_DEG`.
- Phase-2 down step uses `STEP1_AFTER_BURRIED_DEG`.
- Existing planting sequence should be preserved unless the user explicitly asks to change it.

## Current rear measurement behavior
- After 2 plants:
  - first measurement move: `DE + PLANT_TO_CAMBACK_POINT_CM`
  - next measurement moves: `DE`
- There is also a pre-plant rear-size checkpoint before first planting move completes.

## Current important environment values
- `PLANT_TO_CAMBACK_POINT_CM=46.5`
- `PRE_APRIL_BBOX_TOLERANCE_PX=10`
- `PRE_APRIL_ADJUST_NORM=0.18`
- `PRE_APRIL_ADJUST_TIMEOUT_SEC=6.0`
- `VISION_LOOP_SEC=0.05`
- `STATUS_LOOP_SEC=0.10`

## Current run method
```bash
cd /home/besstsu/Documents/AmazingRobot
source venv/bin/activate
bash run.sh
```

## Current manual web controls
- `Forward/Backward/Left/Right/Stop` send direct manual velocity commands and are only allowed when mission is not running.
- Dashboard now has `Rotate By Degrees`:
  - posts `/api/manual/rotate`
  - uses IMU heading from drive ESP32
  - computes `target = current_heading + delta_deg`
  - then rotates through the existing `_rotate_to_heading()` closed-loop helper
  - manual rotate uses the stable dashboard tuning that previously reached `heading locked err≈7.6deg`:
    - `MANUAL_ROTATE_TIMEOUT_SEC=12.0`
    - `MANUAL_ROTATE_HOLD_SAMPLES=1`
    - `MANUAL_ROTATE_GAIN=0.040`
    - `MANUAL_ROTATE_NORM_MAX=0.45`
    - `MANUAL_ROTATE_TOLERANCE_DEG=8.0`
  - fine-stage tuning remains in config for compatibility, but `manual_rotate()` no longer uses it
- Dashboard now also has `Measure AprilTag Yaw`:
  - posts `/api/apriltag/yaw`
  - uses the physical front camera while mission is idle
  - captures a few AprilTag detections and reports median `tag_pose_yaw_deg` as the yaw offset between robot and tag
  - also reports `bearing_x_deg`, `approx_dist_cm`, and sample count in `apriltag_yaw_measurement` on `/api/status`

## Sync method commonly used
```bash
rsync -azP \
  --exclude 'venv/' \
  --exclude 'backups/' \
  --exclude 'logs/' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude '.git/' \
  /home/besstsu/Documents/AmazingRobot/ \
  besst@<PI_IP>:/home/besst/AmazingRobot
```

## Operator-facing statuses that should exist
- If AprilTag is seen but rejected during early scan, keep the reject reason visible in `status_text` instead of overwriting it with generic waiting text.
- `Target locked (...), scanning AprilTag first: ...`
- `No triangle: AprilTag not ready in early scan, continue PID approach`
- `Triangle/coarse complete, begin entry align`
- `AprilTag seen but rejected (...)`
- `AprilTag seen but rejected (not near FaceWoodenbox)`
- `Triangle/coarse failed, continue PID approach (...)`

## Guard rails
- Do not switch work to `Robo_eng2/refactored`.
- Keep changes in `AmazingRobot` unless the user explicitly asks otherwise.
- Preserve planting and measuring process structure unless the user explicitly wants that flow changed.

## Current camera mapping on Pi
- Physical front camera is currently mapped as index `2`.
- Physical rear camera is currently mapped as index `0`.
- `.env` should therefore use `FRONT_CAMERA_INDEX=2` and `REAR_CAMERA_INDEX=0`.
