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
2. Startup `TRIANGLE ALIGN`
   - immediately after initialization/home, do one AprilTag yaw scan using the same path as `Measure AprilTag Yaw`
   - use `approx_dist_cm` and measured yaw to plan a rotate-move-rotate sequence
   - after the startup triangle attempt finishes, continue to `SEARCHING_TARGET`
3. `SEARCHING_TARGET`
   - detect target from cameras
   - `FaceWoodenbox` is the preferred class
4. `READING_APRILTAG`
   - after a target is locked and no cached tag exists yet, mission changes to `READING_APRILTAG` and keeps reading there
   - if no AprilTag yet but front camera sees `FaceWoodenbox` or `woodenbox`, the robot stays stopped and keeps waiting for an acceptable tag reading
   - if no front woodenbox is available, fall back to `SEARCHING_TARGET`
   - if multiple acceptable tags are visible, prefer:
     - tag near `FaceWoodenbox` before `woodenbox` fallback
     - then the most front-facing tag (smaller absolute yaw)
     - nearest tag to the anchor box first
     - then smaller horizontal bearing
     - then larger visible tag size (`px`)
   - only accept tag if:
     - the tag is near `FaceWoodenbox`
     - center is near image middle
     - yaw is not too side-facing
   - after AprilTag is read, continue normal PID approach; do not run triangle again here
5. `ALIGNING_ENTRY`
   - after PID approach reaches the trigger condition, go to entry align
   - front camera only
6. `Pre-April adjust`
   - align bbox bottom to trigger line before accepting final front-of-plot reading
7. `MOVING_TO_PLANT_POINT`
   - use AprilTag-derived distances
8. `PLANTING`
   - plant first point
   - move by `AB`
   - plant second point
9. Rear measuring loop
   - first rear measure move uses `C + PLANT_TO_CAMBACK_POINT_CM`
   - later rear measure moves use `DE`

## Current design intent for triangle align
- Triangle align now runs once right after initialization, before target search.
- It uses the same measurement path as `Measure AprilTag Yaw` and relies on:
  - `approx_dist_cm`
  - measured `yaw_deg`
- Current triangle behavior:
  - if `yaw_deg < 0`, `turn1 = -90 - yaw_deg`
  - if `yaw_deg >= 0`, `turn1 = 90 - yaw_deg`
  - `move1 = abs(approx_dist_cm * sin(yaw_deg))`
  - if initial `yaw_deg < 0`, `turn2 = +90`
  - if initial `yaw_deg >= 0`, `turn2 = -90`
  - `turn1` and `turn2` use the same sign semantics as dashboard `Rotate By Degrees`
- `move1` uses the same relative encoder-distance path as dashboard `Drive By Distance`
- runtime drive-speed tuning from dashboard affects `move1` too
  - after the startup triangle attempt, mission continues to `SEARCHING_TARGET`

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
- `woodenbox` fallback must also reject side-facing tags using `APRILTAG_WOODENBOX_YAW_DEG`; do not accept side-panel tags just because bearing is near center.

## Current triangle-align tuning keys
- `APRILTAG_TRIANGLE_ALIGN_ENABLED=1`
- `APRILTAG_TRIANGLE_MAX_MOVE_CM=120.0`
- `APRILTAG_TRIANGLE_MAX_TURN_DEG=140.0`
- startup triangle rotate steps use the same tuning path as dashboard `Rotate By Degrees`
- startup triangle move step uses the same tuning path as dashboard `Drive By Distance`
- old triangle pose keys remain in config for compatibility, but the startup yaw-scan triangle does not use them

## Current triangle debug/log behavior
- Triangle stage can emit:
  - `AprilTag yaw measured: yaw=...deg bearing=...deg`
  - `Triangle plan: approx=... yaw=... turn1=... move1=... turn2=...`
  - `triangle turn1: current=... target=... err=...`
  - `triangle move1: moved=... remain=...`
  - `triangle turn2: current=... target=... err=...`
- If triangle does not actually execute, status should say why, for example:
  - `Triangle align skipped: no usable AprilTag yaw measurement`
  - `Triangle align skipped: missing approx_dist/yaw measurement`
  - `Triangle align skipped: move too large (...)`
  - `Triangle align skipped: turn1 too large (...)`
  - `Triangle align skipped: turn2 too large (...)`

## Current camera calibration state
- Front camera intrinsics were calibrated and copied into `.env`.
- Current values:
  - `APRILTAG_FX=678.151530`
  - `APRILTAG_FY=679.206763`
  - `APRILTAG_CX=313.821216`
  - `APRILTAG_CY=243.169972`
- AprilTag physical size:
  - `APRILTAG_SIZE_CM=4.2`
- Practical AprilTag pose path adapted from the user's friend:
  - no frame rotation is applied in AmazingRobot
  - tags are sorted by visible area, largest first
  - preprocessing still uses enhanced gray + median blur
  - service now computes an additional solvePnP-based practical pose: `robot_forward_cm`, `robot_lateral_cm`, `yaw_practical_deg`
  - practical pose applies camera offsets and low-pass filtering with:
    - `APRILTAG_CAM_OFFSET_X_CM`
    - `APRILTAG_CAM_OFFSET_Y_CM`
    - `APRILTAG_PIXEL_ERROR_Y_GAIN`
    - `APRILTAG_ALPHA_X`
    - `APRILTAG_ALPHA_Y`
    - `APRILTAG_ALPHA_YAW`

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
- Current firmware-side I2C hardening:
  - `Wire.setClock(100000)` before `bno.begin()`
  - IMU update throttled to `100ms`
  - if one read cycle looks invalid, keep the last good heading/calibration instead of collapsing immediately to zero
  - drive firmware was later aligned closer to the known-good standalone BNO055 sketch:
    - use `bno.getEvent(&event)` and `event.orientation.x` for heading
    - wait `1000ms` before and after `setExtCrystalUse(true)` on boot
    - accept valid heading even if calibration bits are still low
  - because boot now waits longer for IMU settle, drive protocol probe timeout in Python is `3.0s` instead of `0.8s`

## Current planting behavior notes
- Final close should use `SERVO_PLANT_START_DEG`.
- Phase-2 down step uses `STEP1_AFTER_BURRIED_DEG`.
- Existing planting sequence should be preserved unless the user explicitly asks to change it.

## Current rear measurement behavior
- After 2 plants:
  - first measurement move: `C + PLANT_TO_CAMBACK_POINT_CM`
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
- Dashboard now has `Drive By Distance`:
  - posts `/api/manual/distance`
  - uses relative encoder counts from the current position
  - positive cm moves forward, negative cm moves backward
  - if IMU heading is available, it holds the current heading while driving the requested distance
  - dashboard can change the runtime drive speed norm without restart
  - the same runtime drive speed norm is also used by startup triangle `move1`
  - config keys:
    - `MANUAL_DRIVE_DISTANCE_NORM`
    - `MANUAL_DRIVE_DISTANCE_TIMEOUT_SEC`
    - `MANUAL_DRIVE_DISTANCE_MAX_CM`
- Dashboard now has `Rotate By Degrees`:
  - posts `/api/manual/rotate`
  - uses IMU heading from drive ESP32
  - computes `target = current_heading - delta_deg`
  - dashboard sign is inverted relative to IMU-positive heading
  - then rotates through the existing `_rotate_to_heading()` closed-loop helper
  - manual rotate currently uses the stable dashboard tuning path without the fine stage:
    - `MANUAL_ROTATE_TIMEOUT_SEC=14.0`
    - `MANUAL_ROTATE_HOLD_SAMPLES=2`
    - `MANUAL_ROTATE_GAIN=0.040`
    - `MANUAL_ROTATE_NORM_MAX=0.45`
    - `MANUAL_ROTATE_TOLERANCE_DEG=5.0`
  - fine-stage config keys remain in `.env`/config for compatibility, but `manual_rotate()` does not use them
- Dashboard now also has `Measure AprilTag Yaw`:
  - posts `/api/apriltag/yaw`
  - uses the physical front camera while mission is idle
  - measurement path now follows the practical AprilTag sample more closely:
    - preprocess with stronger contrast gain before detection
    - select the largest visible AprilTag first
    - compute yaw from the solvePnP practical pose path first, then fall back to pose yaw
  - `/api/status` keeps `selection_mode`, `yaw_source`, `yaw_pose_deg`, and `yaw_practical_deg` for debugging
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
