"""Flask dashboard for AmazingRobot."""

from __future__ import annotations

import logging
import time

import cv2
from flask import Flask, Response, jsonify, render_template_string, request

from app import config

logger = logging.getLogger(__name__)


class DashboardServer:
    def __init__(self, orchestrator):
        self.orchestrator = orchestrator
        self.app = Flask(__name__)
        self._setup_routes()

    def _setup_routes(self):
        @self.app.route("/")
        def index():
            return render_template_string(self._html())

        @self.app.route("/api/mission/start", methods=["POST"])
        def api_start_mission():
            ok, msg = self.orchestrator.start_mission()
            return jsonify({"ok": ok, "message": msg})

        @self.app.route("/api/mission/stop", methods=["POST"])
        def api_stop_mission():
            ok, msg = self.orchestrator.stop_mission()
            return jsonify({"ok": ok, "message": msg})

        @self.app.route("/api/manual/move/<direction>", methods=["POST"])
        def api_manual_move(direction: str):
            ok, msg = self.orchestrator.manual_move(direction)
            return jsonify({"ok": ok, "message": msg, "direction": direction})

        @self.app.route("/api/manual/rotate", methods=["POST"])
        def api_manual_rotate():
            payload = request.get_json(silent=True) or {}
            degrees = payload.get("degrees")
            ok, msg = self.orchestrator.manual_rotate(degrees)
            return jsonify({"ok": ok, "message": msg, "degrees": degrees})

        @self.app.route("/api/manual/distance", methods=["POST"])
        def api_manual_distance():
            payload = request.get_json(silent=True) or {}
            distance_cm = payload.get("distance_cm", payload.get("distance"))
            ok, msg = self.orchestrator.manual_drive_distance(distance_cm)
            return jsonify({"ok": ok, "message": msg, "distance_cm": distance_cm})

        @self.app.route("/api/tuning/manual-distance", methods=["POST"])
        def api_tuning_manual_distance():
            payload = request.get_json(silent=True) or {}
            norm = payload.get("norm")
            ok, msg = self.orchestrator.set_manual_drive_distance_norm(norm)
            return jsonify({"ok": ok, "message": msg, "norm": norm, "tuning": self.orchestrator.get_runtime_tuning()})

        @self.app.route("/api/tuning/floor-profile", methods=["POST"])
        def api_tuning_floor_profile():
            payload = request.get_json(silent=True) or {}
            profile = payload.get("profile")
            ok, msg = self.orchestrator.set_floor_profile(profile)
            return jsonify({"ok": ok, "message": msg, "profile": profile, "tuning": self.orchestrator.get_runtime_tuning()})

        @self.app.route("/api/tuning/runtime-drive", methods=["POST"])
        def api_tuning_runtime_drive():
            payload = request.get_json(silent=True) or {}
            ok, msg = self.orchestrator.set_runtime_drive_tuning(payload)
            return jsonify({"ok": ok, "message": msg, "tuning": self.orchestrator.get_runtime_tuning()})

        @self.app.route("/api/plant/run", methods=["POST"])
        def api_plant_run():
            ok, msg = self.orchestrator.run_planting_process()
            return jsonify({"ok": ok, "message": msg})

        @self.app.route("/api/measure/run", methods=["POST"])
        def api_measure_run():
            ok, msg = self.orchestrator.run_rear_measurement_process()
            return jsonify({"ok": ok, "message": msg})

        @self.app.route("/api/rear/calibrate", methods=["POST"])
        def api_rear_calibrate():
            payload = request.get_json(silent=True) or {}
            actual_cm = payload.get("actual_cm")
            ok, msg = self.orchestrator.calibrate_rear_measurement(actual_cm)
            return jsonify(
                {
                    "ok": ok,
                    "message": msg,
                    "actual_cm": actual_cm,
                    "rear_scale": self.orchestrator.get_rear_scale_info(),
                }
            )

        @self.app.route("/api/rear/calibrate/save", methods=["POST"])
        def api_rear_calibrate_save():
            ok, msg = self.orchestrator.save_rear_scale_to_env()
            return jsonify(
                {
                    "ok": ok,
                    "message": msg,
                    "rear_scale": self.orchestrator.get_rear_scale_info(),
                }
            )

        @self.app.route("/api/apriltag/yaw", methods=["POST"])
        def api_apriltag_yaw():
            ok, msg = self.orchestrator.run_apriltag_yaw_measurement()
            return jsonify({"ok": ok, "message": msg})

        @self.app.route("/api/camera/swap/toggle", methods=["POST"])
        def api_camera_swap_toggle():
            ok, msg = self.orchestrator.toggle_camera_swap()
            return jsonify({"ok": ok, "message": msg, "camera": self.orchestrator.get_camera_info()})

        @self.app.route("/api/status")
        def api_status():
            snap = self.orchestrator.get_snapshot().to_dict()
            snap["actuator"] = self.orchestrator.actuator.get_status()
            snap["drive_link"] = self.orchestrator.drive.get_status()
            snap["camera"] = self.orchestrator.get_camera_info()
            snap["imu"] = self.orchestrator.drive.get_imu_status()
            snap["tuning"] = self.orchestrator.get_runtime_tuning()
            snap["rear_scale"] = self.orchestrator.get_rear_scale_info()
            return jsonify(snap)

        @self.app.route("/api/config")
        def api_config():
            return jsonify(
                {
                    "model_path": config.VISION.model_path,
                    "target_classes": list(config.VISION.target_classes),
                    "priority_class": config.VISION.priority_class,
                    "bbox_trigger_line_y": config.VISION.bbox_trigger_line_y,
                    "front_camera_index": config.CAMERA.front_index,
                    "rear_camera_index": config.CAMERA.rear_index,
                    "camera_swap": self.orchestrator.get_camera_info().get("swap", config.CAMERA.swap),
                    "front_to_plant_point_cm": config.MOTION.front_to_plant_point_cm,
                    "servo_cam_start_deg": config.ACTUATOR.servo_cam_start_deg,
                    "servo_cam_end_deg": config.ACTUATOR.servo_cam_end_deg,
                    "servo_cam_step_deg": config.ACTUATOR.servo_cam_step_deg,
                    "servo_cam_step_interval_sec": config.ACTUATOR.servo_cam_step_interval_sec,
                    "servo_cam_fast_px_threshold": config.ACTUATOR.servo_cam_fast_px_threshold,
                    "servo_cam_fast_max_multiplier": config.ACTUATOR.servo_cam_fast_max_multiplier,
                    "servo_plant_degree": config.ACTUATOR.servo_plant_degree,
                    "servo_plant_after_deg": config.ACTUATOR.servo_plant_after_deg,
                    "plant_limit_timeout_sec": config.ACTUATOR.plant_limit_timeout_sec,
                    "plant_limit_grace_sec": config.ACTUATOR.plant_limit_grace_sec,
                    "rear_measure_target_class": config.VISION.rear_measure_target_class,
                    "rear_measure_frames": config.VISION.rear_measure_frames,
                    "rear_measure_timeout_sec": config.VISION.rear_measure_timeout_sec,
                    "rear_measure_size_threshold_cm": config.VISION.rear_measure_size_threshold_cm,
                    "apriltag_size_cm": config.VISION.apriltag_size_cm,
                    "rear_cm_per_px_default": config.VISION.rear_cm_per_px_default,
                    "approach_min_norm": config.VISION.approach_min_norm,
                    "approach_max_norm": config.VISION.approach_max_norm,
                    "entry_align_lateral_tol_px": config.VISION.entry_align_lateral_tol_px,
                    "entry_align_heading_tol_norm": config.VISION.entry_align_heading_tol_norm,
                    "entry_align_hold_frames": config.VISION.entry_align_hold_frames,
                    "stuck_window_sec": config.VISION.stuck_window_sec,
                    "stuck_min_progress_cm": config.VISION.stuck_min_progress_cm,
                    "stuck_max_retries": config.VISION.stuck_max_retries,
                    "imu_enabled": config.IMU.enabled,
                    "imu_type": config.IMU.type,
                }
            )

        @self.app.route("/video/front")
        def front_video_feed():
            return Response(self._mjpeg_generator("front"), mimetype="multipart/x-mixed-replace; boundary=frame")

        @self.app.route("/video/rear")
        def rear_video_feed():
            return Response(self._mjpeg_generator("rear"), mimetype="multipart/x-mixed-replace; boundary=frame")

    def _mjpeg_generator(self, which: str):
        while True:
            frame = self.orchestrator.get_front_frame() if which == "front" else self.orchestrator.get_rear_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            ok, buf = cv2.imencode(".jpg", frame)
            if not ok:
                time.sleep(0.05)
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
            )
            time.sleep(0.03)

    def _html(self) -> str:
        return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>AmazingRobot Dashboard</title>
  <style>
    :root {
      --bg: #0a1220;
      --bg2: #101a2f;
      --card: rgba(11, 18, 32, 0.92);
      --card2: rgba(16, 26, 47, 0.95);
      --text: #e5edf7;
      --muted: #8ea0bd;
      --line: rgba(148, 163, 184, 0.18);
      --blue: #3b82f6;
      --cyan: #22d3ee;
      --green: #22c55e;
      --yellow: #f59e0b;
      --red: #ef4444;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      color: var(--text);
      font-family: "Segoe UI", Tahoma, Geneva, Verdana, sans-serif;
      background: radial-gradient(circle at top left, #1d3557 0%, var(--bg) 48%), linear-gradient(160deg, #0a1220 0%, #10192b 100%);
      padding: 18px;
    }
    .container { max-width: 1500px; margin: 0 auto; }
    .hero {
      display: flex;
      justify-content: space-between;
      align-items: end;
      gap: 12px;
      margin-bottom: 14px;
    }
    .hero h2 { margin: 0; font-size: 2rem; }
    .subtitle { color: var(--muted); font-size: 0.95rem; margin-top: 4px; }
    .toolbar, .toolbar-tight { display: flex; gap: 10px; flex-wrap: wrap; margin-bottom: 12px; }
    .toolbar-tight { align-items: center; }
    button {
      border: 0;
      border-radius: 12px;
      padding: 10px 14px;
      font-weight: 700;
      color: white;
      cursor: pointer;
      background: linear-gradient(180deg, #4f8cff, #2563eb);
      box-shadow: 0 8px 24px rgba(37, 99, 235, 0.24);
    }
    button.stop { background: linear-gradient(180deg, #f87171, #dc2626); box-shadow: 0 8px 24px rgba(220, 38, 38, 0.22); }
    input, select {
      border: 1px solid var(--line);
      border-radius: 10px;
      padding: 10px 12px;
      background: #08111f;
      color: var(--text);
      min-width: 130px;
      font-weight: 600;
    }
    .overview {
      display: grid;
      grid-template-columns: repeat(6, minmax(0, 1fr));
      gap: 12px;
      margin-bottom: 12px;
    }
    .stat-card, .card {
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: 16px;
      box-shadow: 0 12px 30px rgba(0,0,0,0.2);
    }
    .stat-card { padding: 14px; min-height: 100px; }
    .stat-label { color: var(--muted); font-size: 0.78rem; text-transform: uppercase; letter-spacing: 0.08em; }
    .stat-value { font-size: 1.5rem; font-weight: 800; margin-top: 8px; }
    .stat-meta { color: var(--muted); font-size: 0.88rem; margin-top: 6px; }
    .grid { display: grid; grid-template-columns: 1.2fr 1.2fr 1fr; gap: 12px; }
    .card { padding: 14px; }
    .card h3 { margin: 0 0 12px; }
    .card h4 { margin: 0 0 8px; color: var(--muted); font-size: 0.86rem; text-transform: uppercase; letter-spacing: 0.08em; }
    .camera-img { width: 100%; border-radius: 12px; border: 1px solid rgba(59,130,246,0.22); background: #020817; }
    .status-stack { display: grid; gap: 10px; }
    .status-banner {
      padding: 12px;
      border-radius: 12px;
      background: var(--card2);
      border: 1px solid rgba(34, 211, 238, 0.18);
    }
    .status-banner .big { font-size: 1.15rem; font-weight: 800; }
    .kv-grid { display: grid; grid-template-columns: repeat(2, minmax(0,1fr)); gap: 10px; }
    .kv { padding: 10px; border-radius: 12px; background: rgba(15, 23, 42, 0.72); border: 1px solid var(--line); }
    .kv .k { color: var(--muted); font-size: 0.76rem; text-transform: uppercase; letter-spacing: 0.06em; }
    .kv .v { margin-top: 6px; font-size: 1rem; font-weight: 700; word-break: break-word; }
    .tuning-help {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
      margin: -2px 0 12px;
    }
    .help-item {
      padding: 10px 12px;
      border-radius: 12px;
      background: rgba(15, 23, 42, 0.72);
      border: 1px solid var(--line);
    }
    .help-item .name {
      color: #e5eefc;
      font-size: 0.82rem;
      font-weight: 800;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }
    .help-item .desc {
      color: var(--muted);
      font-size: 0.88rem;
      line-height: 1.35;
      margin-top: 6px;
    }
    pre {
      margin: 0;
      white-space: pre-wrap;
      word-break: break-word;
      color: #c5d3e5;
      font-size: 12px;
      line-height: 1.45;
      background: #07111f;
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 12px;
      max-height: 280px;
      overflow: auto;
    }
    .charts { margin-top: 12px; display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    canvas { width: 100%; height: 220px; border: 1px solid var(--line); border-radius: 12px; background: #07111f; }
    .logs { margin-top: 12px; display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .ok { color: var(--green); }
    .warn { color: var(--yellow); }
    .bad { color: var(--red); }
    @media (max-width: 1200px) {
      .overview { grid-template-columns: repeat(3, minmax(0, 1fr)); }
      .grid { grid-template-columns: 1fr; }
      .charts, .logs { grid-template-columns: 1fr; }
    }
    @media (max-width: 760px) {
      .overview { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .kv-grid { grid-template-columns: 1fr; }
      .tuning-help { grid-template-columns: 1fr; }
      .hero { flex-direction: column; align-items: start; }
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="hero">
      <div>
        <h2>AmazingRobot Mission Dashboard</h2>
        <div class="subtitle">Mission state, AprilTag pose, drive telemetry, and raw serial logs separated for quicker diagnosis.</div>
      </div>
    </div>

    <div class="toolbar">
      <button onclick="startMission()">Start Mission</button>
      <button class="stop" onclick="stopMission()">Stop Mission</button>
      <button onclick="runPlanting()">Run Planting Process</button>
      <button onclick="runRearMeasure()">Measure Cabbage Size</button>
      <button onclick="runAprilTagYaw()">Measure AprilTag Yaw</button>
    </div>

    <div class="toolbar-tight">
      <input id="rearActualCm" type="number" min="0.1" step="0.1" value="16.0" placeholder="Actual rear size cm" />
      <button onclick="calibrateRearSize()">Calibrate Rear Size</button>
      <button onclick="saveRearScaleToEnv()">Save Rear Scale to .env</button>
      <div class="subtitle">Put a known object under the rear camera, enter its real size in cm, then apply to update rear `cm/px` for the system.</div>
    </div>

    <div class="toolbar">
      <button onclick="manualMove('forward')">Forward</button>
      <button onclick="manualMove('backward')">Backward</button>
      <button onclick="manualMove('left')">Left</button>
      <button onclick="manualMove('right')">Right</button>
      <button onclick="manualMove('stop')">Stop</button>
      <button onclick="toggleCameraSwap()">Switch Camera</button>
    </div>

    <div class="toolbar-tight">
      <input id="rotateDegrees" type="number" step="1" value="90" placeholder="Degrees" />
      <button onclick="manualRotateDegrees()">Rotate By Degrees</button>
    </div>

    <div class="toolbar-tight">
      <input id="driveDistanceCm" type="number" step="1" value="30" placeholder="Distance cm" />
      <button onclick="manualDriveDistance()">Drive By Distance</button>
      <div class="subtitle">Positive = forward, negative = backward</div>
    </div>

    <div class="toolbar-tight">
      <input id="driveSpeedNorm" type="number" min="0.05" max="1.0" step="0.01" value="0.24" placeholder="Drive speed norm" />
      <button onclick="setDriveSpeedNorm()">Set Drive Speed</button>
      <div class="subtitle">Affects Drive By Distance and triangle move1</div>
    </div>

    <div class="toolbar-tight">
      <select id="floorProfile">
        <option value="smooth">Smooth Floor</option>
        <option value="normal" selected>Normal Floor</option>
        <option value="rough">Rough Floor</option>
      </select>
      <button onclick="setFloorProfile()">Apply Floor Profile</button>
      <div class="subtitle">Changes approach, rear-measure, preplant slow zone, manual drive, and rotate min/max together</div>
    </div>

    <div class="toolbar-tight">
      <input id="approachMinNorm" type="number" min="0.05" max="1.0" step="0.01" value="0.18" placeholder="Approach min" />
      <input id="approachMaxNorm" type="number" min="0.05" max="1.0" step="0.01" value="0.45" placeholder="Approach max" />
      <input id="measureSpeedNorm" type="number" min="0.05" max="1.0" step="0.01" value="0.28" placeholder="Rear measure speed" />
      <input id="preplantSlowNorm" type="number" min="0.05" max="1.0" step="0.01" value="0.20" placeholder="Preplant slow" />
      <input id="manualRotateNormMin" type="number" min="0.05" max="1.0" step="0.01" value="0.16" placeholder="Rotate min" />
      <input id="manualRotateNormMax" type="number" min="0.05" max="1.0" step="0.01" value="0.45" placeholder="Rotate max" />
      <button onclick="applyCustomDriveTune()">Apply Custom Tune</button>
    </div>
    <div class="tuning-help">
      <div class="help-item">
        <div class="name">Approach Min</div>
        <div class="desc">ความเร็วต่ำสุดตอนขับหาเป้าหมายด้านหน้า เพิ่มค่านี้ถ้าหุ่นเริ่มไหลช้าเกินหรือแรงไม่พอบนพื้นฝืด</div>
      </div>
      <div class="help-item">
        <div class="name">Approach Max</div>
        <div class="desc">เพดานความเร็วตอน approach/search เพิ่มเมื่ออยากให้เข้าหาเป้าหมายเร็วขึ้น แต่ถ้าสูงเกินจะ overshoot ง่าย</div>
      </div>
      <div class="help-item">
        <div class="name">Rear Measure Speed</div>
        <div class="desc">ความเร็ววิ่งช่วงไปจุดวัดด้านหลัง ลดลงถ้ากล้องหลังจับไม่ทัน เพิ่มขึ้นถ้าระบบช้าเกินโดยที่ยังเห็นภาพทัน</div>
      </div>
      <div class="help-item">
        <div class="name">Preplant Slow</div>
        <div class="desc">ความเร็วชะลอในโซนรอ rear AprilTag ก่อน adjust ค่านี้ต่ำลงจะช่วยให้ไม่วิ่งเลย tag ตอน FPS น้อย</div>
      </div>
      <div class="help-item">
        <div class="name">Rotate Min</div>
        <div class="desc">แรงหมุนขั้นต่ำตอนใกล้เป้าหมาย ใช้แก้อาการหมุนท้ายไม่ไปต่อหรือ timeout บนพื้นหยาบ</div>
      </div>
      <div class="help-item">
        <div class="name">Rotate Max</div>
        <div class="desc">เพดานแรงหมุนทั้งหมด เพิ่มเมื่อพื้นใหม่กินแรงมาก แต่ถ้าสูงเกินจะสะบัดหรือหมุนเลยเป้าได้</div>
      </div>
    </div>

    <div class="overview">
      <div class="stat-card"><div class="stat-label">Mission State</div><div class="stat-value" id="ovState">-</div><div class="stat-meta" id="ovRunning">-</div></div>
      <div class="stat-card"><div class="stat-label">Status</div><div class="stat-value" id="ovStatus">-</div><div class="stat-meta" id="ovTarget">-</div></div>
      <div class="stat-card"><div class="stat-label">Heading</div><div class="stat-value" id="ovHeading">-</div><div class="stat-meta" id="ovImuCal">-</div></div>
      <div class="stat-card"><div class="stat-label">Drive Distance</div><div class="stat-value" id="ovDistance">-</div><div class="stat-meta" id="ovCounts">-</div></div>
      <div class="stat-card"><div class="stat-label">AprilTag</div><div class="stat-value" id="ovApril">-</div><div class="stat-meta" id="ovAprilMeta">-</div></div>
      <div class="stat-card"><div class="stat-label">Actuator</div><div class="stat-value" id="ovActuator">-</div><div class="stat-meta" id="ovServos">-</div></div>
    </div>

    <div class="grid">
      <div class="card">
        <h3>Front Camera</h3>
        <img class="camera-img" src="/video/front" alt="front" />
      </div>
      <div class="card">
        <h3>Rear Camera</h3>
        <img class="camera-img" src="/video/rear" alt="rear" />
      </div>
      <div class="card">
        <h3>Mission Summary</h3>
        <div class="status-stack">
          <div class="status-banner">
            <div class="stat-label">Current Status</div>
            <div class="big" id="statusText">loading...</div>
          </div>
          <div class="kv-grid" id="summaryGrid"></div>
        </div>
      </div>
    </div>

    <div class="charts">
      <div class="card">
        <h3>PID Graph</h3>
        <canvas id="pidChart" width="560" height="220"></canvas>
      </div>
      <div class="card">
        <h3>AprilTag Yaw</h3>
        <pre id="apriltagYaw">No measurement yet</pre>
      </div>
    </div>

    <div class="charts">
      <div class="card">
        <h3>Plant Map</h3>
        <canvas id="plantMap" width="560" height="220"></canvas>
      </div>
      <div class="card">
        <h3>Compact Status JSON</h3>
        <pre id="statusCore">loading...</pre>
      </div>
    </div>

    <div class="logs">
      <div class="card">
        <h3>Drive Link Logs</h3>
        <pre id="driveLog">loading...</pre>
      </div>
      <div class="card">
        <h3>Actuator / Raw Logs</h3>
        <pre id="miscLog">loading...</pre>
      </div>
    </div>
  </div>

  <script>
    async function startMission() { await fetch('/api/mission/start', { method: 'POST' }); }
    async function stopMission() { await fetch('/api/mission/stop', { method: 'POST' }); }

    async function manualMove(direction) {
      const r = await fetch('/api/manual/move/' + direction, { method: 'POST' });
      const data = await r.json();
      if (!data.ok) console.error('manual move failed:', data.message);
    }

    async function runPlanting() {
      const r = await fetch('/api/plant/run', { method: 'POST' });
      const data = await r.json();
      if (!data.ok) console.error('planting failed:', data.message);
    }

    async function runRearMeasure() {
      const r = await fetch('/api/measure/run', { method: 'POST' });
      const data = await r.json();
      if (!data.ok) console.error('rear measure failed:', data.message);
    }

    async function calibrateRearSize() {
      const el = document.getElementById('rearActualCm');
      const actual_cm = Number(el.value);
      const r = await fetch('/api/rear/calibrate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ actual_cm })
      });
      const data = await r.json();
      if (!data.ok) console.error('rear calibration failed:', data.message);
      await refresh();
    }

    async function saveRearScaleToEnv() {
      const r = await fetch('/api/rear/calibrate/save', { method: 'POST' });
      const data = await r.json();
      if (!data.ok) console.error('save rear scale failed:', data.message);
      await refresh();
    }

    async function runAprilTagYaw() {
      const r = await fetch('/api/apriltag/yaw', { method: 'POST' });
      const data = await r.json();
      if (!data.ok) console.error('apriltag yaw failed:', data.message);
      await refresh();
    }

    async function manualRotateDegrees() {
      const el = document.getElementById('rotateDegrees');
      const degrees = Number(el.value);
      const r = await fetch('/api/manual/rotate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ degrees })
      });
      const data = await r.json();
      if (!data.ok) console.error('manual rotate failed:', data.message);
    }

    async function manualDriveDistance() {
      const el = document.getElementById('driveDistanceCm');
      const distance_cm = Number(el.value);
      const r = await fetch('/api/manual/distance', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ distance_cm })
      });
      const data = await r.json();
      if (!data.ok) console.error('manual distance failed:', data.message);
      await refresh();
    }

    async function setDriveSpeedNorm() {
      const el = document.getElementById('driveSpeedNorm');
      const norm = Number(el.value);
      const r = await fetch('/api/tuning/manual-distance', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ norm })
      });
      const data = await r.json();
      if (!data.ok) console.error('set drive speed failed:', data.message);
      await refresh();
    }

    async function setFloorProfile() {
      const el = document.getElementById('floorProfile');
      const profile = String(el.value || 'normal');
      const r = await fetch('/api/tuning/floor-profile', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ profile })
      });
      const data = await r.json();
      if (!data.ok) console.error('set floor profile failed:', data.message);
      await refresh();
    }

    async function applyCustomDriveTune() {
      const payload = {
        approach_min_norm: Number(document.getElementById('approachMinNorm').value),
        approach_max_norm: Number(document.getElementById('approachMaxNorm').value),
        move_to_measure_speed_norm: Number(document.getElementById('measureSpeedNorm').value),
        preplant_adjust_slow_norm: Number(document.getElementById('preplantSlowNorm').value),
        manual_drive_distance_norm: Number(document.getElementById('driveSpeedNorm').value),
        manual_rotate_norm_min: Number(document.getElementById('manualRotateNormMin').value),
        manual_rotate_norm_max: Number(document.getElementById('manualRotateNormMax').value),
      };
      const r = await fetch('/api/tuning/runtime-drive', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      const data = await r.json();
      if (!data.ok) console.error('apply custom drive tune failed:', data.message);
      await refresh();
    }

    async function toggleCameraSwap() {
      const r = await fetch('/api/camera/swap/toggle', { method: 'POST' });
      const data = await r.json();
      if (!data.ok) console.error('camera swap failed:', data.message);
      await refresh();
    }

    function fmt(v, suffix='') {
      if (v === null || v === undefined || v === '') return '-';
      if (typeof v === 'number') return `${v.toFixed(2)}${suffix}`;
      return `${v}${suffix}`;
    }

    function renderSummaryGrid(items) {
      const root = document.getElementById('summaryGrid');
      root.innerHTML = '';
      for (const [k, v] of items) {
        const box = document.createElement('div');
        box.className = 'kv';
        box.innerHTML = `<div class="k">${k}</div><div class="v">${v}</div>`;
        root.appendChild(box);
      }
    }

    function drawPid(history) {
      const cvs = document.getElementById('pidChart');
      if (!cvs) return;
      const ctx = cvs.getContext('2d');
      const w = cvs.width, h = cvs.height;
      ctx.clearRect(0, 0, w, h);
      ctx.strokeStyle = '#334155';
      ctx.strokeRect(0, 0, w, h);

      if (!history || history.length < 2) return;
      const items = history.slice(-100);
      let minY = 1e9, maxY = -1e9;
      for (const p of items) {
        minY = Math.min(minY, p.turn ?? 0, p.error ?? 0);
        maxY = Math.max(maxY, p.turn ?? 0, p.error ?? 0);
      }
      if (Math.abs(maxY - minY) < 1e-6) { maxY += 1; minY -= 1; }

      const mapX = (i) => (i / (items.length - 1)) * (w - 20) + 10;
      const mapY = (v) => (h - 15) - ((v - minY) / (maxY - minY)) * (h - 30);

      ctx.lineWidth = 2;
      ctx.strokeStyle = '#22d3ee';
      ctx.beginPath();
      items.forEach((p, i) => {
        const x = mapX(i), y = mapY(p.turn ?? 0);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();

      ctx.strokeStyle = '#f59e0b';
      ctx.beginPath();
      items.forEach((p, i) => {
        const x = mapX(i), y = mapY(p.error ?? 0);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();
    }

    function drawPlantMap(points) {
      const cvs = document.getElementById('plantMap');
      if (!cvs) return;
      const ctx = cvs.getContext('2d');
      const w = cvs.width, h = cvs.height;
      ctx.clearRect(0, 0, w, h);
      ctx.strokeStyle = '#334155';
      ctx.strokeRect(0, 0, w, h);
      ctx.strokeStyle = '#64748b';
      ctx.beginPath();
      ctx.moveTo(20, h/2);
      ctx.lineTo(w-20, h/2);
      ctx.stroke();
      ctx.fillStyle = '#94a3b8';
      ctx.font = '11px monospace';
      ctx.fillText('distance', 24, h/2 - 14);

      if (!points || points.length === 0) return;
      const items = points.slice(-40);
      const xs = items.map(p => Number(p.distance_cm ?? 0));
      let minX = Math.min(...xs), maxX = Math.max(...xs);
      if (Math.abs(maxX - minX) < 1e-6) { maxX += 1; minX -= 1; }
      const mapX = (v) => ((v - minX) / (maxX - minX)) * (w - 60) + 30;

      items.forEach((p, idx) => {
        const x = mapX(Number(p.distance_cm ?? 0));
        const ok = !!p.reached;
        ctx.fillStyle = ok ? '#22c55e' : '#ef4444';
        ctx.beginPath();
        ctx.arc(x, h/2, 6, 0, Math.PI*2);
        ctx.fill();
        ctx.fillStyle = '#cbd5e1';
        ctx.font = '11px monospace';
        ctx.fillText(String(p.idx ?? idx+1), x-4, h/2 - 10);
        const sizeText = (p.size_cm === null || p.size_cm === undefined)
          ? 'n/a'
          : `${Number(p.size_cm).toFixed(1)} cm`;
        ctx.fillStyle = ok ? '#86efac' : '#fca5a5';
        ctx.fillText(sizeText, x - 18, h/2 + 22);
      });
    }

    async function refresh() {
      try {
        const r = await fetch('/api/status');
        const data = await r.json();

        document.getElementById('ovState').textContent = data.state || '-';
        document.getElementById('ovRunning').textContent = data.running ? 'Mission running' : 'Mission idle';
        document.getElementById('ovStatus').textContent = (data.status_text || '-').slice(0, 42);
        document.getElementById('ovTarget').textContent = data.target_class ? `target=${data.target_class}` : 'no target';
        document.getElementById('ovHeading').textContent = fmt(data.drive?.heading_deg, ' deg');
        const cal = data.imu?.calibration_status || {};
        document.getElementById('ovImuCal').textContent = `sys ${cal.sys ?? '-'} / gyro ${cal.gyro ?? '-'} / accel ${cal.accel ?? '-'} / mag ${cal.mag ?? '-'}`;
        document.getElementById('ovDistance').textContent = fmt(data.drive?.distance_cm, ' cm');
        document.getElementById('ovCounts').textContent = `L ${data.drive?.left_count ?? '-'} / R ${data.drive?.right_count ?? '-'}`;
        document.getElementById('ovApril').textContent = data.april ? `tag ${data.april.tag_id}` : 'no tag';
        document.getElementById('ovAprilMeta').textContent = data.april ? `AB ${data.april.planting_distance_cm} / C ${data.april.spacing_gap_cm} / DE ${data.april.cabbage_interval_cm}` : 'waiting';
        document.getElementById('ovActuator').textContent = data.actuator?.connected ? 'connected' : 'offline';
        document.getElementById('ovServos').textContent = `cam ${data.actuator?.servo_cam_deg ?? '-'} / plant ${data.actuator?.servo_plant_deg ?? '-'}`;

        document.getElementById('statusText').textContent = data.status_text || 'No status';
        renderSummaryGrid([
          ['Mission', `${data.state || '-'} / ${data.running ? 'running' : 'idle'}`],
          ['Target', data.target_class || '-'],
          ['Camera', `front=${data.camera?.effective_front_source || '-'} rear=${data.camera?.effective_rear_source || '-'}`],
          ['Drive RPM', `L ${fmt(data.drive?.left_rpm)} / R ${fmt(data.drive?.right_rpm)}`],
          ['Rear Scale', data.rear_scale?.cm_per_px ? `${fmt(data.rear_scale.cm_per_px)} (${data.rear_scale.source || '-'})` : '-'],
          ['Floor Profile', data.tuning?.floor_profile || '-'],
          ['Drive Speed', fmt(data.tuning?.manual_drive_distance_norm)],
          ['IMU', data.imu?.connected ? `heading ${fmt(data.imu?.heading_deg, ' deg')}` : 'not connected'],
          ['AprilTag Pose', data.apriltag_yaw_measurement ? `yaw ${fmt(data.apriltag_yaw_measurement.yaw_practical_deg ?? data.apriltag_yaw_measurement.yaw_deg, ' deg')}` : 'no measurement'],
        ]);

        const driveSpeedEl = document.getElementById('driveSpeedNorm');
        if (driveSpeedEl && document.activeElement !== driveSpeedEl && data.tuning?.manual_drive_distance_norm !== undefined) {
          driveSpeedEl.value = Number(data.tuning.manual_drive_distance_norm).toFixed(2);
        }
        const approachMinEl = document.getElementById('approachMinNorm');
        if (approachMinEl && document.activeElement !== approachMinEl && data.tuning?.approach_min_norm !== undefined) {
          approachMinEl.value = Number(data.tuning.approach_min_norm).toFixed(2);
        }
        const approachMaxEl = document.getElementById('approachMaxNorm');
        if (approachMaxEl && document.activeElement !== approachMaxEl && data.tuning?.approach_max_norm !== undefined) {
          approachMaxEl.value = Number(data.tuning.approach_max_norm).toFixed(2);
        }
        const measureSpeedEl = document.getElementById('measureSpeedNorm');
        if (measureSpeedEl && document.activeElement !== measureSpeedEl && data.tuning?.move_to_measure_speed_norm !== undefined) {
          measureSpeedEl.value = Number(data.tuning.move_to_measure_speed_norm).toFixed(2);
        }
        const preplantSlowEl = document.getElementById('preplantSlowNorm');
        if (preplantSlowEl && document.activeElement !== preplantSlowEl && data.tuning?.preplant_adjust_slow_norm !== undefined) {
          preplantSlowEl.value = Number(data.tuning.preplant_adjust_slow_norm).toFixed(2);
        }
        const rotateMinEl = document.getElementById('manualRotateNormMin');
        if (rotateMinEl && document.activeElement !== rotateMinEl && data.tuning?.manual_rotate_norm_min !== undefined) {
          rotateMinEl.value = Number(data.tuning.manual_rotate_norm_min).toFixed(2);
        }
        const rotateNormEl = document.getElementById('manualRotateNormMax');
        if (rotateNormEl && document.activeElement !== rotateNormEl && data.tuning?.manual_rotate_norm_max !== undefined) {
          rotateNormEl.value = Number(data.tuning.manual_rotate_norm_max).toFixed(2);
        }
        const floorProfileEl = document.getElementById('floorProfile');
        if (floorProfileEl && document.activeElement !== floorProfileEl && data.tuning?.floor_profile) {
          floorProfileEl.value = String(data.tuning.floor_profile);
        }

        const core = {
          state: data.state,
          running: data.running,
          status_text: data.status_text,
          target_class: data.target_class,
          april: data.april,
          apriltag_yaw_measurement: data.apriltag_yaw_measurement,
          drive: data.drive,
          imu: data.imu,
          tuning: data.tuning,
          actuator: {
            connected: data.actuator?.connected,
            servo_cam_deg: data.actuator?.servo_cam_deg,
            servo_plant_deg: data.actuator?.servo_plant_deg,
            port: data.actuator?.port,
          },
          camera: data.camera,
        };
        document.getElementById('statusCore').textContent = JSON.stringify(core, null, 2);
        document.getElementById('apriltagYaw').textContent = JSON.stringify(data.apriltag_yaw_measurement || {}, null, 2);

        const driveLog = {
          protocol_version: data.drive_link?.protocol_version,
          protocol_caps: data.drive_link?.protocol_caps,
          last_stat_age_sec: data.drive_link?.last_stat_age_sec,
          raw_line: data.drive_link?.raw_line,
          recent_lines: data.drive_link?.recent_lines,
        };
        document.getElementById('driveLog').textContent = JSON.stringify(driveLog, null, 2);

        const miscLog = {
          actuator_last_message: data.actuator?.last_message,
          actuator_caps: data.actuator?.protocol_caps,
          pid_tail: (data.pid && data.pid.history) ? data.pid.history.slice(-12) : [],
        };
        document.getElementById('miscLog').textContent = JSON.stringify(miscLog, null, 2);

        drawPid((data.pid && data.pid.history) ? data.pid.history : []);
        drawPlantMap(data.plant_map || []);
      } catch (e) {
        document.getElementById('statusText').textContent = 'status error: ' + e;
      }
    }

    setInterval(refresh, 500);
    refresh();
  </script>
</body>
</html>
"""

    def run(self) -> None:
        logger.info("Dashboard running at http://%s:%d", config.WEB.host, config.WEB.port)
        self.app.run(host=config.WEB.host, port=config.WEB.port, debug=config.WEB.debug, threaded=True)
