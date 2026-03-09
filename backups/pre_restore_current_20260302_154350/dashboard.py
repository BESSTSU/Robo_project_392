"""Flask dashboard for AmazingRobot."""

from __future__ import annotations

import logging
import time

import cv2
from flask import Flask, Response, jsonify, render_template_string

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

        @self.app.route("/api/plant/run", methods=["POST"])
        def api_plant_run():
            ok, msg = self.orchestrator.run_planting_process()
            return jsonify({"ok": ok, "message": msg})

        @self.app.route("/api/measure/run", methods=["POST"])
        def api_measure_run():
            ok, msg = self.orchestrator.run_rear_measurement_process()
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
                    "entry_align_require_both": config.VISION.entry_align_require_both,
                    "stuck_window_sec": config.VISION.stuck_window_sec,
                    "stuck_min_progress_cm": config.VISION.stuck_min_progress_cm,
                    "stuck_max_retries": config.VISION.stuck_max_retries,
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
      --bg: #0f172a;
      --card: #111827;
      --text: #e5e7eb;
      --muted: #94a3b8;
      --ok: #16a34a;
      --warn: #f59e0b;
      --bad: #ef4444;
      --line: #1f2937;
      --btn: #2563eb;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: radial-gradient(circle at 15% 20%, #1e293b, #0f172a 60%);
      color: var(--text);
      font-family: "Segoe UI", Tahoma, Geneva, Verdana, sans-serif;
      padding: 16px;
    }
    .container { max-width: 1300px; margin: 0 auto; }
    .title { margin: 0 0 12px; }
    .grid { display: grid; grid-template-columns: 2fr 2fr 1.5fr; gap: 12px; }
    .card {
      background: rgba(17,24,39,.92);
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 12px;
      min-height: 120px;
    }
    img { width: 100%; border-radius: 8px; border: 1px solid #334155; }
    .btns { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 12px; }
    .btns + .btns { margin-top: -4px; }
    button {
      border: 0;
      border-radius: 8px;
      padding: 10px 14px;
      font-weight: 700;
      color: white;
      cursor: pointer;
      background: var(--btn);
    }
    button.stop { background: var(--bad); }
    pre {
      margin: 0;
      white-space: pre-wrap;
      word-break: break-word;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.45;
    }
    canvas {
      width: 100%;
      height: 220px;
      border: 1px solid #334155;
      border-radius: 8px;
      background: #0b1220;
    }
    .charts {
      margin-top: 12px;
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
    }
    @media (max-width: 1000px) {
      .grid { grid-template-columns: 1fr; }
      .charts { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="container">
    <h2 class="title">AmazingRobot Mission Dashboard</h2>
    <div class="btns">
      <button onclick="startMission()">Start Mission</button>
      <button class="stop" onclick="stopMission()">Stop Mission</button>
    </div>
    <div class="btns">
      <button onclick="manualMove('forward')">Forward</button>
      <button onclick="manualMove('backward')">Backward</button>
      <button onclick="manualMove('left')">Left</button>
      <button onclick="manualMove('right')">Right</button>
      <button onclick="manualMove('stop')">Stop</button>
      <button onclick="runPlanting()">Run Planting Process</button>
      <button onclick="runRearMeasure()">Measure Cabbage Size</button>
      <button onclick="toggleCameraSwap()">Switch Camera</button>
    </div>

    <div class="grid">
      <div class="card">
        <h3>Front Camera</h3>
        <img src="/video/front" alt="front" />
      </div>
      <div class="card">
        <h3>Rear Camera</h3>
        <img src="/video/rear" alt="rear" />
      </div>
      <div class="card">
        <h3>Status</h3>
        <pre id="status">loading...</pre>
      </div>
    </div>

    <div class="charts">
      <div class="card">
        <h3>PID Graph</h3>
        <canvas id="pidChart" width="560" height="220"></canvas>
      </div>
      <div class="card">
        <h3>Plant Map</h3>
        <canvas id="plantMap" width="560" height="220"></canvas>
      </div>
    </div>
  </div>

  <script>
    async function startMission() {
      await fetch('/api/mission/start', { method: 'POST' });
    }
    async function stopMission() {
      await fetch('/api/mission/stop', { method: 'POST' });
    }

    async function manualMove(direction) {
      const r = await fetch('/api/manual/move/' + direction, { method: 'POST' });
      const data = await r.json();
      if (!data.ok) {
        console.error('manual move failed:', data.message);
      }
    }

    async function runPlanting() {
      const r = await fetch('/api/plant/run', { method: 'POST' });
      const data = await r.json();
      if (!data.ok) {
        console.error('planting failed:', data.message);
      }
    }

    async function runRearMeasure() {
      const r = await fetch('/api/measure/run', { method: 'POST' });
      const data = await r.json();
      if (!data.ok) {
        console.error('rear measure failed:', data.message);
      }
    }

    async function toggleCameraSwap() {
      const r = await fetch('/api/camera/swap/toggle', { method: 'POST' });
      const data = await r.json();
      if (!data.ok) {
        console.error('camera swap failed:', data.message);
      }
      await refresh();
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
      ctx.strokeStyle = '#22d3ee'; // turn
      ctx.beginPath();
      items.forEach((p, i) => {
        const x = mapX(i), y = mapY(p.turn ?? 0);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();

      ctx.strokeStyle = '#f59e0b'; // error
      ctx.beginPath();
      items.forEach((p, i) => {
        const x = mapX(i), y = mapY(p.error ?? 0);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();

      ctx.fillStyle = '#94a3b8';
      ctx.font = '12px monospace';
      ctx.fillText('turn', 12, 16);
      ctx.fillText('error', 60, 16);
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
      });
    }

    async function refresh() {
      try {
        const r = await fetch('/api/status');
        const data = await r.json();
        document.getElementById('status').textContent = JSON.stringify(data, null, 2);
        drawPid((data.pid && data.pid.history) ? data.pid.history : []);
        drawPlantMap(data.plant_map || []);
      } catch (e) {
        document.getElementById('status').textContent = 'status error: ' + e;
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
