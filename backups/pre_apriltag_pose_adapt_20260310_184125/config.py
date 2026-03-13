"""Centralized configuration for AmazingRobot."""

from dataclasses import dataclass
import os
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent.parent


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _env_classes(name: str, default: tuple[str, ...]) -> tuple[str, ...]:
    value = os.getenv(name, "")
    if not value.strip():
        return default
    parts = [p.strip() for p in value.split(",")]
    classes = tuple(p for p in parts if p)
    return classes if classes else default


@dataclass(frozen=True)
class SerialConfig:
    drive_port: str = os.getenv("DRIVE_ESP32_PORT", "/dev/ttyUSB0")
    drive_baud: int = int(os.getenv("DRIVE_ESP32_BAUD", "115200"))
    actuator_port: str = os.getenv("ACT_ESP32_PORT", "/dev/ttyUSB1")
    actuator_baud: int = int(os.getenv("ACT_ESP32_BAUD", "115200"))


@dataclass(frozen=True)
class CameraConfig:
    front_index: int = int(os.getenv("FRONT_CAMERA_INDEX", "0"))
    rear_index: int = int(os.getenv("REAR_CAMERA_INDEX", "1"))
    swap: bool = _env_bool("CAMERA_SWAP", False)
    width: int = int(os.getenv("CAMERA_WIDTH", "640"))
    height: int = int(os.getenv("CAMERA_HEIGHT", "480"))


@dataclass(frozen=True)
class MotionConfig:
    wheel_diameter_inch: float = float(os.getenv("WHEEL_DIAMETER_INCH", "5.0"))
    counts_per_rev: int = int(os.getenv("ENC_COUNTS_PER_REV", "1600"))

    # Existing known geometry
    front_to_plant_point_cm: float = float(os.getenv("FRONT_TO_PLANT_POINT_CM", "23.5"))
    plant_to_camback_point_cm: float = float(
        os.getenv("PLANT_TO_CAMBACK_POINT_CM", os.getenv("plant_to_CamBack_point_cm", "0.0"))
    )

    # User said may provide later; kept configurable
    robot_length_cm: float = float(os.getenv("ROBOT_LENGTH_CM", "0.0"))
    front_to_plant_axis_cm: float = float(os.getenv("FRONT_TO_PLANT_AXIS_CM", "0.0"))


@dataclass(frozen=True)
class VisionConfig:
    model_path: str = os.getenv("YOLO_MODEL_PATH", str(ROOT_DIR / "best1.pt"))
    target_classes: tuple[str, ...] = _env_classes(
        "TARGET_CLASSES", ("FaceWoodenbox", "woodenbox")
    )
    priority_class: str = os.getenv("PRIORITY_CLASS", "FaceWoodenbox")
    confidence_threshold: float = float(os.getenv("CONF_THRESHOLD", "0.5"))
    yolo_max_det: int = int(os.getenv("YOLO_MAX_DET", "20"))

    # Horizontal line in pixel for bbox-bottom trigger
    bbox_trigger_line_y: int = int(os.getenv("BBOX_TRIGGER_LINE_Y", "360"))

    # PID steering
    kp: float = float(os.getenv("PID_KP", "0.8"))
    ki: float = float(os.getenv("PID_KI", "0.0"))
    kd: float = float(os.getenv("PID_KD", "0.1"))
    integral_max: float = float(os.getenv("PID_INTEGRAL_MAX", "1.0"))
    base_norm: float = float(os.getenv("BASE_NORM", "0.45"))
    max_cmd: int = int(os.getenv("MAX_CMD", "800"))
    approach_min_norm: float = float(os.getenv("APPROACH_MIN_NORM", "0.18"))
    approach_max_norm: float = float(os.getenv("APPROACH_MAX_NORM", "0.45"))
    approach_err_slow_gain: float = float(os.getenv("APPROACH_ERR_SLOW_GAIN", "0.65"))
    approach_near_slow_gain: float = float(os.getenv("APPROACH_NEAR_SLOW_GAIN", "0.45"))

    # Entry alignment stage (before entering row/plot)
    entry_align_lateral_tol_px: int = int(os.getenv("ENTRY_ALIGN_LATERAL_TOL_PX", "28"))
    entry_align_heading_tol_norm: float = float(os.getenv("ENTRY_ALIGN_HEADING_TOL_NORM", "0.08"))
    entry_align_hold_frames: int = int(os.getenv("ENTRY_ALIGN_HOLD_FRAMES", "6"))
    entry_align_turn_gain: float = float(os.getenv("ENTRY_ALIGN_TURN_GAIN", "0.9"))
    entry_align_heading_gain: float = float(os.getenv("ENTRY_ALIGN_HEADING_GAIN", "0.5"))
    entry_align_turn_norm: float = float(os.getenv("ENTRY_ALIGN_TURN_NORM", "0.30"))
    entry_align_forward_norm: float = float(os.getenv("ENTRY_ALIGN_FORWARD_NORM", "0.12"))
    pre_april_bbox_tolerance_px: int = int(os.getenv("PRE_APRIL_BBOX_TOLERANCE_PX", "10"))
    pre_april_adjust_norm: float = float(os.getenv("PRE_APRIL_ADJUST_NORM", "0.18"))
    pre_april_adjust_timeout_sec: float = float(os.getenv("PRE_APRIL_ADJUST_TIMEOUT_SEC", "6.0"))

    # Stuck detection + recovery
    stuck_window_sec: float = float(os.getenv("STUCK_WINDOW_SEC", "1.2"))
    stuck_min_progress_cm: float = float(os.getenv("STUCK_MIN_PROGRESS_CM", "0.6"))
    stuck_max_retries: int = int(os.getenv("STUCK_MAX_RETRIES", "3"))
    stuck_backoff_sec: float = float(os.getenv("STUCK_BACKOFF_SEC", "0.8"))
    stuck_backoff_norm: float = float(os.getenv("STUCK_BACKOFF_NORM", "0.26"))
    stuck_turn_sec: float = float(os.getenv("STUCK_TURN_SEC", "0.7"))
    stuck_turn_norm: float = float(os.getenv("STUCK_TURN_NORM", "0.30"))

    # Rear-camera cabbage measurement
    rear_measure_target_class: str = os.getenv("REAR_MEASURE_TARGET_CLASS", "cabbage")
    rear_measure_frames: int = int(os.getenv("REAR_MEASURE_FRAMES", "8"))
    rear_measure_timeout_sec: float = float(os.getenv("REAR_MEASURE_TIMEOUT_SEC", "3.0"))
    rear_measure_size_threshold_cm: float = float(os.getenv("REAR_MEASURE_SIZE_THRESHOLD_CM", "12.0"))
    move_to_measure_speed_norm: float = float(os.getenv("MOVE_TO_MEASURE_SPEED_NORM", "0.28"))
    rear_measure_max_per_class: int = int(os.getenv("REAR_MEASURE_MAX_PER_CLASS", "5"))
    rear_measure_max_total: int = int(os.getenv("REAR_MEASURE_MAX_TOTAL", "5"))

    # px->cm calibration
    apriltag_size_cm: float = float(os.getenv("APRILTAG_SIZE_CM", "5.0"))
    rear_cm_per_px_default: float = float(os.getenv("REAR_CM_PER_PX_DEFAULT", "0.05"))
    apriltag_fx: float = float(os.getenv("APRILTAG_FX", "0.0"))
    apriltag_fy: float = float(os.getenv("APRILTAG_FY", "0.0"))
    apriltag_cx: float = float(os.getenv("APRILTAG_CX", "0.0"))
    apriltag_cy: float = float(os.getenv("APRILTAG_CY", "0.0"))
    apriltag_hfov_deg: float = float(os.getenv("APRILTAG_HFOV_DEG", "0.0"))
    apriltag_vfov_deg: float = float(os.getenv("APRILTAG_VFOV_DEG", "0.0"))

    # AprilTag coarse align before plant-point move.
    apriltag_coarse_align_enabled: bool = _env_bool("APRILTAG_COARSE_ALIGN_ENABLED", True)
    apriltag_coarse_align_tol_px: int = int(os.getenv("APRILTAG_COARSE_ALIGN_TOL_PX", "24"))
    apriltag_coarse_align_hold_frames: int = int(os.getenv("APRILTAG_COARSE_ALIGN_HOLD_FRAMES", "4"))
    apriltag_coarse_align_turn_gain: float = float(os.getenv("APRILTAG_COARSE_ALIGN_TURN_GAIN", "0.85"))
    apriltag_coarse_align_turn_norm: float = float(os.getenv("APRILTAG_COARSE_ALIGN_TURN_NORM", "0.22"))
    apriltag_coarse_align_forward_norm: float = float(os.getenv("APRILTAG_COARSE_ALIGN_FORWARD_NORM", "0.05"))
    apriltag_coarse_align_timeout_sec: float = float(os.getenv("APRILTAG_COARSE_ALIGN_TIMEOUT_SEC", "2.5"))
    apriltag_triangle_align_enabled: bool = _env_bool("APRILTAG_TRIANGLE_ALIGN_ENABLED", True)
    apriltag_triangle_standoff_cm: float = float(os.getenv("APRILTAG_TRIANGLE_STANDOFF_CM", "20.0"))
    apriltag_triangle_max_move_cm: float = float(os.getenv("APRILTAG_TRIANGLE_MAX_MOVE_CM", "120.0"))
    apriltag_triangle_max_turn_deg: float = float(os.getenv("APRILTAG_TRIANGLE_MAX_TURN_DEG", "140.0"))
    apriltag_triangle_move_norm: float = float(os.getenv("APRILTAG_TRIANGLE_MOVE_NORM", "0.24"))
    apriltag_triangle_turn_timeout_sec: float = float(os.getenv("APRILTAG_TRIANGLE_TURN_TIMEOUT_SEC", "4.0"))
    apriltag_triangle_move_timeout_sec: float = float(os.getenv("APRILTAG_TRIANGLE_MOVE_TIMEOUT_SEC", "8.0"))
    apriltag_triangle_turn2_deg: float = float(
        os.getenv("APRILTAG_TRIANGLE_TURN2_DEG", os.getenv("APRILTAG_TRIANGLE_FINAL_HEADING_OFFSET_DEG", "90.0"))
    )
    apriltag_triangle_min_lateral_cm: float = float(os.getenv("APRILTAG_TRIANGLE_MIN_LATERAL_CM", "3.0"))
    apriltag_triangle_final_heading_offset_deg: float = float(os.getenv("APRILTAG_TRIANGLE_FINAL_HEADING_OFFSET_DEG", "0.0"))
    apriltag_accept_center_tol_norm: float = float(os.getenv("APRILTAG_ACCEPT_CENTER_TOL_NORM", "0.22"))
    apriltag_accept_yaw_deg: float = float(os.getenv("APRILTAG_ACCEPT_YAW_DEG", "22.0"))
    apriltag_face_box_expand_scale: float = float(os.getenv("APRILTAG_FACE_BOX_EXPAND_SCALE", "1.35"))
    apriltag_allow_woodenbox_fallback: bool = _env_bool("APRILTAG_ALLOW_WOODENBOX_FALLBACK", False)
    apriltag_woodenbox_center_tol_norm: float = float(os.getenv("APRILTAG_WOODENBOX_CENTER_TOL_NORM", "0.14"))
    apriltag_woodenbox_yaw_deg: float = float(os.getenv("APRILTAG_WOODENBOX_YAW_DEG", "16.0"))
    apriltag_woodenbox_min_side_px: float = float(os.getenv("APRILTAG_WOODENBOX_MIN_SIDE_PX", "16.0"))
    apriltag_woodenbox_bearing_deg: float = float(os.getenv("APRILTAG_WOODENBOX_BEARING_DEG", "18.0"))
    apriltag_seek_forward_norm: float = float(os.getenv("APRILTAG_SEEK_FORWARD_NORM", "0.12"))


@dataclass(frozen=True)
class ActuatorConfig:
    servo_cam_start_deg: int = int(os.getenv("SERVO_CAM_START_DEG", "180"))
    servo_cam_end_deg: int = int(os.getenv("SERVO_CAM_END_DEG", "100"))
    servo_cam_step_deg: int = int(os.getenv("SERVO_CAM_STEP_DEG", "2"))
    servo_cam_step_interval_sec: float = float(os.getenv("SERVO_CAM_STEP_INTERVAL_SEC", "0.04"))
    servo_cam_fast_px_threshold: int = int(os.getenv("SERVO_CAM_FAST_PX_THRESHOLD", "40"))
    servo_cam_fast_max_multiplier: int = int(os.getenv("SERVO_CAM_FAST_MAX_MULTIPLIER", "5"))

    servo_plant_start_deg: int = int(os.getenv("SERVO_PLANT_START_DEG", "0"))
    servo_plant_after_deg: int = int(os.getenv("SERVO_PLANT_AFTER_DEG", "75"))
    servo_plant_degree: int = int(os.getenv("SERVO_PLANT_DEG", "130"))

    feed_step_deg: int = int(os.getenv("FEED_STEP_DEG", "30"))
    feed_delay_sec: float = float(os.getenv("FEED_DELAY_SEC", "1.0"))
    plant_servo_delay_sec: float = float(os.getenv("PLANT_SERVO_DELAY_SEC", "1.0"))
    step1_after_plant_deg: int = int(os.getenv("STEP1_AFTER_PLANT_DEG", "9000"))
    step1_after_burried_deg: int = int(os.getenv("STEP1_AFTER_BURRIED_DEG", "7000"))
    plant_up_step_timeout_sec: float = float(os.getenv("PLANT_UP_STEP_TIMEOUT_SEC", "20.0"))
    plant_retract_delay_sec: float = float(os.getenv("PLANT_RETRACT_DELAY_SEC", "2.0"))
    plant_limit_timeout_sec: float = float(os.getenv("PLANT_LIMIT_TIMEOUT_SEC", "35.0"))
    plant_limit_grace_sec: float = float(os.getenv("PLANT_LIMIT_GRACE_SEC", "12.0"))


@dataclass(frozen=True)
class WebConfig:
    host: str = os.getenv("WEB_HOST", "0.0.0.0")
    port: int = int(os.getenv("WEB_PORT", "5000"))
    debug: bool = os.getenv("WEB_DEBUG", "0") == "1"


@dataclass(frozen=True)
class ImuConfig:
    enabled: bool = _env_bool("IMU_ENABLED", False)
    type: str = os.getenv("IMU_TYPE", "BNO055").strip().upper()
    heading_offset_deg: float = float(os.getenv("IMU_HEADING_OFFSET_DEG", "0.0"))
    invert_heading: bool = _env_bool("IMU_INVERT_HEADING", False)
    heading_tolerance_deg: float = float(os.getenv("IMU_HEADING_TOLERANCE_DEG", "2.5"))
    turn_gain: float = float(os.getenv("IMU_TURN_GAIN", "0.020"))
    turn_norm_max: float = float(os.getenv("IMU_TURN_NORM_MAX", "0.22"))
    manual_rotate_timeout_sec: float = float(os.getenv("MANUAL_ROTATE_TIMEOUT_SEC", "12.0"))
    manual_rotate_hold_samples: int = int(os.getenv("MANUAL_ROTATE_HOLD_SAMPLES", "1"))
    manual_rotate_gain: float = float(os.getenv("MANUAL_ROTATE_GAIN", "0.040"))
    manual_rotate_norm_max: float = float(os.getenv("MANUAL_ROTATE_NORM_MAX", "0.45"))
    manual_rotate_tolerance_deg: float = float(os.getenv("MANUAL_ROTATE_TOLERANCE_DEG", "8.0"))
    manual_rotate_fine_band_deg: float = float(os.getenv("MANUAL_ROTATE_FINE_BAND_DEG", "0.0"))
    manual_rotate_fine_gain: float = float(os.getenv("MANUAL_ROTATE_FINE_GAIN", "0.020"))
    manual_rotate_fine_norm_max: float = float(os.getenv("MANUAL_ROTATE_FINE_NORM_MAX", "0.16"))


@dataclass(frozen=True)
class RuntimeConfig:
    vision_loop_sec: float = float(os.getenv("VISION_LOOP_SEC", "0.05"))
    camera_loop_sec: float = float(os.getenv("CAMERA_LOOP_SEC", "0.01"))
    status_loop_sec: float = float(os.getenv("STATUS_LOOP_SEC", "0.1"))
    search_timeout_sec: float = float(os.getenv("SEARCH_TIMEOUT_SEC", "120.0"))
    apriltag_timeout_sec: float = float(os.getenv("APRILTAG_TIMEOUT_SEC", "5.0"))
    no_box_stop_frames: int = int(os.getenv("NO_BOX_STOP_FRAMES", "8"))


SERIAL = SerialConfig()
CAMERA = CameraConfig()
MOTION = MotionConfig()
VISION = VisionConfig()
ACTUATOR = ActuatorConfig()
WEB = WebConfig()
IMU = ImuConfig()
RUNTIME = RuntimeConfig()
