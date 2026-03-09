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
    front_to_plant_point_cm: float = float(os.getenv("FRONT_TO_PLANT_POINT_CM", "25.0"))

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


@dataclass(frozen=True)
class ActuatorConfig:
    servo_cam_start_deg: int = int(os.getenv("SERVO_CAM_START_DEG", "150"))
    servo_cam_end_deg: int = int(os.getenv("SERVO_CAM_END_DEG", "72"))
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
    step1_after_burried_deg: int = int(os.getenv("STEP1_AFTER_BURRIED_DEG", "4500"))
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
RUNTIME = RuntimeConfig()
