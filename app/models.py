"""Shared dataclasses and enums for AmazingRobot."""

from dataclasses import dataclass, asdict
from enum import Enum
from typing import Optional


class MissionState(str, Enum):
    IDLE = "IDLE"
    INITIALIZING = "INITIALIZING"
    SEARCHING_TARGET = "SEARCHING_TARGET"
    APPROACHING_TARGET = "APPROACHING_TARGET"
    ALIGNING_ENTRY = "ALIGNING_ENTRY"
    READING_APRILTAG = "READING_APRILTAG"
    MOVING_TO_PLANT_POINT = "MOVING_TO_PLANT_POINT"
    PLANTING = "PLANTING"
    MOVING_TO_MEASURE_POINT = "MOVING_TO_MEASURE_POINT"
    MEASURING_REAR_SIZE = "MEASURING_REAR_SIZE"
    FINISHED = "FINISHED"
    STOPPED = "STOPPED"
    ERROR = "ERROR"


@dataclass
class Detection:
    class_name: str
    conf: float
    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def bottom(self) -> int:
        return self.y2


@dataclass
class AprilTagInfo:
    tag_id: int
    planting_distance_cm: int  # AB
    spacing_gap_cm: int        # C mapped
    cabbage_interval_cm: int   # DE


@dataclass
class DriveTelemetry:
    left_rpm: float = 0.0
    right_rpm: float = 0.0
    distance_cm: float = 0.0
    left_count: int = 0
    right_count: int = 0
    imu_connected: bool = False
    heading_deg: Optional[float] = None
    imu_cal_sys: int = 0
    imu_cal_gyro: int = 0
    imu_cal_accel: int = 0
    imu_cal_mag: int = 0
    raw_line: str = ""


@dataclass
class MissionSnapshot:
    state: str
    running: bool
    status_text: str
    target_class: str
    servo_cam_deg: int
    april: Optional[dict]
    drive: dict
    imu: Optional[dict] = None
    pid: Optional[dict] = None
    rear_measurement: Optional[dict] = None
    plant_map: Optional[list[dict]] = None

    def to_dict(self) -> dict:
        return asdict(self)
