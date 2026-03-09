#!/usr/bin/env python3
"""Standalone AprilTag probe for distance and angle experiments."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
except Exception as exc:  # pragma: no cover - runtime environment specific
    raise SystemExit(
        "pupil-apriltags is required. Install project requirements first."
    ) from exc


C_TO_SPACING_MAP = {
    1: 5,
    2: 10,
    3: 15,
    4: 20,
    5: 25,
}


@dataclass
class TagInfo:
    tag_id: int
    ab_cm: int
    c_gap_cm: int
    de_cm: int


def decode_tag_id(tag_id: int) -> TagInfo:
    tag_str = f"{tag_id:05d}"
    ab = int(tag_str[:2])
    c = int(tag_str[2])
    de = int(tag_str[3:])
    return TagInfo(tag_id=tag_id, ab_cm=ab, c_gap_cm=C_TO_SPACING_MAP.get(c, 0), de_cm=de)


def edge_mean_px(corners: np.ndarray) -> float:
    c = corners.astype(np.float32)
    return float(
        np.mean(
            [
                np.linalg.norm(c[0] - c[1]),
                np.linalg.norm(c[1] - c[2]),
                np.linalg.norm(c[2] - c[3]),
                np.linalg.norm(c[3] - c[0]),
            ]
        )
    )


def compute_bearing_deg(
    center_xy: tuple[float, float],
    frame_shape: tuple[int, int, int],
    args: argparse.Namespace,
) -> tuple[float | None, float | None]:
    x, y = center_xy
    h, w = frame_shape[:2]

    if args.fx > 0 and args.cx > 0:
        bearing_x = math.degrees(math.atan2(x - args.cx, args.fx))
    elif args.hfov_deg > 0 and w > 0:
        bearing_x = ((x - (w / 2.0)) / (w / 2.0)) * (args.hfov_deg / 2.0)
    else:
        bearing_x = None

    if args.fy > 0 and args.cy > 0:
        bearing_y = math.degrees(math.atan2(y - args.cy, args.fy))
    elif args.vfov_deg > 0 and h > 0:
        bearing_y = ((y - (h / 2.0)) / (h / 2.0)) * (args.vfov_deg / 2.0)
    else:
        bearing_y = None

    return bearing_x, bearing_y


def rotation_to_euler_deg(rot: np.ndarray) -> tuple[float, float, float]:
    """Return roll, pitch, yaw in degrees from a 3x3 rotation matrix."""
    sy = math.sqrt(rot[0, 0] * rot[0, 0] + rot[1, 0] * rot[1, 0])
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll = math.atan2(-rot[1, 2], rot[1, 1])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = 0.0

    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def build_capture(args: argparse.Namespace) -> cv2.VideoCapture:
    source = args.device if args.device else args.camera_index
    backend = cv2.CAP_ANY
    if args.backend == "v4l2":
        backend = cv2.CAP_V4L2
    elif args.backend == "ffmpeg":
        backend = cv2.CAP_FFMPEG
    cap = cv2.VideoCapture(source, backend)
    if args.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc[:4]))
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.fps > 0:
        cap.set(cv2.CAP_PROP_FPS, args.fps)
    return cap


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read AprilTag 52h13 distance and pose.")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--device", default="", help="Optional explicit camera device path")
    parser.add_argument("--width", type=int, default=640, help="Capture width")
    parser.add_argument("--height", type=int, default=480, help="Capture height")
    parser.add_argument("--fps", type=float, default=0.0, help="Requested FPS")
    parser.add_argument("--tag-size-cm", type=float, default=4.2, help="Real tag size in cm")
    parser.add_argument("--fx", type=float, default=0.0, help="Camera fx")
    parser.add_argument("--fy", type=float, default=0.0, help="Camera fy")
    parser.add_argument("--cx", type=float, default=0.0, help="Camera cx")
    parser.add_argument("--cy", type=float, default=0.0, help="Camera cy")
    parser.add_argument("--hfov-deg", type=float, default=0.0, help="Optional horizontal FOV in degrees")
    parser.add_argument("--vfov-deg", type=float, default=0.0, help="Optional vertical FOV in degrees")
    parser.add_argument("--family", default="tagStandard52h13", help="AprilTag family")
    parser.add_argument("--backend", choices=("any", "v4l2", "ffmpeg"), default="any", help="OpenCV backend")
    parser.add_argument("--fourcc", default="", help="Requested FOURCC such as MJPG")
    parser.add_argument("--image", default="", help="Analyze one image file instead of a live camera")
    parser.add_argument("--video", default="", help="Analyze a video file instead of a live camera")
    return parser.parse_args()


def annotate_frame(
    frame: np.ndarray,
    detector: Detector,
    args: argparse.Namespace,
    estimate_pose: bool,
    camera_params: tuple[float, float, float, float] | None,
) -> np.ndarray:
    annotated = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(
        gray,
        estimate_tag_pose=estimate_pose,
        camera_params=camera_params,
        tag_size=args.tag_size_cm / 100.0 if estimate_pose else None,
    )

    lines = [
        "AprilTag Probe",
        f"Pose: {'ON' if estimate_pose else 'OFF'}",
        f"Source: {args.image or args.video or args.device or args.camera_index}",
    ]

    for idx, tag in enumerate(tags):
        info = decode_tag_id(int(tag.tag_id))
        corners = tag.corners.astype(int)
        side_px = edge_mean_px(tag.corners)

        for i in range(4):
            p1 = tuple(corners[i])
            p2 = tuple(corners[(i + 1) % 4])
            cv2.line(annotated, p1, p2, (0, 255, 0), 2)

        center = tuple(tag.center.astype(int))
        cv2.circle(annotated, center, 4, (0, 0, 255), -1)

        lines.append(
            f"id={info.tag_id:05d} AB={info.ab_cm} C={info.c_gap_cm} DE={info.de_cm} side={side_px:.1f}px"
        )

        if estimate_pose and getattr(tag, "pose_t", None) is not None and getattr(tag, "pose_R", None) is not None:
            t = np.array(tag.pose_t).reshape(-1)
            distance_cm = float(np.linalg.norm(t)) * 100.0
            z_cm = float(t[2]) * 100.0
            roll, pitch, yaw = rotation_to_euler_deg(np.array(tag.pose_R))
            lines.append(
                f"dist={distance_cm:.1f}cm z={z_cm:.1f}cm roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f}"
            )
        elif args.fx > 0 and args.tag_size_cm > 0 and side_px > 1.0:
            approx_cm = float(args.tag_size_cm * args.fx / side_px)
            lines.append(f"approx_dist={approx_cm:.1f}cm (pinhole)")

        bearing_x, bearing_y = compute_bearing_deg(
            (float(tag.center[0]), float(tag.center[1])),
            frame.shape,
            args,
        )
        if bearing_x is not None or bearing_y is not None:
            bx = "na" if bearing_x is None else f"{bearing_x:.1f}"
            by = "na" if bearing_y is None else f"{bearing_y:.1f}"
            lines.append(f"cam_angle_x={bx}deg cam_angle_y={by}deg")

        cv2.putText(
            annotated,
            f"{idx + 1}:{info.tag_id:05d}",
            (corners[0][0], max(20, corners[0][1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

    if not tags:
        lines.append("no tag")

    for i, line in enumerate(lines):
        cv2.putText(
            annotated,
            line,
            (10, 25 + (i * 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (50, 220, 50) if i < 3 else (255, 255, 255),
            2,
        )
    return annotated


def main() -> int:
    args = parse_args()
    detector = Detector(
        families=args.family,
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

    camera_params = None
    estimate_pose = all(v > 0 for v in (args.fx, args.fy, args.cx, args.cy)) and args.tag_size_cm > 0
    if estimate_pose:
        camera_params = (args.fx, args.fy, args.cx, args.cy)

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"Failed to load image: {args.image}")
            return 1
        annotated = annotate_frame(frame, detector, args, estimate_pose, camera_params)
        cv2.imshow("AprilTag Probe", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return 0

    cap = cv2.VideoCapture(args.video) if args.video else build_capture(args)
    if not cap.isOpened():
        print(f"Failed to open source: {args.image or args.video or args.device or args.camera_index}")
        return 1

    print("Press q to quit.")
    print(f"Pose enabled: {'yes' if estimate_pose else 'no'}")
    if args.backend != "any":
        print(f"Backend: {args.backend}")
    if args.fourcc:
        print(f"FOURCC: {args.fourcc}")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Source read failed")
            break

        annotated = annotate_frame(frame, detector, args, estimate_pose, camera_params)
        cv2.imshow("AprilTag Probe", annotated)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
