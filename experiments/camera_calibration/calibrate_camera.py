#!/usr/bin/env python3
"""Chessboard-based camera calibration utility for AmazingRobot."""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np


def build_object_points(cols: int, rows: int, square_size_mm: float) -> np.ndarray:
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_size_mm)
    return objp


def detect_corners(
    image: np.ndarray,
    board_size: tuple[int, int],
    criteria: tuple[int, int, float],
) -> tuple[bool, np.ndarray | None]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray,
        board_size,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK,
    )
    if not found or corners is None:
        return False, None

    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, refined


def calibrate_from_images(
    image_paths: list[Path],
    cols: int,
    rows: int,
    square_size_mm: float,
    debug_detect: bool = False,
) -> dict:
    board_size = (cols, rows)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = build_object_points(cols, rows, square_size_mm)

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    used_images: list[str] = []

    for path in image_paths:
        image = cv2.imread(str(path))
        if image is None:
            if debug_detect:
                print(f"skip unreadable: {path}")
            continue
        found, corners = detect_corners(image, board_size, criteria)
        if not found or corners is None:
            if debug_detect:
                print(f"no pattern: {path}")
            continue

        if debug_detect:
            print(f"pattern OK: {path}")
        image_size = (image.shape[1], image.shape[0])
        object_points.append(objp.copy())
        image_points.append(corners)
        used_images.append(str(path))

    if len(image_points) < 6 or image_size is None:
        raise RuntimeError(
            f"Need at least 6 valid chessboard images, found {len(image_points)}"
        )

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    per_view_errors: list[float] = []
    total_error = 0.0
    for i in range(len(object_points)):
        projected, _ = cv2.projectPoints(
            object_points[i],
            rvecs[i],
            tvecs[i],
            camera_matrix,
            dist_coeffs,
        )
        err = cv2.norm(image_points[i], projected, cv2.NORM_L2) / len(projected)
        per_view_errors.append(float(err))
        total_error += err

    mean_error = total_error / len(object_points)

    return {
        "rms_reprojection_error": float(ret),
        "mean_reprojection_error_px": float(mean_error),
        "image_size": {"width": int(image_size[0]), "height": int(image_size[1])},
        "board": {
            "inner_corners_cols": int(cols),
            "inner_corners_rows": int(rows),
            "square_size_mm": float(square_size_mm),
        },
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.flatten().tolist(),
        "apriltag_env": {
            "APRILTAG_FX": float(camera_matrix[0, 0]),
            "APRILTAG_FY": float(camera_matrix[1, 1]),
            "APRILTAG_CX": float(camera_matrix[0, 2]),
            "APRILTAG_CY": float(camera_matrix[1, 2]),
        },
        "used_images": used_images,
        "per_view_error_px": per_view_errors,
    }


def build_capture(args: argparse.Namespace) -> cv2.VideoCapture:
    source: str | int = args.device if args.device else args.camera_index
    backend = cv2.CAP_ANY
    if args.backend == "v4l2":
        backend = cv2.CAP_V4L2
    elif args.backend == "ffmpeg":
        backend = cv2.CAP_FFMPEG

    cap = cv2.VideoCapture(source, backend)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera source: {source}")
    if args.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc[:4]))
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def capture_single_frame_ffmpeg(args: argparse.Namespace, output_path: Path) -> bool:
    source = args.device if args.device else f"/dev/video{args.camera_index}"
    ffmpeg_input_format = ""
    if args.fourcc:
        ffmpeg_input_format = args.fourcc.lower()
        if ffmpeg_input_format == "mjpg":
            ffmpeg_input_format = "mjpeg"
    cmd = [
        "ffmpeg",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "v4l2",
    ]
    if ffmpeg_input_format:
        cmd.extend(["-input_format", ffmpeg_input_format])
    if args.width > 0 and args.height > 0:
        cmd.extend(["-video_size", f"{args.width}x{args.height}"])
    cmd.extend(
        [
            "-i",
            source,
            "-frames:v",
            "1",
            "-update",
            "1",
            str(output_path),
        ]
    )
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        stderr = result.stderr.strip()
        if stderr:
            print(f"ffmpeg capture failed: {stderr}")
        return False
    return output_path.exists() and output_path.stat().st_size > 0


def capture_images_gui(args: argparse.Namespace) -> list[Path]:
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = build_capture(args)

    board_size = (args.cols, args.rows)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    saved: list[Path] = []

    print("Press SPACE to save a valid chessboard frame, C to calibrate, Q to quit.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera read failed")
                break

            view = frame.copy()
            found, corners = detect_corners(frame, board_size, criteria)
            if found and corners is not None:
                cv2.drawChessboardCorners(view, board_size, corners, found)
                color = (0, 255, 0)
                msg = f"pattern OK | saved={len(saved)}"
            else:
                color = (0, 0, 255)
                msg = f"pattern not found | saved={len(saved)}"

            cv2.putText(view, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.imshow("Camera Calibration", view)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            if key == ord(" ") and found:
                filename = out_dir / f"calib_{len(saved)+1:02d}.jpg"
                cv2.imwrite(str(filename), frame)
                saved.append(filename)
                print(f"saved {filename}")
            if key == ord("c") and len(saved) >= 6:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    return saved


def capture_images_headless(args: argparse.Namespace) -> list[Path]:
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = build_capture(args)
    board_size = (args.cols, args.rows)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    saved: list[Path] = []
    last_save_ts = 0.0
    deadline = time.monotonic() + max(5.0, float(args.capture_timeout_sec))
    frames_seen = 0
    last_status_ts = 0.0

    print(
        "Headless capture started: "
        f"target={args.capture_count} interval={args.capture_interval_sec:.1f}s timeout={args.capture_timeout_sec:.1f}s"
    )
    try:
        while len(saved) < args.capture_count and time.monotonic() < deadline:
            ok, frame = cap.read()
            if not ok:
                print("Camera read failed")
                break

            frames_seen += 1
            found, _ = detect_corners(frame, board_size, criteria)
            now = time.monotonic()
            if now - last_status_ts >= 2.0:
                print(
                    f"status: frames={frames_seen} saved={len(saved)} chessboard={'yes' if found else 'no'}"
                )
                last_status_ts = now
            if not found:
                time.sleep(0.05)
                continue

            if now - last_save_ts < float(args.capture_interval_sec):
                time.sleep(0.05)
                continue

            filename = out_dir / f"calib_{len(saved)+1:02d}.jpg"
            cv2.imwrite(str(filename), frame)
            saved.append(filename)
            last_save_ts = now
            print(f"saved {filename} ({len(saved)}/{args.capture_count})")
            time.sleep(0.15)
    finally:
        cap.release()

    return saved


def capture_images_headless_ffmpeg(args: argparse.Namespace) -> list[Path]:
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    board_size = (args.cols, args.rows)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    saved: list[Path] = []
    last_save_ts = 0.0
    deadline = time.monotonic() + max(5.0, float(args.capture_timeout_sec))
    probe_path = out_dir / "_ffmpeg_probe.jpg"
    frames_seen = 0
    last_status_ts = 0.0

    print(
        "Headless ffmpeg capture started: "
        f"target={args.capture_count} interval={args.capture_interval_sec:.1f}s timeout={args.capture_timeout_sec:.1f}s"
    )
    try:
        while len(saved) < args.capture_count and time.monotonic() < deadline:
            if not capture_single_frame_ffmpeg(args, probe_path):
                time.sleep(0.2)
                continue

            frame = cv2.imread(str(probe_path))
            if frame is None:
                time.sleep(0.1)
                continue

            frames_seen += 1
            found, _ = detect_corners(frame, board_size, criteria)
            now = time.monotonic()
            if now - last_status_ts >= 2.0:
                print(
                    f"status: frames={frames_seen} saved={len(saved)} chessboard={'yes' if found else 'no'}"
                )
                last_status_ts = now
            if not found:
                time.sleep(0.05)
                continue

            if now - last_save_ts < float(args.capture_interval_sec):
                time.sleep(0.05)
                continue

            filename = out_dir / f"calib_{len(saved)+1:02d}.jpg"
            probe_path.replace(filename)
            saved.append(filename)
            last_save_ts = now
            print(f"saved {filename} ({len(saved)}/{args.capture_count})")
            time.sleep(0.15)
    finally:
        if probe_path.exists():
            probe_path.unlink()

    return saved


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate camera intrinsics with a chessboard.")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index for live capture")
    parser.add_argument("--device", default="", help="Optional explicit video device path, e.g. /dev/video0")
    parser.add_argument("--width", type=int, default=640, help="Capture width")
    parser.add_argument("--height", type=int, default=480, help="Capture height")
    parser.add_argument("--backend", choices=("any", "v4l2", "ffmpeg"), default="any", help="OpenCV video backend")
    parser.add_argument("--fourcc", default="", help="Optional FOURCC such as MJPG")
    parser.add_argument("--cols", type=int, default=9, help="Chessboard inner corners horizontally")
    parser.add_argument("--rows", type=int, default=6, help="Chessboard inner corners vertically")
    parser.add_argument("--square-size-mm", type=float, default=25.0, help="One chessboard square size in mm")
    parser.add_argument("--images-glob", default="", help="Existing calibration images glob, e.g. 'images/*.jpg'")
    parser.add_argument("--output-dir", default="experiments/camera_calibration/captures", help="Where to store captured frames")
    parser.add_argument("--result-json", default="experiments/camera_calibration/calibration_result.json", help="Where to store calibration result")
    parser.add_argument("--debug-detect", action="store_true", help="Print whether each input image contains a valid chessboard")
    parser.add_argument("--headless", action="store_true", help="Capture without GUI window (for SSH/headless Pi)")
    parser.add_argument(
        "--headless-method",
        choices=("auto", "opencv", "ffmpeg"),
        default="auto",
        help="Headless capture backend",
    )
    parser.add_argument("--capture-count", type=int, default=12, help="Headless target number of valid images")
    parser.add_argument("--capture-interval-sec", type=float, default=1.0, help="Minimum seconds between saved headless frames")
    parser.add_argument("--capture-timeout-sec", type=float, default=90.0, help="Headless capture timeout")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.images_glob:
        image_paths = [Path(p) for p in sorted(glob.glob(args.images_glob))]
    else:
        if args.headless:
            if args.headless_method == "ffmpeg":
                image_paths = capture_images_headless_ffmpeg(args)
            elif args.headless_method == "opencv":
                image_paths = capture_images_headless(args)
            else:
                image_paths = capture_images_headless(args)
                if not image_paths:
                    print("OpenCV headless capture failed; retrying with ffmpeg.")
                    image_paths = capture_images_headless_ffmpeg(args)
        else:
            image_paths = capture_images_gui(args)

    if not image_paths:
        print("No calibration images available")
        return 1

    result = calibrate_from_images(
        image_paths=image_paths,
        cols=args.cols,
        rows=args.rows,
        square_size_mm=args.square_size_mm,
        debug_detect=args.debug_detect,
    )

    result_path = Path(args.result_json)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    env = result["apriltag_env"]
    print(f"saved result -> {result_path}")
    print(f"RMS reprojection error: {result['rms_reprojection_error']:.4f}")
    print(f"Mean reprojection error: {result['mean_reprojection_error_px']:.4f} px")
    print("Put these in .env:")
    print(f"APRILTAG_FX={env['APRILTAG_FX']:.6f}")
    print(f"APRILTAG_FY={env['APRILTAG_FY']:.6f}")
    print(f"APRILTAG_CX={env['APRILTAG_CX']:.6f}")
    print(f"APRILTAG_CY={env['APRILTAG_CY']:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
