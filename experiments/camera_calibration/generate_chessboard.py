#!/usr/bin/env python3
"""Generate a printable chessboard calibration target."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a printable chessboard PNG.")
    parser.add_argument("--inner-cols", type=int, default=9, help="Number of inner corners horizontally")
    parser.add_argument("--inner-rows", type=int, default=6, help="Number of inner corners vertically")
    parser.add_argument("--square-px", type=int, default=120, help="Square size in pixels")
    parser.add_argument("--margin-px", type=int, default=80, help="White margin around the board")
    parser.add_argument(
        "--output",
        default="experiments/camera_calibration/chessboard_9x6.png",
        help="Output PNG path",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    squares_x = args.inner_cols + 1
    squares_y = args.inner_rows + 1
    board_w = squares_x * args.square_px
    board_h = squares_y * args.square_px
    width = board_w + 2 * args.margin_px
    height = board_h + 2 * args.margin_px

    image = np.full((height, width), 255, dtype=np.uint8)
    for y in range(squares_y):
        for x in range(squares_x):
            if (x + y) % 2 == 0:
                x0 = args.margin_px + x * args.square_px
                y0 = args.margin_px + y * args.square_px
                image[y0 : y0 + args.square_px, x0 : x0 + args.square_px] = 0

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), image)
    print(f"saved {out_path}")
    print(
        f"board: inner_corners={args.inner_cols}x{args.inner_rows} "
        f"squares={squares_x}x{squares_y} square_px={args.square_px}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
