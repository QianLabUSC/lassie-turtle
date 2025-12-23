#!/usr/bin/env python3
"""
RealSense D435i: "Viewer-default" preview.

Goal: make the depth look like Intel RealSense Viewer with *default* settings:
- Don't force visual preset / laser power / emitter unless you ask for it.
- Use librealsense rs.colorizer() (the same style Viewer uses) instead of OpenCV
  per-frame normalization.
- No post-processing by default (you can enable it explicitly).

Usage:
  python3 realsense_d435i_viewer_defaults.py
  python3 realsense_d435i_viewer_defaults.py --post
  python3 realsense_d435i_viewer_defaults.py --min-depth 0.10 --max-depth 0.70
"""

import argparse
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs


@dataclass
class StreamCfg:
    w: int
    h: int
    fps: int


# These match common RealSense Viewer defaults for D435 depth.
DEFAULT_DEPTH = StreamCfg(848, 480, 30)
DEFAULT_COLOR = StreamCfg(848, 480, 30)


def _try_set(opt_owner, option, value) -> bool:
    try:
        opt_owner.set_option(option, value)
        return True
    except Exception as e:
        print(f"[WARN] Could not set {option} to {value}: {e}")
        return False


def _make_colorizer(min_m: float, max_m: float, scheme: str, hist_eq: bool) -> rs.colorizer:
    cz = rs.colorizer()

    # Color scheme mapping: Viewer exposes multiple schemes; numbers are SDK-defined.
    scheme_map = {
        # These indices follow librealsense colorizer option "color_scheme".
        # Common mapping (most SDKs):
        # 0=Jet, 1=Classic, 2=WhiteToBlack, 3=BlackToWhite, 4=Bio,
        # 5=Cold, 6=Warm, 7=Quantized, 8=Pattern, 9=Turbo (newer SDKs).
        "jet": 0,
        "classic": 1,
        "white_to_black": 2,
        "black_to_white": 3,
        "bio": 4,
        "cold": 5,
        "warm": 6,
        "quantized": 7,
        "pattern": 8,
        "turbo": 9,
    }
    if scheme in scheme_map:
        # Try requested scheme; fall back to Jet if unsupported on this SDK.
        try:
            _try_set(cz, rs.option.color_scheme, float(scheme_map[scheme]))
        except KeyError:
            print("[WARN] Unknown scheme '%s', falling back to 'jet'." % scheme)
            _try_set(cz, rs.option.color_scheme, 0.0)
        # If the SDK doesn't support the chosen index (e.g., Turbo), _try_set will warn.

    # These options are supported in most recent librealsense builds.
    _try_set(cz, rs.option.min_distance, float(min_m))
    _try_set(cz, rs.option.max_distance, float(max_m))

    # Histogram equalization often makes Viewer look "nicer" (more contrast).
    # If your SDK doesn't support it, the call just fails silently.
    _try_set(cz, rs.option.histogram_equalization_enabled, 1.0 if hist_eq else 0.0)

    return cz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min-depth", type=float, default=0.10, help="Colorizer min distance in meters.")
    ap.add_argument("--max-depth", type=float, default=0.70, help="Colorizer max distance in meters.")
    ap.add_argument("--scheme", type=str, default="jet",
                    choices=["jet", "classic", "white_to_black", "black_to_white", "bio", "cold", "warm", "quantized", "pattern", "turbo"],
                    help="Depth color scheme (Viewer-like).")
    ap.add_argument("--hist-eq", action="store_true", help="Enable histogram equalization in the colorizer.")
    ap.add_argument("--post", action="store_true", help="Enable post-processing filters (spatial/temporal/hole fill).")

    # Only set these if you explicitly ask; Viewer defaults are usually good.
    ap.add_argument("--emitter", type=int, default=None, choices=[0, 1], help="Force emitter_enabled (0/1).")
    ap.add_argument("--laser", type=float, default=None, help="Force laser_power (device units).")
    ap.add_argument("--preset", type=str, default=None,
                    choices=["default", "high_accuracy", "high_density", "medium_density"],
                    help="Force visual_preset on depth sensor.")

    # Stream overrides (use these to exactly match what you see in Viewer if needed).
    ap.add_argument("--depth-w", type=int, default=DEFAULT_DEPTH.w)
    ap.add_argument("--depth-h", type=int, default=DEFAULT_DEPTH.h)
    ap.add_argument("--depth-fps", type=int, default=DEFAULT_DEPTH.fps)
    ap.add_argument("--color-w", type=int, default=DEFAULT_COLOR.w)
    ap.add_argument("--color-h", type=int, default=DEFAULT_COLOR.h)
    ap.add_argument("--color-fps", type=int, default=DEFAULT_COLOR.fps)

    args = ap.parse_args()

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, args.depth_w, args.depth_h, rs.format.z16, args.depth_fps)
    cfg.enable_stream(rs.stream.color, args.color_w, args.color_h, rs.format.bgr8, args.color_fps)

    profile = pipeline.start(cfg)

    # Sensor options (only if requested)
    dev = profile.get_device()
    depth_sensor = dev.first_depth_sensor()
    if args.emitter is not None:
        _try_set(depth_sensor, rs.option.emitter_enabled, float(args.emitter))
    if args.laser is not None:
        _try_set(depth_sensor, rs.option.laser_power, float(args.laser))
    if args.preset is not None:
        preset_map = {
            "default": 0,
            "high_accuracy": 3,
            "high_density": 4,
            "medium_density": 5,
        }
        _try_set(depth_sensor, rs.option.visual_preset, float(preset_map[args.preset]))

    # Post-processing filters (OFF by default, ON if --post)
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    hole = rs.hole_filling_filter()

    # Viewer-like colorizer
    colorizer = _make_colorizer(args.min_depth, args.max_depth, args.scheme, args.hist_eq)

    win = "RealSense D435i (Viewer defaults)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth = frames.get_depth_frame()
            color = frames.get_color_frame()
            if not depth or not color:
                continue

            if args.post:
                # These are SDK default parameters; Viewer commonly uses similar ones.
                depth = spatial.process(depth)
                depth = temporal.process(depth)
                depth = hole.process(depth)

            depth_color = colorizer.colorize(depth)
            depth_img = np.asanyarray(depth_color.get_data())  # RGB
            depth_bgr = cv2.cvtColor(depth_img, cv2.COLOR_RGB2BGR)

            color_img = np.asanyarray(color.get_data())

            # Side-by-side
            h = min(color_img.shape[0], depth_bgr.shape[0])
            color_vis = color_img[:h, :, :]
            depth_vis = depth_bgr[:h, :, :]

            canvas = np.hstack([color_vis, depth_vis])

            # Simple overlay
            cv2.putText(canvas, f"Depth ({args.scheme.upper()}) {args.min_depth:.2f}–{args.max_depth:.2f} m",
                        (color_vis.shape[1] + 20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow(win, canvas)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord('q')):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
