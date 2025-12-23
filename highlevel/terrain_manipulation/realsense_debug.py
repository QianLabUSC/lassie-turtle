#!/usr/bin/env python3
"""
RealSense D435i depth preview with *viewer-like* coloring + debugging.

This script is built to answer two practical questions:
1) Are "black" pixels at close range coming from *invalid depth* (depth==0),
   or from visualization range settings?
2) Is your visualization range actually being applied (so small 2–3 cm changes show up)?

Key features:
- Two visualization modes:
  - rs_colorizer: uses librealsense rs.colorizer() (closest to Viewer)
  - manual: fixed-range linear mapping (always respects min/max; no per-frame normalize)
- Optional histogram equalization (rs_colorizer).
- Optional sensor option toggles (emitter, laser power, visual preset).
- On-screen debug: center depth (m) and % invalid pixels.

Usage examples:
  # Closest to Viewer-ish coloring (but with debug overlay)
  python3 realsense_depth_debug_viewerlike.py --viz rs --scheme jet --min-depth 0.10 --max-depth 0.70

  # If your SDK ignores colorizer min/max, switch to manual (guaranteed sensitivity)
  python3 realsense_depth_debug_viewerlike.py --viz manual --scheme jet --min-depth 0.10 --max-depth 0.70

  # If you get black at close range due to invalid depth, try enabling emitter/laser:
  python3 realsense_depth_debug_viewerlike.py --enable-emitter --laser-power max

Notes:
- D435i stereo depth has a physical *minimum usable range* that depends on resolution/preset.
  If center depth becomes 0.000 m when you move close, that's not a coloring issue; it's invalid depth.
"""

import argparse
from dataclasses import dataclass
from typing import Optional, Dict

import cv2
import numpy as np
import pyrealsense2 as rs


@dataclass
class StreamCfg:
    w: int
    h: int
    fps: int


DEFAULT_DEPTH = StreamCfg(848, 480, 30)
DEFAULT_COLOR = StreamCfg(848, 480, 30)


def _safe_set(opt_owner, opt: rs.option, value: float, name: str) -> bool:
    """Try set an rs.option; return True on success, False on failure."""
    try:
        if hasattr(opt_owner, "supports") and not opt_owner.supports(opt):
            return False
        opt_owner.set_option(opt, float(value))
        return True
    except Exception as e:
        print(f"[WARN] Failed to set {name} ({opt}) = {value}: {e}")
        return False


def _safe_get(opt_owner, opt: rs.option, name: str) -> Optional[float]:
    try:
        if hasattr(opt_owner, "supports") and not opt_owner.supports(opt):
            return None
        return float(opt_owner.get_option(opt))
    except Exception:
        return None


def _make_colorizer(scheme: str, hist_eq: bool) -> rs.colorizer:
    cz = rs.colorizer()

    # Librealsense colorizer "color_scheme" values (common mapping):
    # 0: Jet, 1: Classic, 2: WhiteToBlack, 3: BlackToWhite, 4: Bio, 5: Cold, 6: Warm, 7: Quantized, 8: Pattern
    # Some builds also support Turbo as 9 (not guaranteed).
    scheme_map: Dict[str, float] = {
        "jet": 0.0,
        "classic": 1.0,
        "white2black": 2.0,
        "black2white": 3.0,
        "bio": 4.0,
        "cold": 5.0,
        "warm": 6.0,
        "quantized": 7.0,
        "pattern": 8.0,
        "turbo": 9.0,  # may be unsupported on some SDKs
    }

    if scheme not in scheme_map:
        print(f"[WARN] Unknown scheme '{scheme}', falling back to 'jet'.")
        scheme = "jet"

    ok = _safe_set(cz, rs.option.color_scheme, scheme_map[scheme], "color_scheme")
    if not ok:
        print("[WARN] This SDK may not support setting color_scheme; colorizer will use its default scheme.")

    if hist_eq:
        ok2 = _safe_set(cz, rs.option.histogram_equalization_enabled, 1.0, "histogram_equalization_enabled")
        if not ok2:
            print("[WARN] histogram_equalization_enabled not supported by this SDK.")

    return cz


def _manual_colorize(depth_z16: np.ndarray, depth_scale: float, min_m: float, max_m: float, scheme: str) -> np.ndarray:
    """Stable, fixed-range colorization that always respects min/max and never per-frame normalizes."""
    depth_m = depth_z16.astype(np.float32) * float(depth_scale)
    invalid = depth_z16 == 0

    # Clamp then map to [0,255] linearly
    depth_clipped = np.clip(depth_m, min_m, max_m)
    denom = max(max_m - min_m, 1e-6)
    norm = (depth_clipped - min_m) / denom  # 0..1

    # We want "deeper/farther" -> warmer (red) to match what you described (holes deeper => red).
    # OpenCV JET maps low->blue high->red, so keep it non-inverted here.
    u8 = (norm * 255.0).astype(np.uint8)

    cmap_map = {
        "jet": cv2.COLORMAP_JET,
        "turbo": getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET),
        "plasma": getattr(cv2, "COLORMAP_PLASMA", cv2.COLORMAP_JET),
        "inferno": getattr(cv2, "COLORMAP_INFERNO", cv2.COLORMAP_JET),
        "magma": getattr(cv2, "COLORMAP_MAGMA", cv2.COLORMAP_JET),
        "viridis": getattr(cv2, "COLORMAP_VIRIDIS", cv2.COLORMAP_JET),
    }
    cm = cmap_map.get(scheme, cv2.COLORMAP_JET)
    colored = cv2.applyColorMap(u8, cm)

    # Invalid depth -> black
    colored[invalid] = 0
    return colored


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--viz", choices=["rs", "manual"], default="rs",
                    help="Depth visualization method: rs=rs.colorizer (Viewer-like), manual=fixed linear mapping.")
    ap.add_argument("--scheme", type=str, default="jet",
                    help="Scheme for colorization (rs mode supports viewer schemes; manual supports OpenCV maps).")
    ap.add_argument("--hist-eq", action="store_true",
                    help="Enable histogram equalization in rs.colorizer (if supported).")
    ap.add_argument("--min-depth", type=float, default=0.10, help="Min distance (m) for visualization range.")
    ap.add_argument("--max-depth", type=float, default=0.70, help="Max distance (m) for visualization range.")

    ap.add_argument("--post", action="store_true", help="Enable a light post-processing chain (spatial+temporal+hole).")

    ap.add_argument("--preset", choices=["none", "default", "high_accuracy", "high_density", "medium_density"],
                    default="none", help="Optionally set D400 visual preset.")
    ap.add_argument("--enable-emitter", action="store_true", help="Enable IR emitter (often helps indoors).")
    ap.add_argument("--laser-power", type=str, default="",
                    help="Laser power (number) or 'max'. Only applied if supported and --enable-emitter is set.")

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

    # Grab depth scale + sensor
    dev = profile.get_device()
    depth_sensor = None
    for s in dev.sensors:
        # Depth sensor is typically the one that supports depth scale
        try:
            _ = s.get_depth_scale()
            depth_sensor = s
            break
        except Exception:
            continue
    if depth_sensor is None:
        # fallback: first sensor
        depth_sensor = dev.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"[INFO] depth_scale = {depth_scale} m/unit")

    # Optional sensor tuning (only if requested)
    if args.preset != "none":
        preset_map = {
            "default": float(rs.rs400_visual_preset.default),
            "high_accuracy": float(rs.rs400_visual_preset.high_accuracy),
            "high_density": float(rs.rs400_visual_preset.high_density),
            "medium_density": float(rs.rs400_visual_preset.medium_density),
        }
        if depth_sensor.supports(rs.option.visual_preset):
            _safe_set(depth_sensor, rs.option.visual_preset, preset_map[args.preset], "visual_preset")
            print(f"[INFO] Set visual preset: {args.preset}")
        else:
            print("[WARN] visual_preset not supported on this device/SDK.")

    if args.enable_emitter and depth_sensor.supports(rs.option.emitter_enabled):
        _safe_set(depth_sensor, rs.option.emitter_enabled, 1.0, "emitter_enabled")
        if args.laser_power:
            if args.laser_power.lower() == "max" and depth_sensor.supports(rs.option.laser_power):
                r = depth_sensor.get_option_range(rs.option.laser_power)
                _safe_set(depth_sensor, rs.option.laser_power, r.max, "laser_power")
                print(f"[INFO] Set laser power to max ({r.max})")
            else:
                try:
                    v = float(args.laser_power)
                    _safe_set(depth_sensor, rs.option.laser_power, v, "laser_power")
                    print(f"[INFO] Set laser power to {v}")
                except ValueError:
                    print("[WARN] --laser-power must be a number or 'max'.")

    # Colorizer for rs mode
    colorizer = _make_colorizer(args.scheme, args.hist_eq)

    # Optional post-processing (kept light)
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    hole = rs.hole_filling_filter()

    align = rs.align(rs.stream.color)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            depth = frames.get_depth_frame()
            color = frames.get_color_frame()
            if not depth or not color:
                continue

            if args.post:
                depth = spatial.process(depth)
                depth = temporal.process(depth)
                depth = hole.process(depth)

            color_img = np.asanyarray(color.get_data())
            depth_z16 = np.asanyarray(depth.get_data())

            # Debug numbers
            h, w = depth_z16.shape
            cx, cy = w // 2, h // 2
            center_m = float(depth.get_distance(cx, cy))
            invalid_ratio = float(np.mean(depth_z16 == 0)) * 100.0

            if args.viz == "rs":
                # Attempt to set min/max distance on the colorizer, but *do not hide failures*.
                # If your SDK doesn't support these options, you'll see it and can switch to manual.
                ok_min = _safe_set(colorizer, rs.option.min_distance, float(args.min_depth), "colorizer.min_distance")
                ok_max = _safe_set(colorizer, rs.option.max_distance, float(args.max_depth), "colorizer.max_distance")
                if (not ok_min) or (not ok_max):
                    # Only print once in a while would be better, but keep simple.
                    pass
                depth_color_frame = colorizer.colorize(depth)
                depth_vis = np.asanyarray(depth_color_frame.get_data())
            else:
                depth_vis = _manual_colorize(depth_z16, depth_scale, args.min_depth, args.max_depth, args.scheme)

            # Compose side-by-side
            depth_vis_bgr = depth_vis  # already BGR
            if depth_vis_bgr.shape[:2] != color_img.shape[:2]:
                depth_vis_bgr = cv2.resize(depth_vis_bgr, (color_img.shape[1], color_img.shape[0]),
                                           interpolation=cv2.INTER_NEAREST)

            canvas = np.hstack([color_img, depth_vis_bgr])

            # Overlay debug text
            txt1 = f"center depth: {center_m:.3f} m | invalid: {invalid_ratio:.1f}%"
            txt2 = f"viz={args.viz} scheme={args.scheme} range={args.min_depth:.2f}-{args.max_depth:.2f} m post={'on' if args.post else 'off'}"
            cv2.putText(canvas, txt1, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(canvas, txt2, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.imshow("RealSense D435i (viewer-like debug)", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()