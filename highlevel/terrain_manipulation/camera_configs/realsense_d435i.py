"""Shared configuration values for the Intel RealSense D435i.

This module centralizes all the *defaults* that control how the camera and
depth processing behave. Individual scripts can still override these via
CLI flags, but if you want to change the global behavior, this is the
only file you need to touch.
"""

# Stream configuration (sensor-level).
STREAM_WIDTH = 640
STREAM_HEIGHT = 480
STREAM_FPS = 30

# Depth range (in meters) used for clamping/normalization.
DEFAULT_MIN_DEPTH_M = 0.0
DEFAULT_MAX_DEPTH_M = 1.5

# RealSense visual preset used for the depth sensor.
# Must be one of: "default", "high_accuracy", "high_density", "medium_density".
DEFAULT_VISUAL_PRESET = "high_density"

# Decimation magnitude (>=1). Values >1 downsample depth to reduce noise.
DEFAULT_DECIMATE_MAGNITUDE = 1

# Whether to automatically estimate depth min/max per frame using percentiles.
DEFAULT_AUTO_RANGE = True

# Extra smoothing applied only to the depth visualization (not raw depth).
# One of: "none", "median", "gaussian", "bilateral".
DEFAULT_DISPLAY_SMOOTH = "median"

# Kernel size for display smoothing (odd integer).
DEFAULT_SMOOTH_KERNEL = 5

# Default perceptual colormap name for depth visualization.
DEFAULT_COLORMAP_NAME = "turbo"

