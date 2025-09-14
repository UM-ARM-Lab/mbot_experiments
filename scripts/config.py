#!/usr/bin/env python3
"""
MBot Experiment Configuration
"""

# LCM Configuration
DEFAULT_LCM_ADDRESS = "udpm://239.255.76.67:7667?ttl=1"

# Control System Parameters  
CONTROL_RATE_HZ = 25.0
VICON_RATE_HZ = 100.0

# Very approximate for percentage calculation in GUI
BATTERY_VOLTAGE_MIN = 9.0
BATTERY_VOLTAGE_MAX = 12.6

# LCM Channels
VEL_CMD_CHANNEL = "MBOT_VEL_CMD"
MOTION_CMD_CHANNEL = "MBOT_MOTION_CMD"
MOTION_STATUS_CHANNEL = "MBOT_MOTION_STATUS"
BATTERY_CHANNEL = "MBOT_ANALOG_IN"
DEFAULT_POSE_CHANNEL = "VICON_STATE"
DEFAULT_VELOCITY_CHANNEL = "VICON_TWIST_BODY"
VICON_BODY_NAME = "arm-mbot"
ROBOT_HOSTNAME = "arm-mbot"
ROBOT_USER = "mbot"
BLUE_WALL_SIZE = [1.36, 0.44] # [width, height] in meters
ROBOT_SAFE_RADIUS = 0.2  # Robot safe radius in meters (for safety margin calculation)

ROBOT_BODY_OFFSET_X = -0.010  # -10mm along body-frame x (forward direction)
ROBOT_BODY_OFFSET_Y = -0.060  # -60mm along body-frame y (left direction)

# Velocity Limits
MIN_LINEAR_VEL = 0.1    # m/s
MAX_LINEAR_VEL = 0.3    # m/s
MAX_LINEAR_ACCEL = 0.5  # m/s^2
MAX_ANGULAR_VEL = 0.5   # rad/s
MAX_ANGULAR_ACCEL = 2.0  # rad/s^2

# Logging
LOG_BASE_DIR = "real_data"