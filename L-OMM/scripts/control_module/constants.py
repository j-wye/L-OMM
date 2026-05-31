#!/usr/bin/env python3
"""Shared scalar/vector constants for the control module (no classes)."""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np


Rect = Tuple[float, float, float, float]
Cell = Tuple[int, int]

DEFAULT_TAG = "0.50"
DEFAULT_MU_MIN = 0.002
DEFAULT_OBSTACLE: Rect = (0.20, 0.50, 0.30, 0.30)
DEFAULT_MAP_UPDATE_HZ = 10.0
DEFAULT_OBSTACLE_INFLATION_M = 0.005
DEFAULT_TRACKING_GAP_M = 0.01
DEFAULT_GOAL_PROJECTION_FACTOR = 1.5
DEFAULT_COST_MODE = "manipulability"
DEFAULT_MANIP_WEIGHT = 0.25
DEFAULT_MANIP_KAPPA = 3.0
DEFAULT_MANIP_MU_SAFE = 0.0
DEFAULT_MANIP_MU_SAFE_PERCENTILE = 25.0

N_ACTIVE_JOINTS = 3
N_CHAIN_JOINTS = 5
N_JOINTS = N_ACTIVE_JOINTS
TASK_DIM = 3
ARM_BASE_Z = 0.47971
Y_PLANE_FIXED = 0.047
X_EE_GOAL = 0.40
X_TARGET_CONTACT = 0.53
TOOL_CONTACT_OFFSET = 0.13

Q1_FIXED = 0.0
Q4_FIXED = math.pi / 2.0
Q6_FIXED = -math.pi / 2.0

JOINT_OFFSETS: Tuple[Tuple[float, float, float, float, float, float], ...] = (
    (0.0,    -0.03,   0.115,   math.pi / 2.0,  0.0,           0.0),
    (0.0,     0.28,   0.0,    -math.pi,        0.0,           0.0),
    (0.0,    -0.14,   0.02,    math.pi / 2.0,  0.0,           0.0),
    (0.0285,  0.0,    0.105,   0.0,            math.pi / 2.0, 0.0),
    (-0.105,  0.0,    0.0285,  0.0,           -math.pi / 2.0, 0.0),
)

JL_LO = np.array([-2.69, -2.69, -2.59, -2.57, -2.59], dtype=np.float64)
JL_HI = np.array([ 2.69,  2.69,  2.59,  2.57,  2.59], dtype=np.float64)
ACTIVE_CHAIN_INDICES = np.array([0, 1, 3], dtype=np.int64)
JL_LO_ACTIVE = JL_LO[ACTIVE_CHAIN_INDICES].copy()
JL_HI_ACTIVE = JL_HI[ACTIVE_CHAIN_INDICES].copy()

# Reduced planar capsule proxy used for runtime arm-volume safety.
# The indices refer to Kinematics.fk_chain output frames:
# q2 pivot, q3 pivot, q5 pivot, and EE reference point.
# Radii are rounded from STL x-z projection width for each reduced capsule
# segment.  The wrist segment uses the signed cross-section width rather than
# the finite end-cap overhang so the reduced proxy stays compact.
CAPSULE_PROXY_FRAME_INDICES = np.array([1, 2, 4, 5], dtype=np.int64)
CAPSULE_PROXY_RADII_M = np.array([0.040, 0.041, 0.033], dtype=np.float64)
CAPSULE_PROXY_SAMPLE_DS_M = 0.005

Q_HOME_DEFAULT: Tuple[float, ...] = (
    1.2596254348754883,
    2.0562567710876465,
    1.2892378568649292,
)
