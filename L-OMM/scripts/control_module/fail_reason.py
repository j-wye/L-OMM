#!/usr/bin/env python3
"""Episode failure classification."""
from __future__ import annotations

from enum import Enum


class FailReason(Enum):
    SUCCESS = 0
    POS_TOL_EXCEED = 1
    ROT_TOL_EXCEED = 2
    JOINT_LIMIT_STUCK = 3
    NON_FINITE_STATE = 4
    INVALID_INPUT = 5
    REFERENCE_PROGRESS_INCOMPLETE = 6
