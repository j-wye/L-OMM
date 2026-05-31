#!/usr/bin/env python3
"""Keyframe extractor output (indices into source path + diagnostic metrics)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List


@dataclass
class KeyframeResult:
    indices: List[int]
    metrics: Dict[str, Any]
