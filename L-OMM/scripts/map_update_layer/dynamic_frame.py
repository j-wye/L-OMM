#!/usr/bin/env python3
"""Dataclass contracts for synthetic dynamic map-update dry runs."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple

from .map_update_request import MapUpdateRequest


@dataclass(frozen=True)
class SyntheticDynamicFrame:
    """One deterministic synthetic frame.

    The frame is intentionally camera-free.  It only carries the semantic
    geometry request that the map-update layer already understands.
    """

    frame_index: int
    label: str
    request: MapUpdateRequest
    current_s_fraction: float = 0.0
    expected_event: str = ""
    target_changed_event: bool = False
    notes: str = ""
    metadata: Dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class SyntheticDynamicScenario:
    """A camera-free dynamic sequence used before live runtime integration."""

    name: str
    description: str
    frames: Tuple[SyntheticDynamicFrame, ...] = field(default_factory=tuple)
    metadata: Dict[str, object] = field(default_factory=dict)
    initial_request: MapUpdateRequest | None = None
