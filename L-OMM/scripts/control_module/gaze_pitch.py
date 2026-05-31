#!/usr/bin/env python3
"""Vectorized deterministic gaze-pitch reconstruction."""
from __future__ import annotations

from typing import Any, Dict

import numpy as np


class GazePitch:
    """Per-map gaze constants and atan2 reconstruction."""

    __slots__ = ("x_gaze", "target_height", "_meta_keys")

    def __init__(self, meta: Dict[str, Any]) -> None:
        try:
            self.x_gaze = float(meta["target_contact_x_m"])
            self.target_height = float(meta["target_height"])
        except KeyError as exc:
            raise KeyError(
                f"GazePitch requires 'target_contact_x_m' and 'target_height' in map metadata; missing {exc}"
            )
        self._meta_keys = ("target_contact_x_m", "target_height")

    def reconstruct(self, xz: np.ndarray) -> np.ndarray:
        pts = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
        dx = self.x_gaze - pts[:, 0]
        dz = self.target_height - pts[:, 1]
        return np.arctan2(dx, dz)

    def reconstruct_scalar(self, x: float, z: float) -> float:
        return float(np.arctan2(self.x_gaze - float(x), self.target_height - float(z)))

    def reconstruction_error(self,
                             pitch_grid: np.ndarray,
                             handle_x0: float,
                             handle_z0: float,
                             resolution_m: float,
                             valid_mask: np.ndarray | None = None) -> Dict[str, float]:
        n_x, n_z = pitch_grid.shape
        ix = np.arange(n_x).reshape(-1, 1)
        iz = np.arange(n_z).reshape(1, -1)
        x = handle_x0 + ix * resolution_m
        z = handle_z0 + iz * resolution_m
        recon = np.arctan2(self.x_gaze - x, self.target_height - z).astype(np.float64)
        stored = np.asarray(pitch_grid, dtype=np.float64)
        err = np.abs(recon - stored)
        if valid_mask is not None:
            err = err[np.asarray(valid_mask, dtype=bool)]
        if err.size == 0:
            return {
                "pitch_recon_max_err_rad": 0.0,
                "pitch_recon_rms_err_rad": 0.0,
                "pitch_recon_p98_err_rad": 0.0,
                "pitch_recon_sample_count": 0,
            }
        return {
            "pitch_recon_max_err_rad": float(err.max()),
            "pitch_recon_rms_err_rad": float(np.sqrt(np.mean(err * err))),
            "pitch_recon_p98_err_rad": float(np.percentile(err, 98)),
            "pitch_recon_sample_count": int(err.size),
        }
