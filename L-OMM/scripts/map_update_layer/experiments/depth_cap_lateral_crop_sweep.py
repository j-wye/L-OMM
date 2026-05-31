#!/usr/bin/env python3
"""Synthetic depth-cap and lateral-crop sweep for CameraFrameMapUpdateBridge."""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

import numpy as np


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(0, CONTROL_MODULE_DIR)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(1, SCRIPTS_DIR)

from map_update_layer.camera_frame_bridge import apply_depth_cap, apply_lateral_crop  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run a synthetic DEPTH_RELEVANT_MAX_M / CROP_HALF_PIXELS sweep. "
            "No live camera, ROS2 topic, or physical robot execution."
        )
    )
    parser.add_argument("--out", default=None)
    parser.add_argument("--depth-cap-range", default="0.5,0.75,1.0,1.25,1.5,2.0")
    parser.add_argument("--crop-range", default="100,150,200,250,300,400,640")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--seed", type=int, default=42)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = Path(args.out or os.path.join(PROJECT_ROOT, "path", "depth_cap_lateral_crop_sweep"))
    out_root.mkdir(parents=True, exist_ok=True)
    depth_caps = float_list(args.depth_cap_range)
    crop_values = int_list(args.crop_range)
    depth = synthetic_depth(width=int(args.width), height=int(args.height), seed=int(args.seed))
    rows = run_sweep(depth=depth, depth_caps=depth_caps, crop_values=crop_values)
    summary = summarize(rows, width=int(args.width), height=int(args.height), seed=int(args.seed))
    write_csv(out_root / "depth_cap_lateral_crop_sweep.csv", rows)
    write_json(out_root / "depth_cap_lateral_crop_sweep_summary.json", summary)
    print(
        "[depth_cap_lateral_crop_sweep] "
        f"rows={len(rows)} best_drop={summary['max_estimated_processing_load_reduction']:.3f} out={out_root}"
    )
    return 0 if summary["pass"] else 1


def synthetic_depth(*, width: int, height: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    u = np.linspace(0.0, 1.0, int(width), dtype=np.float64)[None, :]
    v = np.linspace(0.0, 1.0, int(height), dtype=np.float64)[:, None]
    base = 0.25 + 1.75 * u + 0.20 * v
    noise = rng.normal(loc=0.0, scale=0.02, size=(int(height), int(width)))
    depth = np.clip(base + noise, 0.05, 3.0)
    invalid_stride = max(1, int(width) // 40)
    depth[:, ::invalid_stride] = 0.0
    return depth.astype(np.float64)


def run_sweep(*, depth: np.ndarray, depth_caps: Sequence[float], crop_values: Sequence[int]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    total = int(np.asarray(depth).size)
    for cap in depth_caps:
        capped, cap_stats = apply_depth_cap(depth, max_depth_m=float(cap))
        for crop_half in crop_values:
            cropped, crop_stats = apply_lateral_crop(capped, half_pixels=int(crop_half))
            kept = np.isfinite(cropped) & (cropped > 0.0)
            filtered_count = int(np.count_nonzero(kept))
            combined_drop = float(1.0 - filtered_count / max(total, 1))
            rows.append(
                {
                    "depth_relevant_max_m": float(cap),
                    "crop_half_pixels": int(crop_half),
                    "drop_ratio_depth": float(cap_stats["depth_cap_drop_ratio"]),
                    "drop_ratio_lateral": float(crop_stats["lateral_crop_drop_ratio"]),
                    "drop_ratio_combined": float(combined_drop),
                    "filtered_pixel_count": int(filtered_count),
                    "input_pixel_count": int(total),
                    "estimated_processing_load_reduction": float(combined_drop),
                }
            )
    return rows


def summarize(rows: Sequence[Mapping[str, Any]], *, width: int, height: int, seed: int) -> Dict[str, Any]:
    return {
        "mode": "depth_cap_lateral_crop_sweep",
        "explicitly_no_live_camera": True,
        "explicitly_no_ros2_topic": True,
        "explicitly_no_physical_robot_execution": True,
        "width": int(width),
        "height": int(height),
        "seed": int(seed),
        "row_count": int(len(rows)),
        "max_estimated_processing_load_reduction": float(
            max((float(row["estimated_processing_load_reduction"]) for row in rows), default=0.0)
        ),
        "min_filtered_pixel_count": int(min((int(row["filtered_pixel_count"]) for row in rows), default=0)),
        "pass": bool(rows and all(0.0 <= float(row["drop_ratio_combined"]) <= 1.0 for row in rows)),
    }


def float_list(text: str) -> List[float]:
    return [float(v.strip()) for v in str(text).split(",") if v.strip()]


def int_list(text: str) -> List[int]:
    return [int(float(v.strip())) for v in str(text).split(",") if v.strip()]


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    keys: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, data: Mapping[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


if __name__ == "__main__":
    raise SystemExit(main())
