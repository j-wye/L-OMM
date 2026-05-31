#!/usr/bin/env python3
"""Stored-frame RealSense D435 depth-noise calibration helper.

This script does not open a camera, subscribe to ROS2, or command a robot.
It either writes a Jetson measurement protocol, or processes stored depth
frames collected by the user.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Process stored depth frames for RealSense depth-noise calibration. "
            "No live camera, ROS2 topic, or robot command is used."
        )
    )
    parser.add_argument("--out", default=None, help="Output directory.")
    parser.add_argument("--manifest", default=None, help="CSV with columns path,distance_m.")
    parser.add_argument("--frames-dir", default=None, help="Directory containing .npy depth frames.")
    parser.add_argument("--roi", default=None, help="Optional ROI as u0,v0,u1,v1. Defaults to center 40%% crop.")
    parser.add_argument("--write-protocol-only", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "depth_noise_calibration"))
    os.makedirs(out_root, exist_ok=True)
    protocol_path = os.path.join(out_root, "depth_noise_calibration_protocol.md")
    _write_protocol(protocol_path)
    if args.write_protocol_only or (not args.manifest and not args.frames_dir):
        summary = {
            "mode": "depth_noise_calibration_protocol_only",
            "explicitly_no_live_camera": True,
            "explicitly_no_ros2_topic": True,
            "explicitly_no_physical_robot_execution": True,
            "requires_hardware_capture": True,
            "protocol_path": protocol_path,
            "pass": False,
        }
        _write_json(os.path.join(out_root, "depth_noise_calibration_summary.json"), summary)
        print(f"[depth_noise_calibration] protocol_only out={out_root}")
        return 0
    records = _load_records(args.manifest, args.frames_dir)
    rows = _measure(records, _parse_roi(args.roi))
    fit = _fit(rows)
    summary = {
        "mode": "stored_depth_noise_calibration",
        "explicitly_no_live_camera": True,
        "explicitly_no_ros2_topic": True,
        "explicitly_no_physical_robot_execution": True,
        "requires_hardware_capture": True,
        "distance_count": len(rows),
        "frame_count": int(sum(int(row["frame_count"]) for row in rows)),
        "fit": fit,
        "protocol_path": protocol_path,
        "pass": bool(fit["monotonic_non_decreasing"] and fit["r2"] >= 0.95),
    }
    _write_csv(os.path.join(out_root, "depth_noise_by_distance.csv"), rows)
    _write_csv(os.path.join(out_root, "depth_noise_fit_coefficients.csv"), [fit])
    _write_json(os.path.join(out_root, "depth_noise_calibration_summary.json"), summary)
    _write_plot(os.path.join(out_root, "depth_noise_fit.png"), rows, fit)
    print(
        "[depth_noise_calibration] "
        f"distances={len(rows)} r2={fit['r2']:.3f} pass={summary['pass']} out={out_root}"
    )
    return 0 if summary["pass"] else 1


def _write_protocol(path: str) -> None:
    text = """# RealSense D435 Depth Noise Calibration Protocol

This protocol is intentionally hardware-measurement only.  The script that
created this file does not open the camera or use ROS2.

1. Place a matte planar board at 0.3 m, 0.5 m, 1.0 m, 1.5 m, and 2.0 m from the camera.
2. Keep exposure, IR projector, resolution, and depth units fixed for the whole capture.
3. Capture at least 30 aligned depth frames per distance.
4. Save each depth frame as a `.npy` array in meters.
5. Either create a manifest CSV with `path,distance_m`, or store files in folders whose names contain the distance, such as `d_0.50/frame_000.npy`.
6. Run this script with `--manifest` or `--frames-dir`.

Expected artifacts:
- `depth_noise_by_distance.csv`
- `depth_noise_fit_coefficients.csv`
- `depth_noise_fit.png`
- `depth_noise_calibration_summary.json`

Jetson execution after capture:

```bash
python3 L-OMM/scripts/map_update_layer/experiments/depth_noise_calibration.py \\
    --frames-dir /home/orin/depth_calibration_frames/ \\
    --out path/p2_depth_noise_calibration

cat path/p2_depth_noise_calibration/depth_noise_calibration_summary.json
```

Pass criteria:
- `fit.r2 >= 0.95`
- `fit.monotonic_non_decreasing = true`
- If the measured model differs from `0.001 + 0.002*d^2` by more than about 30%,
  update `realsense_d435_sigma_depth_m` and rerun the P2.1 sweep.

Optional no-ROS2 `.npy` capture sketch for Jetson:

```python
import os
import numpy as np
import pyrealsense2 as rs

pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(cfg)

distance_m = 0.50
out_dir = f"/home/orin/depth_calibration_frames/d_{distance_m:.2f}"
os.makedirs(out_dir, exist_ok=True)

try:
    for i in range(30):
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        arr = np.asarray(depth.get_data(), dtype=np.float32) * depth.get_units()
        np.save(f"{out_dir}/frame_{i:03d}.npy", arr)
finally:
    pipeline.stop()
```
"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


def _load_records(manifest: Optional[str], frames_dir: Optional[str]) -> List[Tuple[str, float]]:
    records: List[Tuple[str, float]] = []
    if manifest:
        with open(manifest, "r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                records.append((os.path.abspath(row["path"]), float(row["distance_m"])))
    if frames_dir:
        for root, _, files in os.walk(frames_dir):
            dist = _distance_from_path(root)
            for name in files:
                if name.lower().endswith(".npy"):
                    d = dist if dist is not None else _distance_from_path(name)
                    if d is None:
                        continue
                    records.append((os.path.abspath(os.path.join(root, name)), float(d)))
    if not records:
        raise ValueError("no depth frames found; provide --manifest or distance-coded --frames-dir")
    return records


def _distance_from_path(text: str) -> Optional[float]:
    match = re.search(r"(?:d|dist|distance)[_-]?([0-9]+(?:\.[0-9]+)?)", str(text), flags=re.IGNORECASE)
    return float(match.group(1)) if match else None


def _measure(records: List[Tuple[str, float]], roi: Optional[Tuple[int, int, int, int]]) -> List[Dict[str, Any]]:
    groups: Dict[float, List[str]] = {}
    for path, distance in records:
        groups.setdefault(float(distance), []).append(path)
    rows: List[Dict[str, Any]] = []
    for distance, paths in sorted(groups.items()):
        frame_means: List[float] = []
        frame_stds: List[float] = []
        for path in sorted(paths):
            depth = np.asarray(np.load(path), dtype=np.float64)
            patch = _roi(depth, roi)
            vals = patch[np.isfinite(patch) & (patch > 0.0)]
            if vals.size == 0:
                continue
            frame_means.append(float(np.mean(vals)))
            frame_stds.append(float(np.std(vals)))
        if not frame_stds:
            continue
        rows.append(
            {
                "distance_m": float(distance),
                "frame_count": len(frame_stds),
                "depth_mean_m": float(np.mean(frame_means)),
                "sigma_depth_m": float(np.mean(frame_stds)),
                "sigma_depth_mm": float(1000.0 * np.mean(frame_stds)),
                "sigma_depth_frame_std_m": float(np.std(frame_stds)),
            }
        )
    return rows


def _roi(depth: np.ndarray, roi: Optional[Tuple[int, int, int, int]]) -> np.ndarray:
    h, w = depth.shape[:2]
    if roi is None:
        u0, u1 = int(0.3 * w), int(0.7 * w)
        v0, v1 = int(0.3 * h), int(0.7 * h)
    else:
        u0, v0, u1, v1 = roi
        u0, u1 = max(0, u0), min(w - 1, u1)
        v0, v1 = max(0, v0), min(h - 1, v1)
    return depth[v0:v1 + 1, u0:u1 + 1]


def _fit(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    if len(rows) < 3:
        raise ValueError("at least three distances are required for a quadratic fit")
    d = np.asarray([float(row["distance_m"]) for row in rows], dtype=np.float64)
    y = np.asarray([float(row["sigma_depth_m"]) for row in rows], dtype=np.float64)
    coeff = np.polyfit(d, y, deg=2)
    pred = np.polyval(coeff, d)
    ss_res = float(np.sum(np.square(y - pred)))
    ss_tot = float(np.sum(np.square(y - np.mean(y))))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 1.0
    mono = bool(np.all(np.diff(y[np.argsort(d)]) >= -1.0e-9))
    return {
        "quadratic_a_m_per_m2": float(coeff[0]),
        "linear_b_m_per_m": float(coeff[1]),
        "constant_c_m": float(coeff[2]),
        "r2": float(r2),
        "monotonic_non_decreasing": bool(mono),
    }


def _parse_roi(text: Optional[str]) -> Optional[Tuple[int, int, int, int]]:
    if not text:
        return None
    vals = [int(v.strip()) for v in str(text).split(",")]
    if len(vals) != 4:
        raise ValueError("--roi must be u0,v0,u1,v1")
    return vals[0], vals[1], vals[2], vals[3]


def _write_csv(path: str, rows: Iterable[Dict[str, Any]]) -> None:
    rows = list(rows)
    keys: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in keys:
                keys.append(key)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def _write_json(path: str, data: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _write_plot(path: str, rows: List[Dict[str, Any]], fit: Dict[str, Any]) -> Optional[str]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return None
    d = np.asarray([float(row["distance_m"]) for row in rows], dtype=np.float64)
    y = np.asarray([float(row["sigma_depth_mm"]) for row in rows], dtype=np.float64)
    xs = np.linspace(float(np.min(d)), float(np.max(d)), 120)
    coeff_m = np.asarray([
        fit["quadratic_a_m_per_m2"],
        fit["linear_b_m_per_m"],
        fit["constant_c_m"],
    ], dtype=np.float64)
    ys = 1000.0 * np.polyval(coeff_m, xs)
    fig, ax = plt.subplots(figsize=(5.5, 4.0), dpi=150)
    ax.scatter(d, y, label="measured")
    ax.plot(xs, ys, label=f"quadratic fit R2={fit['r2']:.3f}")
    ax.set_xlabel("depth [m]")
    ax.set_ylabel("sigma depth [mm]")
    ax.set_title("RealSense depth noise calibration")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


if __name__ == "__main__":
    raise SystemExit(main())
