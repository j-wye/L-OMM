#!/usr/bin/env python3
"""Offline audit for invalid-depth unknown projection conservatism.

The script consumes rich ``live_snapshot_passive_decision`` artifacts only.  It
does not subscribe to camera topics, run ROS, or change production map-update
semantics.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, Mapping, Sequence

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))

from map_handle import MapHandle  # noqa: E402
from map_update_layer.depth_geometry_evidence import (  # noqa: E402
    DepthGeometryConfig,
    DepthGeometryEvidenceBuilder,
    _image_component_unknown_rect,
)
from map_update_layer.perception_to_map import CameraIntrinsics  # noqa: E402


PER_FRAME_CSV = "invalid_depth_unknown_geometry_per_frame.csv"
SUMMARY_JSON = "invalid_depth_unknown_geometry_summary.json"
TEMPORAL_JSON = "temporal_stability.json"
CAPSULE_JSON = "capsule_aware_overlap.json"


@dataclass(frozen=True)
class FrameInput:
    frame_id: int
    npz_path: Path
    metadata_path: Path
    depth_m: np.ndarray
    unknown_candidate_mask: np.ndarray
    T_base_cam: np.ndarray
    intrinsics: CameraIntrinsics
    config: DepthGeometryConfig
    handle: MapHandle
    goal_corridor_cells: tuple[tuple[int, int], ...]


def analyze_run(*, run_dir: Path | str, out_dir: Path | str | None = None) -> Dict[str, Any]:
    run_path = Path(run_dir)
    out_path = Path(out_dir) if out_dir is not None else run_path / "stage_b_geometry_audit"
    out_path.mkdir(parents=True, exist_ok=True)
    inputs = _load_frame_inputs(run_path)
    rows = [_analyze_frame(frame) for frame in inputs]
    _write_csv(out_path / PER_FRAME_CSV, rows)
    summary = _aggregate_summary(rows)
    summary["run_dir"] = str(run_path)
    summary["out_dir"] = str(out_path)
    _write_json(out_path / SUMMARY_JSON, summary)
    temporal = _temporal_summary(rows)
    _write_json(out_path / TEMPORAL_JSON, temporal)
    overlap = _capsule_overlap_summary(rows)
    _write_json(out_path / CAPSULE_JSON, overlap)
    return summary


def _load_frame_inputs(run_dir: Path) -> list[FrameInput]:
    events_path = run_dir / "events.csv"
    if events_path.exists():
        event_rows = _read_csv(events_path)
    else:
        event_rows = [
            {
                "frame_id": str(idx),
                "frame_artifact": path.name,
                "frame_metadata_artifact": path.with_suffix(".json").name,
            }
            for idx, path in enumerate(sorted((run_dir / "frames").glob("frame_*.npz")))
        ]
    frames: list[FrameInput] = []
    for idx, row in enumerate(event_rows):
        frame_id = int(row.get("frame_id", idx))
        npz_path = run_dir / "frames" / str(row.get("frame_artifact", f"frame_{frame_id:04d}.npz"))
        metadata_path = run_dir / "frames" / str(
            row.get("frame_metadata_artifact", f"frame_{frame_id:04d}.json")
        )
        if not npz_path.exists() or not metadata_path.exists():
            continue
        data = np.load(npz_path, allow_pickle=False)
        missing = [name for name in ("depth_m", "unknown_candidate_mask", "T_base_cam") if name not in data.files]
        if missing:
            raise RuntimeError(f"{npz_path} missing required audit arrays: {', '.join(missing)}")
        metadata = _read_json(metadata_path)
        intrinsics = _intrinsics(metadata["intrinsics"])
        cfg = _config(metadata["depth_geometry_config"])
        handle = _handle(metadata["map_geometry"])
        goal_cells = tuple(
            (int(cell[0]), int(cell[1]))
            for cell in metadata.get("goal_diagnostics", {}).get("goal_corridor_cells", [])
        )
        frames.append(
            FrameInput(
                frame_id=frame_id,
                npz_path=npz_path,
                metadata_path=metadata_path,
                depth_m=np.asarray(data["depth_m"], dtype=np.float64),
                unknown_candidate_mask=np.asarray(data["unknown_candidate_mask"], dtype=bool),
                T_base_cam=np.asarray(data["T_base_cam"], dtype=np.float64),
                intrinsics=intrinsics,
                config=cfg,
                handle=handle,
                goal_corridor_cells=goal_cells,
            )
        )
    return frames


def _analyze_frame(frame: FrameInput) -> Dict[str, Any]:
    depth = frame.depth_m
    finite_positive = np.isfinite(depth) & (depth > 0.0)
    band = _pixel_band(depth.shape, frame.intrinsics, frame.config.pixel_band_half_width_px)
    invalid = band & (~finite_positive) & frame.unknown_candidate_mask
    v_idx, u_idx = np.nonzero(invalid)
    min_pixels = max(int(frame.config.min_unknown_component_pixels), 1)
    if v_idx.size < min_pixels:
        return _empty_row(frame)
    u_min, u_max = int(np.min(u_idx)), int(np.max(u_idx))
    v_min, v_max = int(np.min(v_idx)), int(np.max(v_idx))
    prod_rect = _project_rect(frame, u_min, u_max, v_min, v_max, (frame.config.depth_min_m, frame.config.depth_max_m))
    min_rect = _project_rect(frame, u_min, u_max, v_min, v_max, (frame.config.depth_min_m,))
    adjacent_depths = _adjacent_valid_depths(depth, invalid, u_min, u_max, v_min, v_max)
    median_depth = float(np.median(adjacent_depths)) if adjacent_depths.size >= frame.config.min_unknown_component_pixels else math.nan
    median_rect = (
        _project_rect(frame, u_min, u_max, v_min, v_max, (median_depth,))
        if np.isfinite(median_depth)
        else None
    )
    prod_cells = _rect_cells(frame.handle, prod_rect)
    min_cells = _rect_cells(frame.handle, min_rect)
    median_cells = _rect_cells(frame.handle, median_rect)
    goal = set(frame.goal_corridor_cells)
    prod_consistency = _production_consistency(frame, prod_rect)
    return {
        "frame_id": frame.frame_id,
        "invalid_pixel_count": int(v_idx.size),
        "invalid_bbox_u_min": u_min,
        "invalid_bbox_u_max": u_max,
        "invalid_bbox_v_min": v_min,
        "invalid_bbox_v_max": v_max,
        "invalid_centroid_u": float(np.mean(u_idx)),
        "invalid_centroid_v": float(np.mean(v_idx)),
        **_rect_fields("prod_rect", prod_rect, frame.handle),
        **_rect_fields("min_depth_rect", min_rect, frame.handle),
        "median_adjacent_depth_value_m": median_depth,
        **_rect_fields("median_adjacent_rect", median_rect, frame.handle),
        "goal_corridor_overlap_prod": int(len(goal & prod_cells)),
        "goal_corridor_overlap_min_depth": int(len(goal & min_cells)),
        "goal_corridor_overlap_median_adjacent": int(len(goal & median_cells)),
        "adjacent_valid_pixel_count": int(adjacent_depths.size),
        "production_consistency_check_ok": bool(prod_consistency),
        "_prod_cells": sorted(prod_cells),
        "_min_cells": sorted(min_cells),
        "_median_cells": sorted(median_cells),
        "_goal_cells": sorted(goal),
    }


def _empty_row(frame: FrameInput) -> Dict[str, Any]:
    row = {
        "frame_id": frame.frame_id,
        "invalid_pixel_count": 0,
        "invalid_bbox_u_min": -1,
        "invalid_bbox_u_max": -1,
        "invalid_bbox_v_min": -1,
        "invalid_bbox_v_max": -1,
        "invalid_centroid_u": math.nan,
        "invalid_centroid_v": math.nan,
        "median_adjacent_depth_value_m": math.nan,
        "goal_corridor_overlap_prod": 0,
        "goal_corridor_overlap_min_depth": 0,
        "goal_corridor_overlap_median_adjacent": 0,
        "adjacent_valid_pixel_count": 0,
        "production_consistency_check_ok": True,
        "_prod_cells": [],
        "_min_cells": [],
        "_median_cells": [],
        "_goal_cells": list(frame.goal_corridor_cells),
    }
    row.update(_rect_fields("prod_rect", None, frame.handle))
    row.update(_rect_fields("min_depth_rect", None, frame.handle))
    row.update(_rect_fields("median_adjacent_rect", None, frame.handle))
    return row


def _project_rect(
    frame: FrameInput,
    u_min: int,
    u_max: int,
    v_min: int,
    v_max: int,
    depths: Sequence[float],
) -> tuple[float, float, float, float] | None:
    finite_depths = [float(d) for d in depths if np.isfinite(float(d))]
    if not finite_depths:
        return None
    if len(finite_depths) == 2:
        return _image_component_unknown_rect(
            handle=frame.handle,
            u_min=u_min,
            u_max=u_max,
            v_min=v_min,
            v_max=v_max,
            intrinsics=frame.intrinsics,
            T_base_cam=frame.T_base_cam,
            depth_min_m=finite_depths[0],
            depth_max_m=finite_depths[1],
        )
    samples = np.asarray(
        [(float(u), float(v), finite_depths[0]) for u in (u_min, u_max) for v in (v_min, v_max)],
        dtype=np.float64,
    )
    points_cam = _backproject(samples[:, 0], samples[:, 1], samples[:, 2], frame.intrinsics)
    points_base = (frame.T_base_cam[:3, :3] @ points_cam.T).T + frame.T_base_cam[:3, 3][None, :]
    return _clip_rect(frame.handle, float(np.min(points_base[:, 0])), float(np.max(points_base[:, 0])), float(np.min(points_base[:, 2])), float(np.max(points_base[:, 2])))


def _production_consistency(frame: FrameInput, prod_rect: tuple[float, float, float, float] | None) -> bool:
    builder = DepthGeometryEvidenceBuilder(frame.config)
    evidence = builder.build(
        depth_m=frame.depth_m,
        intrinsics=frame.intrinsics,
        T_base_cam=frame.T_base_cam,
        handle=frame.handle,
        unknown_candidate_mask=frame.unknown_candidate_mask,
    )
    if prod_rect is None:
        return len(evidence.unknown_rects) == 0
    if len(evidence.unknown_rects) != 1:
        return False
    return bool(np.allclose(np.asarray(evidence.unknown_rects[0]), np.asarray(prod_rect), atol=1.0e-9))


def _pixel_band(shape: Sequence[int], intrinsics: CameraIntrinsics, half_width: float) -> np.ndarray:
    h, w = int(shape[0]), int(shape[1])
    uu = np.arange(w, dtype=np.float64)[None, :]
    return np.broadcast_to(np.abs(uu - float(intrinsics.cx)) <= float(half_width), (h, w))


def _adjacent_valid_depths(
    depth: np.ndarray,
    invalid: np.ndarray,
    u_min: int,
    u_max: int,
    v_min: int,
    v_max: int,
) -> np.ndarray:
    h, w = depth.shape
    u0, u1 = max(0, u_min - 1), min(w - 1, u_max + 1)
    v0, v1 = max(0, v_min - 1), min(h - 1, v_max + 1)
    border = np.zeros_like(invalid, dtype=bool)
    border[v0:v1 + 1, u0:u1 + 1] = True
    border[v_min:v_max + 1, u_min:u_max + 1] = False
    vals = np.asarray(depth[border], dtype=np.float64)
    return vals[np.isfinite(vals) & (vals > 0.0)]


def _rect_fields(prefix: str, rect: tuple[float, float, float, float] | None, handle: MapHandle) -> Dict[str, Any]:
    if rect is None:
        xl = zt = xr = zb = math.nan
        area = math.nan
        count = 0
    else:
        xl, zt, xr, zb = rect
        area = max(0.0, float(xr) - float(xl)) * max(0.0, float(zt) - float(zb))
        count = len(_rect_cells(handle, rect))
    return {
        f"{prefix}_xl": float(xl),
        f"{prefix}_zt": float(zt),
        f"{prefix}_xr": float(xr),
        f"{prefix}_zb": float(zb),
        f"{prefix}_area_m2": float(area),
        f"{prefix}_cell_count": int(count),
    }


def _rect_cells(handle: MapHandle, rect: tuple[float, float, float, float] | None) -> set[tuple[int, int]]:
    if rect is None:
        return set()
    xl, zt, xr, zb = rect
    ix0 = max(0, int(math.floor((float(xl) - float(handle.x0)) / float(handle.resolution_m))))
    ix1 = min(handle.shape[0] - 1, int(math.ceil((float(xr) - float(handle.x0)) / float(handle.resolution_m))))
    iz0 = max(0, int(math.floor((float(zb) - float(handle.z0)) / float(handle.resolution_m))))
    iz1 = min(handle.shape[1] - 1, int(math.ceil((float(zt) - float(handle.z0)) / float(handle.resolution_m))))
    if ix1 < ix0 or iz1 < iz0:
        return set()
    return {(ix, iz) for ix in range(ix0, ix1 + 1) for iz in range(iz0, iz1 + 1)}


def _aggregate_summary(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    def stats(name: str) -> Dict[str, float]:
        vals = np.asarray([float(r[name]) for r in rows if np.isfinite(float(r.get(name, math.nan)))], dtype=np.float64)
        if vals.size == 0:
            return {"min": math.nan, "max": math.nan, "mean": math.nan}
        return {"min": float(np.min(vals)), "max": float(np.max(vals)), "mean": float(np.mean(vals))}

    prod_area = np.asarray([float(r["prod_rect_area_m2"]) for r in rows], dtype=np.float64)
    min_area = np.asarray([float(r["min_depth_rect_area_m2"]) for r in rows], dtype=np.float64)
    prod_overlap = np.asarray([float(r["goal_corridor_overlap_prod"]) for r in rows], dtype=np.float64)
    med_overlap = np.asarray([float(r["goal_corridor_overlap_median_adjacent"]) for r in rows], dtype=np.float64)
    ratio = prod_area / np.maximum(min_area, 1.0e-12)
    reduction = 1.0 - med_overlap / np.maximum(prod_overlap, 1.0)
    return {
        "frames_analyzed": int(len(rows)),
        "prod_rect_area_m2": stats("prod_rect_area_m2"),
        "prod_rect_cell_count": stats("prod_rect_cell_count"),
        "invalid_pixel_count": stats("invalid_pixel_count"),
        "goal_corridor_overlap_prod": stats("goal_corridor_overlap_prod"),
        "goal_corridor_overlap_min_depth": stats("goal_corridor_overlap_min_depth"),
        "goal_corridor_overlap_median_adjacent": stats("goal_corridor_overlap_median_adjacent"),
        "median_adjacent_depth_value_m": stats("median_adjacent_depth_value_m"),
        "production_consistency_check_true": int(sum(bool(r["production_consistency_check_ok"]) for r in rows)),
        "production_consistency_check_total": int(len(rows)),
        "production_consistency_check_rate": float(sum(bool(r["production_consistency_check_ok"]) for r in rows) / max(len(rows), 1)),
        "conservatism_ratio_prod_area_over_min_depth_area_median": _nanmedian(ratio),
        "median_adjacent_reduction_ratio_median": _nanmedian(reduction),
    }


def _temporal_summary(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    def std(name: str) -> float:
        vals = np.asarray([float(r[name]) for r in rows if np.isfinite(float(r.get(name, math.nan)))], dtype=np.float64)
        return float(np.std(vals)) if vals.size else math.nan
    cx = [0.5 * (float(r["prod_rect_xl"]) + float(r["prod_rect_xr"])) for r in rows if np.isfinite(float(r["prod_rect_xl"]))]
    cz = [0.5 * (float(r["prod_rect_zt"]) + float(r["prod_rect_zb"])) for r in rows if np.isfinite(float(r["prod_rect_zt"]))]
    centroid_std = math.hypot(std("invalid_centroid_u"), std("invalid_centroid_v"))
    return {
        "invalid_centroid_u_std_px": std("invalid_centroid_u"),
        "invalid_centroid_v_std_px": std("invalid_centroid_v"),
        "prod_rect_center_x_std_m": float(np.std(cx)) if cx else math.nan,
        "prod_rect_center_z_std_m": float(np.std(cz)) if cz else math.nan,
        "prod_rect_cell_count_std": std("prod_rect_cell_count"),
        "goal_corridor_overlap_prod_std": std("goal_corridor_overlap_prod"),
        "static_hole_score": float(1.0 / (1.0 + centroid_std)) if np.isfinite(centroid_std) else math.nan,
    }


def _capsule_overlap_summary(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    per_frame = []
    for row in rows:
        prod = set(tuple(v) for v in row["_prod_cells"])
        min_depth = set(tuple(v) for v in row["_min_cells"])
        median = set(tuple(v) for v in row["_median_cells"])
        goal = set(tuple(v) for v in row["_goal_cells"])
        variants = [prod, min_depth]
        if median:
            variants.append(median)
        essential = set.intersection(*(v & goal for v in variants)) if variants else set()
        artifact = (prod & goal) - set.union(*(v & goal for v in variants[1:])) if len(variants) > 1 else set()
        per_frame.append(
            {
                "frame_id": int(row["frame_id"]),
                "essential_overlap_count": int(len(essential)),
                "projection_artifact_overlap_count": int(len(artifact)),
                "production_goal_overlap_count": int(len(prod & goal)),
            }
        )
    return {
        "frames_analyzed": int(len(per_frame)),
        "essential_overlap_frame_average": _mean([r["essential_overlap_count"] for r in per_frame]),
        "projection_artifact_overlap_frame_average": _mean([r["projection_artifact_overlap_count"] for r in per_frame]),
        "per_frame": per_frame,
    }


def _backproject(u: np.ndarray, v: np.ndarray, depth_m: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray:
    x = (u - float(intrinsics.cx)) * depth_m / float(intrinsics.fx)
    y = (v - float(intrinsics.cy)) * depth_m / float(intrinsics.fy)
    z = depth_m
    return np.stack([x, y, z], axis=1)


def _clip_rect(handle: MapHandle, x0: float, x1: float, z0: float, z1: float) -> tuple[float, float, float, float] | None:
    xl = max(min(x0, x1), float(handle.x0))
    xr = min(max(x0, x1), float(handle.x0) + float(handle.shape[0] - 1) * float(handle.resolution_m))
    zb = max(min(z0, z1), float(handle.z0))
    zt = min(max(z0, z1), float(handle.z0) + float(handle.shape[1] - 1) * float(handle.resolution_m))
    if xr < xl or zt < zb:
        return None
    return xl, zt, xr, zb


def _intrinsics(data: Mapping[str, Any]) -> CameraIntrinsics:
    return CameraIntrinsics(
        fx=float(data["fx"]),
        fy=float(data["fy"]),
        cx=float(data["cx"]),
        cy=float(data["cy"]),
        width=int(data.get("width", 0)),
        height=int(data.get("height", 0)),
    )


def _config(data: Mapping[str, Any]) -> DepthGeometryConfig:
    return DepthGeometryConfig(
        pixel_band_half_width_px=float(data["pixel_band_half_width_px"]),
        depth_min_m=float(data["depth_min_m"]),
        depth_max_m=float(data["depth_max_m"]),
        y_plane=float(data.get("y_plane", 0.047)),
        delta_y_static_m=float(data.get("delta_y_static_m", 0.091)),
        k_sigma=float(data.get("k_sigma", 2.5)),
        min_unknown_component_pixels=int(float(data.get("min_unknown_component_pixels", 64))),
    )


def _handle(data: Mapping[str, Any]) -> MapHandle:
    shape = tuple(int(v) for v in data["shape"])
    mu = np.ones(shape, dtype=np.float64)
    pitch = np.zeros(shape, dtype=np.float64)
    return MapHandle(
        map_path=str(data.get("map_path", "")),
        meta_path=str(data.get("meta_path", "")),
        mu_grid=mu,
        pitch_grid=pitch,
        q_grid=None,
        meta=dict(data),
        resolution_m=float(data["resolution_m"]),
        x0=float(data["x0"]),
        z0=float(data["z0"]),
        target_y=float(data.get("target_y", data.get("target_y_fixed_m", 0.047))),
        tag=str(data.get("tag", "audit")),
    )


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _read_json(path: Path) -> Mapping[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    rows = list(rows)
    public_rows = [{k: v for k, v in row.items() if not str(k).startswith("_")} for row in rows]
    if not public_rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(public_rows[0].keys()))
        writer.writeheader()
        for row in public_rows:
            writer.writerow(row)


def _write_json(path: Path, data: Mapping[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(_jsonable(data), f, indent=2, ensure_ascii=False)


def _jsonable(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    return value


def _mean(values: Sequence[float]) -> float:
    return float(np.mean(np.asarray(values, dtype=np.float64))) if values else math.nan


def _nanmedian(values: np.ndarray) -> float:
    vals = np.asarray(values, dtype=np.float64)
    vals = vals[np.isfinite(vals)]
    return float(np.median(vals)) if vals.size else math.nan


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--out", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = analyze_run(run_dir=Path(args.run_dir), out_dir=None if args.out is None else Path(args.out))
    print(json.dumps(_jsonable(summary), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
