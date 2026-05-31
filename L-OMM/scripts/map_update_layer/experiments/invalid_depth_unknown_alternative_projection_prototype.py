#!/usr/bin/env python3
"""Experiment-only invalid-depth unknown projection prototype.

This script reads stored Stage A3 live-snapshot artifacts and compares
production shell projection against local-depth projection variants.  It does
not subscribe to ROS topics and does not mutate production map-update code.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any, Callable, Dict, Iterable, Mapping, Sequence

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))

from map_handle import MapHandle  # noqa: E402
from map_update_layer.active_map_snapshot import ActiveMapSnapshot  # noqa: E402
from map_update_layer.blockage_classifier import ReferenceBlockageClassifier  # noqa: E402
from map_update_layer.depth_geometry_evidence import (  # noqa: E402
    DepthGeometryConfig,
    _image_component_unknown_rect,
)
from map_update_layer.perception_to_map import CameraIntrinsics  # noqa: E402


PER_FRAME_CSV = "invalid_depth_unknown_alternative_projection_per_frame.csv"
SUMMARY_JSON = "invalid_depth_unknown_alternative_projection_summary.json"
CORRIDOR_CSV = "invalid_depth_unknown_alternative_projection_corridor_overlap.csv"
REPLAN_CSV = "invalid_depth_unknown_alternative_projection_replan_trigger.csv"
NO_COMMAND_JSON = "invalid_depth_unknown_alternative_projection_no_command.json"
SWEEP_CSV = "invalid_depth_unknown_alternative_projection_radius_sweep.csv"
TEMPORAL_JSON = "invalid_depth_unknown_alternative_projection_temporal_stability.json"

VARIANTS = ("production_shell", "min_depth_only", "median_adjacent")
Rect = tuple[float, float, float, float]


def compute_alternative_rect(
    *,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    T_base_cam: np.ndarray,
    handle: MapHandle,
    unknown_candidate_mask: np.ndarray,
    variant: str,
    adjacency_radius_px: int,
    cfg: DepthGeometryConfig,
) -> Dict[str, Any]:
    """Compute one diagnostic unknown rectangle for one frame and variant."""

    variant = str(variant)
    if variant not in VARIANTS:
        raise ValueError(f"unknown variant: {variant}")
    depth = np.asarray(depth_m, dtype=np.float64)
    if depth.ndim != 2:
        raise ValueError("depth_m must be a 2D array")
    T = np.asarray(T_base_cam, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError("T_base_cam must have shape 4x4")
    candidate = np.asarray(unknown_candidate_mask, dtype=bool)
    if candidate.shape != depth.shape:
        raise ValueError("unknown_candidate_mask shape must match depth_m")

    invalid = _invalid_component_mask(depth, candidate, intrinsics, cfg)
    v_idx, u_idx = np.nonzero(invalid)
    invalid_count = int(v_idx.size)
    base_record: Dict[str, Any] = {
        "prototype_variant": variant,
        "rect": None,
        "rect_area_m2": math.nan,
        "rect_cell_count": 0,
        "depth_used_m": math.nan,
        "adjacency_valid_pixel_count": 0,
        "fallback_to_production_shell": False,
        "invalid_pixel_count": invalid_count,
        "invalid_bbox_uv": (-1, -1, -1, -1),
    }
    if invalid_count < max(int(cfg.min_unknown_component_pixels), 1):
        return base_record

    u_min, u_max = int(np.min(u_idx)), int(np.max(u_idx))
    v_min, v_max = int(np.min(v_idx)), int(np.max(v_idx))
    base_record["invalid_bbox_uv"] = (u_min, v_min, u_max, v_max)
    production_rect = _production_shell_rect(
        handle=handle,
        u_min=u_min,
        u_max=u_max,
        v_min=v_min,
        v_max=v_max,
        intrinsics=intrinsics,
        T_base_cam=T,
        cfg=cfg,
    )
    if variant == "production_shell":
        return _with_rect(base_record, production_rect, handle)

    adjacent_depths = _adjacent_valid_depths(
        depth,
        invalid,
        u_min=u_min,
        u_max=u_max,
        v_min=v_min,
        v_max=v_max,
        radius_px=int(adjacency_radius_px),
    )
    base_record["adjacency_valid_pixel_count"] = int(adjacent_depths.size)
    if adjacent_depths.size == 0:
        base_record["fallback_to_production_shell"] = True
        return _with_rect(base_record, production_rect, handle)

    if variant == "min_depth_only":
        depth_used = float(np.min(adjacent_depths))
    else:
        depth_used = float(np.median(adjacent_depths))
    base_record["depth_used_m"] = depth_used
    rect = _single_depth_rect(
        handle=handle,
        u_min=u_min,
        u_max=u_max,
        v_min=v_min,
        v_max=v_max,
        intrinsics=intrinsics,
        T_base_cam=T,
        depth_m=depth_used,
    )
    return _with_rect(base_record, rect, handle)


def iterate_run_dir(
    *,
    run_dir: str | Path,
    variants: Sequence[str],
    adjacency_radius_px: int,
    corridor_mask_loader: Callable[[Mapping[str, Any]], set[tuple[int, int]]] | None = None,
) -> list[Dict[str, Any]]:
    """Read a Stage A3 run directory and emit per-frame prototype records."""

    run_path = Path(run_dir)
    events = _event_rows(run_path)
    rows: list[Dict[str, Any]] = []
    accepted = _load_accepted_reference(run_path)
    for event in events:
        frame_id = int(event.get("frame_id", len(rows)))
        try:
            frame_path = run_path / "frames" / str(event.get("frame_artifact", f"frame_{frame_id:04d}.npz"))
            meta_path = run_path / "frames" / str(event.get("frame_metadata_artifact", f"frame_{frame_id:04d}.json"))
            frame = np.load(frame_path, allow_pickle=False)
            metadata = _read_json(meta_path)
            depth = np.asarray(frame["depth_m"], dtype=np.float64)
            unknown_candidate = np.asarray(frame["unknown_candidate_mask"], dtype=bool)
            T_base_cam = np.asarray(frame["T_base_cam"], dtype=np.float64)
            handle = _handle(metadata["map_geometry"])
            intrinsics = _intrinsics(metadata["intrinsics"])
            cfg = _config(metadata["depth_geometry_config"])
            goal_cells = (
                corridor_mask_loader(metadata)
                if corridor_mask_loader is not None
                else _goal_cells_from_metadata(metadata)
            )
            base_feasible = np.asarray(frame["base_feasible_mask"], dtype=bool)
            original_blocked = np.asarray(frame["blocked_mask"], dtype=bool)
            original_unknown = np.asarray(
                frame["layer_unknown"] if "layer_unknown" in frame.files else np.zeros(handle.shape, dtype=bool),
                dtype=bool,
            )
            for variant in variants:
                record = compute_alternative_rect(
                    depth_m=depth,
                    intrinsics=intrinsics,
                    T_base_cam=T_base_cam,
                    handle=handle,
                    unknown_candidate_mask=unknown_candidate,
                    variant=str(variant),
                    adjacency_radius_px=int(adjacency_radius_px),
                    cfg=cfg,
                )
                rect_cells = _rect_cells(handle, record["rect"])
                prototype_blocked = _substitute_unknown_layer(
                    original_blocked=original_blocked,
                    original_unknown=original_unknown,
                    rect_cells=rect_cells,
                    shape=handle.shape,
                )
                final_active = base_feasible & (~prototype_blocked)
                event_class = _classify_event(
                    handle=handle,
                    base_feasible=base_feasible,
                    final_active=final_active,
                    blocked=prototype_blocked,
                    accepted_reference=accepted,
                    goal_cells=goal_cells,
                )
                row = _public_record(
                    frame_id=frame_id,
                    record=record,
                    handle=handle,
                    goal_cells=goal_cells,
                    rect_cells=rect_cells,
                    event_class=event_class,
                )
                rows.append(row)
        except Exception as exc:  # keep batch audits exhaustive
            for variant in variants:
                rows.append(
                    {
                        "frame_id": frame_id,
                        "prototype_variant": str(variant),
                        "prototype_error": str(exc),
                        "prototype_error_count": 1,
                    }
                )
    return rows


def run_prototype(
    *,
    run_dir: str | Path,
    stage_b_dir: str | Path | None,
    variants: Sequence[str],
    adjacency_radius_px: int,
    out_dir: str | Path,
    sweep_radii: Sequence[int] = (),
) -> Dict[str, Any]:
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    rows = iterate_run_dir(
        run_dir=run_dir,
        variants=variants,
        adjacency_radius_px=int(adjacency_radius_px),
    )
    _write_csv(out_path / PER_FRAME_CSV, rows)
    _write_csv(out_path / CORRIDOR_CSV, _corridor_rows(rows))
    replan_rows = _replan_trigger_rows(rows)
    _write_csv(out_path / REPLAN_CSV, replan_rows)
    summary = _summary(rows, stage_b_dir=Path(stage_b_dir) if stage_b_dir else None)
    summary["adjacency_radius_px"] = int(adjacency_radius_px)
    summary["run_dir"] = str(run_dir)
    summary["out_dir"] = str(out_path)
    _write_json(out_path / SUMMARY_JSON, summary)
    temporal = _temporal(rows)
    _write_json(out_path / TEMPORAL_JSON, temporal)
    no_command = {
        "robot_command_endpoint_touched": False,
        "control_module_called_in_live_loop": False,
        "prototype_modifies_production": False,
        "live_capture_performed": False,
        "artifact_read_only": True,
    }
    _write_json(out_path / NO_COMMAND_JSON, no_command)
    if sweep_radii:
        sweep_rows = []
        for radius in sweep_radii:
            radius_rows = iterate_run_dir(
                run_dir=run_dir,
                variants=variants,
                adjacency_radius_px=int(radius),
            )
            for variant, stats in _summary(radius_rows, stage_b_dir=None)["per_variant"].items():
                sweep_rows.append(
                    {
                        "adjacency_radius_px": int(radius),
                        "prototype_variant": variant,
                        "fallback_rate": stats["fallback_rate"],
                        "rect_none_rate": stats["rect_none_rate"],
                        "goal_corridor_overlap_cells_mean": stats["goal_corridor_overlap_cells"]["mean"],
                        "rect_cell_count_mean": stats["rect_cell_count"]["mean"],
                    }
                )
        _write_csv(out_path / SWEEP_CSV, sweep_rows)
    return summary


def _invalid_component_mask(
    depth: np.ndarray,
    unknown_candidate: np.ndarray,
    intrinsics: CameraIntrinsics,
    cfg: DepthGeometryConfig,
) -> np.ndarray:
    h, w = depth.shape
    uu = np.arange(w, dtype=np.float64)[None, :]
    band = np.broadcast_to(
        np.abs(uu - float(intrinsics.cx)) <= float(cfg.pixel_band_half_width_px),
        (h, w),
    )
    finite_positive = np.isfinite(depth) & (depth > 0.0)
    return band & (~finite_positive) & np.asarray(unknown_candidate, dtype=bool)


def _production_shell_rect(
    *,
    handle: MapHandle,
    u_min: int,
    u_max: int,
    v_min: int,
    v_max: int,
    intrinsics: CameraIntrinsics,
    T_base_cam: np.ndarray,
    cfg: DepthGeometryConfig,
) -> Rect | None:
    return _image_component_unknown_rect(
        handle=handle,
        u_min=int(u_min),
        u_max=int(u_max),
        v_min=int(v_min),
        v_max=int(v_max),
        intrinsics=intrinsics,
        T_base_cam=np.asarray(T_base_cam, dtype=np.float64),
        depth_min_m=float(cfg.depth_min_m),
        depth_max_m=float(cfg.depth_max_m),
    )


def _single_depth_rect(
    *,
    handle: MapHandle,
    u_min: int,
    u_max: int,
    v_min: int,
    v_max: int,
    intrinsics: CameraIntrinsics,
    T_base_cam: np.ndarray,
    depth_m: float,
) -> Rect | None:
    samples = np.asarray(
        [(float(u), float(v), float(depth_m)) for u in (u_min, u_max) for v in (v_min, v_max)],
        dtype=np.float64,
    )
    points_cam = _backproject(samples[:, 0], samples[:, 1], samples[:, 2], intrinsics)
    points_base = (T_base_cam[:3, :3] @ points_cam.T).T + T_base_cam[:3, 3][None, :]
    return _clip_rect(
        handle,
        float(np.min(points_base[:, 0])),
        float(np.max(points_base[:, 0])),
        float(np.min(points_base[:, 2])),
        float(np.max(points_base[:, 2])),
    )


def _adjacent_valid_depths(
    depth: np.ndarray,
    invalid: np.ndarray,
    *,
    u_min: int,
    u_max: int,
    v_min: int,
    v_max: int,
    radius_px: int,
) -> np.ndarray:
    h, w = depth.shape
    r = max(int(radius_px), 0)
    u0, u1 = max(0, int(u_min) - r), min(w - 1, int(u_max) + r)
    v0, v1 = max(0, int(v_min) - r), min(h - 1, int(v_max) + r)
    border = np.zeros_like(invalid, dtype=bool)
    border[v0 : v1 + 1, u0 : u1 + 1] = True
    border[v_min : v_max + 1, u_min : u_max + 1] = False
    vals = np.asarray(depth[border], dtype=np.float64)
    return vals[np.isfinite(vals) & (vals > 0.0)]


def _with_rect(record: Dict[str, Any], rect: Rect | None, handle: MapHandle) -> Dict[str, Any]:
    out = dict(record)
    out["rect"] = None if rect is None else tuple(float(v) for v in rect)
    out["rect_area_m2"] = _rect_area(rect)
    out["rect_cell_count"] = len(_rect_cells(handle, rect))
    return out


def _public_record(
    *,
    frame_id: int,
    record: Mapping[str, Any],
    handle: MapHandle,
    goal_cells: set[tuple[int, int]],
    rect_cells: set[tuple[int, int]],
    event_class: str,
) -> Dict[str, Any]:
    rect = record.get("rect")
    bbox = tuple(record.get("invalid_bbox_uv", (-1, -1, -1, -1)))
    xl = zt = xr = zb = math.nan
    if rect is not None:
        xl, zt, xr, zb = (float(v) for v in rect)
    overlap = len(goal_cells & rect_cells)
    return {
        "frame_id": int(frame_id),
        "prototype_variant": str(record["prototype_variant"]),
        "rect_xl": float(xl),
        "rect_zt": float(zt),
        "rect_xr": float(xr),
        "rect_zb": float(zb),
        "rect_area_m2": float(record["rect_area_m2"]),
        "rect_cell_count": int(record["rect_cell_count"]),
        "depth_used_m": float(record["depth_used_m"]),
        "adjacency_valid_pixel_count": int(record["adjacency_valid_pixel_count"]),
        "fallback_to_production_shell": bool(record["fallback_to_production_shell"]),
        "invalid_pixel_count": int(record["invalid_pixel_count"]),
        "invalid_bbox_u_min": int(bbox[0]),
        "invalid_bbox_v_min": int(bbox[1]),
        "invalid_bbox_u_max": int(bbox[2]),
        "invalid_bbox_v_max": int(bbox[3]),
        "goal_corridor_overlap_cells": int(overlap),
        "event_class": str(event_class),
        "prototype_error_count": 0,
    }


def _substitute_unknown_layer(
    *,
    original_blocked: np.ndarray,
    original_unknown: np.ndarray,
    rect_cells: set[tuple[int, int]],
    shape: tuple[int, int],
) -> np.ndarray:
    blocked = np.asarray(original_blocked, dtype=bool).copy()
    unknown = np.asarray(original_unknown, dtype=bool)
    if unknown.shape == blocked.shape:
        blocked[unknown] = False
    for ix, iz in rect_cells:
        if 0 <= ix < shape[0] and 0 <= iz < shape[1]:
            blocked[ix, iz] = True
    return blocked


def _classify_event(
    *,
    handle: MapHandle,
    base_feasible: np.ndarray,
    final_active: np.ndarray,
    blocked: np.ndarray,
    accepted_reference: Mapping[str, np.ndarray] | None,
    goal_cells: set[tuple[int, int]],
) -> str:
    if not accepted_reference:
        return "GOAL_CORRIDOR_BLOCKED" if np.any([blocked[ix, iz] for ix, iz in goal_cells]) else "NO_RELEVANT_CHANGE"
    snapshot = ActiveMapSnapshot(
        handle=handle,
        base_feasible_mask=base_feasible,
        final_active_mask=final_active,
        blocked_mask=blocked,
        occupied_mask=np.zeros(handle.shape, dtype=bool),
        stats={"source": "alternative_projection_prototype"},
    )
    s = np.asarray(accepted_reference["s"], dtype=np.float64).reshape(-1)
    xz = np.asarray(accepted_reference["xz"], dtype=np.float64).reshape(-1, 2)
    goal_xz = tuple(float(v) for v in xz[-1]) if xz.size else None
    report = ReferenceBlockageClassifier().classify(
        snapshot,
        s_table=s,
        xz_table=xz,
        current_s_m=0.0,
        previous_blocked_mask=None,
        goal_xz=goal_xz,
    )
    return str(report.event_class)


def _event_rows(run_path: Path) -> list[dict[str, str]]:
    events_path = run_path / "events.csv"
    if events_path.exists():
        with events_path.open("r", newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if rows:
            return rows
    frames = sorted((run_path / "frames").glob("frame_*.npz"))
    return [
        {
            "frame_id": str(idx),
            "frame_artifact": path.name,
            "frame_metadata_artifact": path.with_suffix(".json").name,
        }
        for idx, path in enumerate(frames)
    ]


def _goal_cells_from_metadata(metadata: Mapping[str, Any]) -> set[tuple[int, int]]:
    return {
        (int(cell[0]), int(cell[1]))
        for cell in metadata.get("goal_diagnostics", {}).get("goal_corridor_cells", [])
    }


def _load_accepted_reference(run_path: Path) -> Dict[str, np.ndarray] | None:
    path = run_path / "accepted_reference.npz"
    if not path.exists():
        return None
    with np.load(path, allow_pickle=False) as data:
        if "s" not in data.files or "xz" not in data.files:
            return None
        return {"s": np.asarray(data["s"], dtype=np.float64), "xz": np.asarray(data["xz"], dtype=np.float64)}


def _summary(rows: Sequence[Mapping[str, Any]], *, stage_b_dir: Path | None) -> Dict[str, Any]:
    variants = sorted({str(r.get("prototype_variant", "")) for r in rows if r.get("prototype_variant")})
    per_variant = {}
    for variant in variants:
        subset = [r for r in rows if str(r.get("prototype_variant")) == variant]
        per_variant[variant] = {
            "rect_area_m2": _stats(subset, "rect_area_m2"),
            "rect_cell_count": _stats(subset, "rect_cell_count"),
            "goal_corridor_overlap_cells": _stats(subset, "goal_corridor_overlap_cells"),
            "depth_used_m": _stats(subset, "depth_used_m"),
            "fallback_rate": _rate(subset, "fallback_to_production_shell"),
            "rect_none_rate": float(sum(_is_nan(r.get("rect_area_m2", math.nan)) for r in subset) / max(len(subset), 1)),
            "prototype_error_count": int(sum(int(r.get("prototype_error_count", 0)) for r in subset)),
            "event_counts": _event_counts(subset),
        }
    summary: Dict[str, Any] = {
        "frames_analyzed": len({int(r.get("frame_id", -1)) for r in rows if int(r.get("frame_id", -1)) >= 0}),
        "row_count": int(len(rows)),
        "per_variant": per_variant,
        "prototype_error_count": int(sum(int(r.get("prototype_error_count", 0)) for r in rows)),
    }
    if stage_b_dir is not None:
        stage_b_summary = stage_b_dir / "invalid_depth_unknown_geometry_summary.json"
        if stage_b_summary.exists() and "production_shell" in per_variant:
            data = _read_json(stage_b_summary)
            stage_b_prod = float(data.get("goal_corridor_overlap_prod", {}).get("mean", math.nan))
            proto_prod = float(per_variant["production_shell"]["goal_corridor_overlap_cells"]["mean"])
            summary["production_shell_stage_b_overlap_mean_delta"] = proto_prod - stage_b_prod
            summary["stage_b_goal_corridor_overlap_prod_mean"] = stage_b_prod
    return summary


def _corridor_rows(rows: Sequence[Mapping[str, Any]]) -> list[Dict[str, Any]]:
    return [
        {
            "frame_id": r.get("frame_id"),
            "prototype_variant": r.get("prototype_variant"),
            "goal_corridor_overlap_cells": r.get("goal_corridor_overlap_cells", 0),
            "rect_cell_count": r.get("rect_cell_count", 0),
            "fallback_to_production_shell": r.get("fallback_to_production_shell", False),
        }
        for r in rows
    ]


def _replan_trigger_rows(rows: Sequence[Mapping[str, Any]]) -> list[Dict[str, Any]]:
    output = []
    variants = sorted({str(r.get("prototype_variant", "")) for r in rows if r.get("prototype_variant")})
    for variant in variants:
        subset = [r for r in rows if str(r.get("prototype_variant")) == variant]
        counts = _event_counts(subset)
        output.append(
            {
                "prototype_variant": variant,
                "GOAL_CORRIDOR_BLOCKED": int(counts.get("GOAL_CORRIDOR_BLOCKED", 0)),
                "REFERENCE_BLOCKED": int(counts.get("REFERENCE_BLOCKED", 0)),
                "NO_RELEVANT_CHANGE": int(counts.get("NO_RELEVANT_CHANGE", 0)),
                "CURRENT_POSE_UNSAFE": int(counts.get("CURRENT_POSE_UNSAFE", 0)),
                "MASK_CHANGED_NONCRITICAL": int(counts.get("MASK_CHANGED_NONCRITICAL", 0)),
                "total_frames": len(subset),
            }
        )
    return output


def _temporal(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    output = {}
    variants = sorted({str(r.get("prototype_variant", "")) for r in rows if r.get("prototype_variant")})
    for variant in variants:
        subset = [r for r in rows if str(r.get("prototype_variant")) == variant]
        centers_x = []
        centers_z = []
        for row in subset:
            try:
                xl, xr = float(row["rect_xl"]), float(row["rect_xr"])
                zt, zb = float(row["rect_zt"]), float(row["rect_zb"])
                if all(np.isfinite(v) for v in (xl, xr, zt, zb)):
                    centers_x.append(0.5 * (xl + xr))
                    centers_z.append(0.5 * (zt + zb))
            except Exception:
                continue
        output[variant] = {
            "prototype_rect_center_x_std_m": _std_values(centers_x),
            "prototype_rect_center_z_std_m": _std_values(centers_z),
            "prototype_rect_cell_count_std": _std(subset, "rect_cell_count"),
            "goal_corridor_overlap_prototype_std": _std(subset, "goal_corridor_overlap_cells"),
        }
    return output


def _event_counts(rows: Sequence[Mapping[str, Any]]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for row in rows:
        event = str(row.get("event_class", ""))
        if event:
            counts[event] = counts.get(event, 0) + 1
    return counts


def _stats(rows: Sequence[Mapping[str, Any]], key: str) -> Dict[str, float]:
    vals = []
    for row in rows:
        try:
            value = float(row.get(key, math.nan))
        except Exception:
            value = math.nan
        if np.isfinite(value):
            vals.append(value)
    if not vals:
        return {"min": math.nan, "max": math.nan, "mean": math.nan, "std": math.nan}
    arr = np.asarray(vals, dtype=np.float64)
    return {
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
    }


def _std(rows: Sequence[Mapping[str, Any]], key: str) -> float:
    return _stats(rows, key)["std"]


def _std_values(values: Sequence[float]) -> float:
    vals = np.asarray([float(v) for v in values if np.isfinite(float(v))], dtype=np.float64)
    return float(np.std(vals)) if vals.size else math.nan


def _rate(rows: Sequence[Mapping[str, Any]], key: str) -> float:
    return float(sum(bool(r.get(key, False)) for r in rows) / max(len(rows), 1))


def _rect_area(rect: Rect | None) -> float:
    if rect is None:
        return math.nan
    xl, zt, xr, zb = rect
    return max(0.0, float(xr) - float(xl)) * max(0.0, float(zt) - float(zb))


def _rect_cells(handle: MapHandle, rect: Rect | None) -> set[tuple[int, int]]:
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


def _backproject(u: np.ndarray, v: np.ndarray, depth_m: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray:
    x = (u - float(intrinsics.cx)) * depth_m / float(intrinsics.fx)
    y = (v - float(intrinsics.cy)) * depth_m / float(intrinsics.fy)
    z = depth_m
    return np.stack([x, y, z], axis=1)


def _clip_rect(handle: MapHandle, x0: float, x1: float, z0: float, z1: float) -> Rect | None:
    xl = max(min(x0, x1), float(handle.x0))
    xr = min(max(x0, x1), float(handle.x0) + float(handle.shape[0] - 1) * float(handle.resolution_m))
    zb = max(min(z0, z1), float(handle.z0))
    zt = min(max(z0, z1), float(handle.z0) + float(handle.shape[1] - 1) * float(handle.resolution_m))
    if xr < xl or zt < zb:
        return None
    return float(xl), float(zt), float(xr), float(zb)


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
    return MapHandle(
        map_path=str(data.get("map_path", "")),
        meta_path=str(data.get("meta_path", "")),
        mu_grid=np.ones(shape, dtype=np.float64),
        pitch_grid=np.zeros(shape, dtype=np.float64),
        q_grid=None,
        meta=dict(data),
        resolution_m=float(data["resolution_m"]),
        x0=float(data["x0"]),
        z0=float(data["z0"]),
        target_y=float(data.get("target_y", data.get("target_y_fixed_m", 0.047))),
        tag=str(data.get("tag", "prototype")),
    )


def _read_json(path: Path) -> Mapping[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(str(key))
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, "") for name in fieldnames})


def _write_json(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
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


def _is_nan(value: Any) -> bool:
    try:
        return not np.isfinite(float(value))
    except Exception:
        return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--stage-b-dir", default=None)
    parser.add_argument("--variants", nargs="+", default=list(VARIANTS))
    parser.add_argument("--adjacency-radius-px", type=int, default=5)
    parser.add_argument("--sweep-radii", nargs="*", type=int, default=[])
    parser.add_argument("--out", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = run_prototype(
        run_dir=Path(args.run_dir),
        stage_b_dir=None if args.stage_b_dir is None else Path(args.stage_b_dir),
        variants=[str(v) for v in args.variants],
        adjacency_radius_px=int(args.adjacency_radius_px),
        out_dir=Path(args.out),
        sweep_radii=[int(v) for v in args.sweep_radii],
    )
    print(json.dumps(_jsonable(summary), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
