#!/usr/bin/env python3
"""Experiment-only hardening audit for invalid-depth unknown projection.

This script reuses stored Stage A3 artifacts.  It does not subscribe to ROS
topics, does not command hardware, and does not modify production modules.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, Mapping, Sequence

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
EXPERIMENTS_ROOT = SCRIPTS_ROOT / "map_update_layer" / "experiments"
if str(EXPERIMENTS_ROOT) not in sys.path:
    sys.path.insert(0, str(EXPERIMENTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))

from capsule_collision import CapsuleCollision  # noqa: E402
from map_handle import MapHandle  # noqa: E402
from map_update_layer.active_map_snapshot import ActiveMapSnapshot  # noqa: E402
from map_update_layer.blockage_classifier import ReferenceBlockageClassifier  # noqa: E402
from map_update_layer.path_collision_monitor import PathCollisionMonitor  # noqa: E402

from invalid_depth_unknown_alternative_projection_prototype import (  # noqa: E402
    VARIANTS,
    _config,
    _event_rows,
    _goal_cells_from_metadata,
    _handle,
    _intrinsics,
    _jsonable,
    _read_json,
    _rect_cells,
    _stats,
    _write_csv,
    _write_json,
    compute_alternative_rect,
)


AUDIT_CSV = "classifier_reconstruction_audit.csv"
AUDIT_JSON = "classifier_reconstruction_audit.json"
HARDENED_REPLAN_CSV = "hardened_replan_trigger.csv"
HARDENED_REASONS_CSV = "hardened_replan_trigger_reasons.csv"
HARDENED_SUMMARY_JSON = "hardened_summary.json"
HARDENED_PER_FRAME_CSV = "hardened_per_frame.csv"
HARDENED_NO_COMMAND_JSON = "hardened_no_command.json"
REFERENCE_ALIGNMENT_AUDIT_CSV = "reference_alignment_audit.csv"
REFERENCE_ALIGNMENT_AUDIT_JSON = "reference_alignment_audit.json"
ALIGNED_PER_FRAME_CSV = "aligned_hardened_per_frame.csv"
ALIGNED_SUMMARY_JSON = "aligned_hardened_summary.json"
ALIGNED_REPLAN_CSV = "aligned_replan_trigger.csv"
ALIGNED_REASONS_CSV = "aligned_replan_trigger_reasons.csv"
ALIGNED_NO_COMMAND_JSON = "aligned_no_command.json"

REASON_CODES = (
    "true_capsule_future_reference_collision",
    "goal_corridor_neighborhood_loss",
    "substitution_artifact_corridor_radius_fallback",
    "substitution_artifact_non_unknown_removal",
    "classifier_future_slice_fallback",
    "path_collision_monitor_unavailable",
    "no_replan_trigger",
)
SUBSTITUTION_SCOPES = ("full_unknown_layer", "rect_specific")
ACCEPTED_REFERENCE_MODES = ("stored_npz", "episode_resampled", "q_grid_reconstructed")


@dataclass(frozen=True)
class SubstitutionResult:
    blocked: np.ndarray
    substitution_scope: str
    removed_unknown_cells: int
    removed_non_unknown_cells: int
    added_only_prototype_cells: int


@dataclass(frozen=True)
class AcceptedReferenceModeResult:
    accepted_reference: Dict[str, np.ndarray]
    mode: str
    q_alignment_status: str
    q_samples_before: int
    q_samples_after: int
    expected_future_slice_fallback: bool


def substitute_unknown_layer_hardened(
    *,
    original_blocked: np.ndarray,
    original_unknown: np.ndarray,
    prototype_rect_cells: set[tuple[int, int]],
    production_shell_rect_cells: set[tuple[int, int]],
    shape: tuple[int, int],
    substitution_scope: str,
) -> SubstitutionResult:
    """Replace the production unknown shell with an experiment-only prototype.

    ``full_unknown_layer`` preserves the Stage C behavior.  ``rect_specific``
    removes only the production shell cells being replaced, preventing unrelated
    unknown cells from being erased by the diagnostic substitution itself.
    """

    scope = str(substitution_scope)
    if scope not in SUBSTITUTION_SCOPES:
        raise ValueError(f"unknown substitution_scope: {scope}")

    blocked0 = np.asarray(original_blocked, dtype=bool)
    unknown = np.asarray(original_unknown, dtype=bool)
    blocked = blocked0.copy()
    if scope == "full_unknown_layer" and unknown.shape == blocked.shape:
        remove_cells = {(int(ix), int(iz)) for ix, iz in np.argwhere(unknown)}
    else:
        remove_cells = set(production_shell_rect_cells)

    removed_unknown = 0
    removed_non_unknown = 0
    for ix, iz in remove_cells:
        if 0 <= ix < shape[0] and 0 <= iz < shape[1] and bool(blocked[ix, iz]):
            if unknown.shape == blocked.shape and bool(unknown[ix, iz]):
                removed_unknown += 1
            else:
                removed_non_unknown += 1
            blocked[ix, iz] = False

    added_only = 0
    for ix, iz in prototype_rect_cells:
        if 0 <= ix < shape[0] and 0 <= iz < shape[1]:
            if not bool(blocked[ix, iz]):
                added_only += 1
            blocked[ix, iz] = True

    return SubstitutionResult(
        blocked=blocked,
        substitution_scope=scope,
        removed_unknown_cells=int(removed_unknown),
        removed_non_unknown_cells=int(removed_non_unknown),
        added_only_prototype_cells=int(added_only),
    )


def run_hardening(
    *,
    run_dir: str | Path,
    stage_b_dir: str | Path | None,
    stage_c_dir: str | Path | None,
    variants: Sequence[str],
    substitution_scope: str,
    adjacency_radius_px: int,
    enable_path_collision_monitor: bool,
    accepted_reference_modes: Sequence[str] = ("stored_npz",),
    out_dir: str | Path,
) -> Dict[str, Any]:
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    modes = [str(m) for m in accepted_reference_modes]
    for mode in modes:
        if mode not in ACCEPTED_REFERENCE_MODES:
            raise ValueError(f"unknown accepted_reference_mode: {mode}")

    if modes != ["stored_npz"]:
        audit = audit_reference_alignment(Path(run_dir))
        _write_csv(out_path / REFERENCE_ALIGNMENT_AUDIT_CSV, [audit])
        _write_json(out_path / REFERENCE_ALIGNMENT_AUDIT_JSON, audit)
        aligned_rows = []
        for mode in modes:
            aligned_rows.extend(
                iterate_hardening_rows(
                    run_dir=Path(run_dir),
                    variants=variants,
                    adjacency_radius_px=int(adjacency_radius_px),
                    substitution_scope=str(substitution_scope),
                    enable_path_collision_monitor=bool(enable_path_collision_monitor),
                    accepted_reference_mode=mode,
                )
            )
        _write_csv(out_path / ALIGNED_PER_FRAME_CSV, aligned_rows)
        _write_csv(out_path / ALIGNED_REPLAN_CSV, _replan_trigger_rows(aligned_rows, group_keys=("accepted_reference_mode", "prototype_variant")))
        _write_csv(out_path / ALIGNED_REASONS_CSV, _reason_rows(aligned_rows, group_keys=("accepted_reference_mode", "prototype_variant")))
        summary = _aligned_summary(
            rows=aligned_rows,
            run_dir=Path(run_dir),
            stage_b_dir=None if stage_b_dir is None else Path(stage_b_dir),
            stage_c_dir=None if stage_c_dir is None else Path(stage_c_dir),
            audit=audit,
            accepted_reference_modes=modes,
            substitution_scope=str(substitution_scope),
            enable_path_collision_monitor=bool(enable_path_collision_monitor),
            adjacency_radius_px=int(adjacency_radius_px),
        )
        _write_json(out_path / ALIGNED_SUMMARY_JSON, summary)
        _write_json(out_path / ALIGNED_NO_COMMAND_JSON, _no_command_record())
        return summary

    rows_by_scope = {
        scope: iterate_hardening_rows(
            run_dir=Path(run_dir),
            variants=variants,
            adjacency_radius_px=int(adjacency_radius_px),
            substitution_scope=scope,
            enable_path_collision_monitor=bool(enable_path_collision_monitor),
            accepted_reference_mode="stored_npz",
        )
        for scope in SUBSTITUTION_SCOPES
    }
    audit_rows = rows_by_scope["full_unknown_layer"]
    hardened_rows = rows_by_scope[str(substitution_scope)]

    _write_csv(out_path / AUDIT_CSV, audit_rows)
    _write_json(out_path / AUDIT_JSON, _audit_json(audit_rows))
    _write_csv(out_path / HARDENED_PER_FRAME_CSV, hardened_rows)
    _write_csv(out_path / HARDENED_REPLAN_CSV, _replan_trigger_rows(hardened_rows))
    _write_csv(out_path / HARDENED_REASONS_CSV, _reason_rows(hardened_rows))

    summary = _summary(
        rows_by_scope=rows_by_scope,
        requested_scope=str(substitution_scope),
        run_dir=Path(run_dir),
        stage_b_dir=None if stage_b_dir is None else Path(stage_b_dir),
        stage_c_dir=None if stage_c_dir is None else Path(stage_c_dir),
        enable_path_collision_monitor=bool(enable_path_collision_monitor),
        adjacency_radius_px=int(adjacency_radius_px),
    )
    _write_json(out_path / HARDENED_SUMMARY_JSON, summary)
    _write_json(
        out_path / HARDENED_NO_COMMAND_JSON,
        _no_command_record(),
    )
    return summary


def iterate_hardening_rows(
    *,
    run_dir: Path,
    variants: Sequence[str],
    adjacency_radius_px: int,
    substitution_scope: str,
    enable_path_collision_monitor: bool,
    accepted_reference_mode: str = "stored_npz",
) -> list[Dict[str, Any]]:
    rows: list[Dict[str, Any]] = []
    events = _event_rows(run_dir)
    accepted = _load_accepted_reference(run_dir)
    episode_q = _load_episode_q_traj(run_dir / "initial_control" / "episode.csv")
    for event in events:
        frame_id = int(event.get("frame_id", len(rows)))
        try:
            frame_path = run_dir / "frames" / str(event.get("frame_artifact", f"frame_{frame_id:04d}.npz"))
            meta_path = run_dir / "frames" / str(event.get("frame_metadata_artifact", f"frame_{frame_id:04d}.json"))
            frame = np.load(frame_path, allow_pickle=False)
            metadata = _read_json(meta_path)
            depth = np.asarray(frame["depth_m"], dtype=np.float64)
            unknown_candidate = np.asarray(frame["unknown_candidate_mask"], dtype=bool)
            T_base_cam = np.asarray(frame["T_base_cam"], dtype=np.float64)
            handle = _handle(metadata["map_geometry"])
            intrinsics = _intrinsics(metadata["intrinsics"])
            cfg = _config(metadata["depth_geometry_config"])
            goal_cells = _goal_cells_from_metadata(metadata)
            base_feasible = np.asarray(frame["base_feasible_mask"], dtype=bool)
            original_blocked = np.asarray(frame["blocked_mask"], dtype=bool)
            original_unknown = np.asarray(
                frame["layer_unknown"] if "layer_unknown" in frame.files else np.zeros(handle.shape, dtype=bool),
                dtype=bool,
            )
            production_record = compute_alternative_rect(
                depth_m=depth,
                intrinsics=intrinsics,
                T_base_cam=T_base_cam,
                handle=handle,
                unknown_candidate_mask=unknown_candidate,
                variant="production_shell",
                adjacency_radius_px=int(adjacency_radius_px),
                cfg=cfg,
            )
            production_shell_cells = _rect_cells(handle, production_record["rect"])
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
                prototype_cells = _rect_cells(handle, record["rect"])
                subst = substitute_unknown_layer_hardened(
                    original_blocked=original_blocked,
                    original_unknown=original_unknown,
                    prototype_rect_cells=prototype_cells,
                    production_shell_rect_cells=production_shell_cells,
                    shape=handle.shape,
                    substitution_scope=str(substitution_scope),
                )
                final_active = base_feasible & (~subst.blocked)
                report = _classify_report(
                    handle=handle,
                    base_feasible=base_feasible,
                    final_active=final_active,
                    blocked=subst.blocked,
                    accepted_reference=accepted,
                    goal_cells=goal_cells,
                    enable_path_collision_monitor=bool(enable_path_collision_monitor),
                    accepted_reference_mode=str(accepted_reference_mode),
                    episode_q_traj=episode_q,
                )
                rows.append(
                    _public_row(
                        frame_id=frame_id,
                        record=record,
                        handle=handle,
                        prototype_cells=prototype_cells,
                        production_shell_cells=production_shell_cells,
                        goal_cells=goal_cells,
                        base_feasible=base_feasible,
                        final_active=final_active,
                        subst=subst,
                        report=report,
                        path_collision_monitor_requested=bool(enable_path_collision_monitor),
                    )
                )
        except Exception as exc:
            for variant in variants:
                rows.append(
                    {
                        "frame_id": frame_id,
                        "prototype_variant": str(variant),
                        "substitution_scope": str(substitution_scope),
                        "accepted_reference_mode": str(accepted_reference_mode),
                        "prototype_error_count": 1,
                        "prototype_error": str(exc),
                    }
                )
    return rows


def _classify_report(
    *,
    handle: MapHandle,
    base_feasible: np.ndarray,
    final_active: np.ndarray,
    blocked: np.ndarray,
    accepted_reference: Mapping[str, np.ndarray] | None,
    goal_cells: set[tuple[int, int]],
    enable_path_collision_monitor: bool,
    accepted_reference_mode: str,
    episode_q_traj: np.ndarray | None,
) -> Mapping[str, Any]:
    if not accepted_reference:
        overlap = int(sum(1 for ix, iz in goal_cells if bool(blocked[ix, iz])))
        return {
            "event_class": "GOAL_CORRIDOR_BLOCKED" if overlap else "NO_RELEVANT_CHANGE",
            "reference_blocked_source": "none",
            "reference_future_slice_fallback": False,
            "path_collision_monitor_enabled": False,
            "path_collision_is_collision": False,
            "path_collision_colliding_cells_count": 0,
            "path_collision_checked_sample_count": 0,
            "goal_feasible_neighborhood_count": 0 if overlap else 1,
            "goal_connectivity_ok": not bool(overlap),
            "reference_blocked_count": 0,
            "goal_blocked_count": overlap,
            "current_pose_blocked_count": 0,
            "mask_changed_cell_count": 0,
            "accepted_reference_mode": str(accepted_reference_mode),
            "q_alignment_status": "accepted_reference_missing",
            "q_samples_before": 0,
            "q_samples_after": 0,
            "expected_future_slice_fallback": True,
        }
    snapshot = ActiveMapSnapshot(
        handle=handle,
        base_feasible_mask=base_feasible,
        final_active_mask=final_active,
        blocked_mask=blocked,
        occupied_mask=np.zeros(handle.shape, dtype=bool),
        stats={"source": "invalid_depth_unknown_alternative_projection_hardening"},
    )
    s = np.asarray(accepted_reference["s"], dtype=np.float64).reshape(-1)
    xz = np.asarray(accepted_reference["xz"], dtype=np.float64).reshape(-1, 2)
    goal_xz = tuple(float(v) for v in xz[-1]) if xz.size else None
    mode_result = build_accepted_reference_for_mode(
        accepted_reference=accepted_reference,
        accepted_reference_mode=str(accepted_reference_mode),
        episode_q_traj=episode_q_traj,
        handle=handle,
    )
    monitor = PathCollisionMonitor(CapsuleCollision(), handle=handle) if enable_path_collision_monitor else None
    classifier = ReferenceBlockageClassifier(path_collision_monitor=monitor)
    report = classifier.classify(
        snapshot,
        s_table=s,
        xz_table=xz,
        current_s_m=0.0,
        previous_blocked_mask=None,
        goal_xz=goal_xz,
        accepted_reference=mode_result.accepted_reference,
    )
    out = report.as_dict()
    out.update(
        {
            "accepted_reference_mode": mode_result.mode,
            "q_alignment_status": mode_result.q_alignment_status,
            "q_samples_before": mode_result.q_samples_before,
            "q_samples_after": mode_result.q_samples_after,
            "expected_future_slice_fallback": mode_result.expected_future_slice_fallback,
        }
    )
    return out


def _accepted_for_classifier(accepted_reference: Mapping[str, np.ndarray]) -> Dict[str, np.ndarray]:
    out: Dict[str, np.ndarray] = {"xz_path": np.asarray(accepted_reference["xz"], dtype=np.float64)}
    for key in ("q_traj", "q_samples", "q_active_samples"):
        if key in accepted_reference:
            out[key] = np.asarray(accepted_reference[key], dtype=np.float64)
            break
    return out


def build_accepted_reference_for_mode(
    *,
    accepted_reference: Mapping[str, np.ndarray],
    accepted_reference_mode: str,
    episode_q_traj: np.ndarray | None,
    handle: MapHandle | None,
) -> AcceptedReferenceModeResult:
    mode = str(accepted_reference_mode)
    if mode not in ACCEPTED_REFERENCE_MODES:
        raise ValueError(f"unknown accepted_reference_mode: {mode}")
    s = np.asarray(accepted_reference["s"], dtype=np.float64).reshape(-1)
    xz = np.asarray(accepted_reference["xz"], dtype=np.float64).reshape(-1, 2)
    expected_len = int(s.size)
    base: Dict[str, np.ndarray] = {"xz_path": xz}

    if mode == "stored_npz":
        q = _first_q_array(accepted_reference)
        if q is None:
            return AcceptedReferenceModeResult(base, mode, "stored_q_missing", 0, 0, False)
        q_arr = np.asarray(q, dtype=np.float64).reshape(-1, 3)
        base["q_traj"] = q_arr
        status = "aligned_to_reference_support" if q_arr.shape[0] == expected_len else "stored_q_length_mismatch"
        return AcceptedReferenceModeResult(
            base,
            mode,
            status,
            int(q_arr.shape[0]),
            int(q_arr.shape[0]),
            bool(q_arr.shape[0] != expected_len),
        )

    if mode == "episode_resampled":
        if episode_q_traj is None or np.asarray(episode_q_traj).size == 0:
            return AcceptedReferenceModeResult(base, mode, "episode_q_missing", 0, 0, True)
        q_arr = np.asarray(episode_q_traj, dtype=np.float64).reshape(-1, 3)
        aligned = _resample_q_traj_to_reference(q_arr, reference_len=expected_len)
        base["q_traj"] = aligned
        return AcceptedReferenceModeResult(
            base,
            mode,
            "resampled_to_reference_support",
            int(q_arr.shape[0]),
            int(aligned.shape[0]),
            False,
        )

    if handle is None or getattr(handle, "q_grid", None) is None:
        return AcceptedReferenceModeResult(base, mode, "q_grid_unavailable", 0, 0, True)
    q_grid = np.asarray(handle.q_grid, dtype=np.float64)
    q_samples = []
    kept_xz = []
    for x, z in xz:
        ix = int(round((float(x) - float(handle.x0)) / float(handle.resolution_m)))
        iz = int(round((float(z) - float(handle.z0)) / float(handle.resolution_m)))
        if 0 <= ix < q_grid.shape[0] and 0 <= iz < q_grid.shape[1]:
            q = np.asarray(q_grid[ix, iz], dtype=np.float64).reshape(-1)
            if q.size == 3 and np.all(np.isfinite(q)):
                q_samples.append(q)
                kept_xz.append((float(x), float(z)))
    if len(q_samples) != expected_len:
        return AcceptedReferenceModeResult(base, mode, "q_grid_partial_or_empty", int(len(q_samples)), int(len(q_samples)), True)
    base["xz_path"] = np.asarray(kept_xz, dtype=np.float64).reshape(-1, 2)
    base["q_traj"] = np.asarray(q_samples, dtype=np.float64).reshape(-1, 3)
    return AcceptedReferenceModeResult(base, mode, "q_grid_reconstructed_to_reference_support", expected_len, expected_len, False)


def _first_q_array(accepted_reference: Mapping[str, np.ndarray]) -> np.ndarray | None:
    for key in ("q_traj", "q_samples", "q_active_samples"):
        if key in accepted_reference:
            return np.asarray(accepted_reference[key], dtype=np.float64)
    return None


def _resample_q_traj_to_reference(q_traj: np.ndarray, *, reference_len: int) -> np.ndarray:
    q = np.asarray(q_traj, dtype=np.float64).reshape(-1, 3)
    n_ref = int(reference_len)
    if n_ref <= 0:
        return np.zeros((0, 3), dtype=np.float64)
    if q.shape[0] == 0:
        return np.zeros((n_ref, 3), dtype=np.float64)
    if q.shape[0] == n_ref:
        return q.copy()
    src = np.linspace(0.0, 1.0, q.shape[0], dtype=np.float64)
    dst = np.linspace(0.0, 1.0, n_ref, dtype=np.float64)
    cols = [np.interp(dst, src, q[:, j]) for j in range(3)]
    return np.stack(cols, axis=1).astype(np.float64, copy=False)


def _public_row(
    *,
    frame_id: int,
    record: Mapping[str, Any],
    handle: MapHandle,
    prototype_cells: set[tuple[int, int]],
    production_shell_cells: set[tuple[int, int]],
    goal_cells: set[tuple[int, int]],
    base_feasible: np.ndarray,
    final_active: np.ndarray,
    subst: SubstitutionResult,
    report: Mapping[str, Any],
    path_collision_monitor_requested: bool,
) -> Dict[str, Any]:
    rect = record.get("rect")
    bbox = "none"
    if rect is not None:
        bbox = f"{float(rect[0]):.6f},{float(rect[1]):.6f},{float(rect[2]):.6f},{float(rect[3]):.6f}"
    path_monitor_available = bool(report.get("path_collision_monitor_enabled", False))
    reason_code = _hardening_reason_code(
        event_class=str(report.get("event_class", "NO_RELEVANT_CHANGE")),
        path_collision_monitor_requested=bool(path_collision_monitor_requested),
        path_collision_monitor_available=path_monitor_available,
        path_collision_is_collision=bool(report.get("path_collision_is_collision", False)),
        reference_blocked_source=str(report.get("reference_blocked_source", "none")),
        reference_future_slice_fallback=bool(report.get("reference_future_slice_fallback", False)),
        goal_feasible_neighborhood_count=int(report.get("goal_feasible_neighborhood_count", 0)),
        goal_connectivity_ok=bool(report.get("goal_connectivity_ok", True)),
        substitution_removed_non_unknown_cells=int(subst.removed_non_unknown_cells),
    )
    return {
        "prototype_variant": str(record["prototype_variant"]),
        "frame_id": int(frame_id),
        "substitution_scope": str(subst.substitution_scope),
        "accepted_reference_mode": str(report.get("accepted_reference_mode", "stored_npz")),
        "q_alignment_status": str(report.get("q_alignment_status", "")),
        "q_samples_before": int(report.get("q_samples_before", 0)),
        "q_samples_after": int(report.get("q_samples_after", 0)),
        "future_slice_fallback": bool(report.get("reference_future_slice_fallback", False)),
        "expected_future_slice_fallback": bool(report.get("expected_future_slice_fallback", False)),
        "prototype_rect_cells": int(len(prototype_cells)),
        "prototype_rect_bbox_xz": bbox,
        "production_shell_rect_cells": int(len(production_shell_cells)),
        "production_shell_minus_prototype_cells": int(len(production_shell_cells - prototype_cells)),
        "prototype_minus_production_shell_cells": int(len(prototype_cells - production_shell_cells)),
        "goal_corridor_overlap_cells_proxy": int(len(goal_cells & prototype_cells)),
        "future_reference_corridor_proxy_overlap": int(_future_reference_proxy_overlap(handle, prototype_cells, report)),
        "future_reference_corridor_overlap_cells_proxy": int(_future_reference_proxy_overlap(handle, prototype_cells, report)),
        "current_pose_radius_proxy_overlap": int(_current_pose_proxy_overlap(handle, prototype_cells, report)),
        "goal_feasible_neighborhood_count": int(report.get("goal_feasible_neighborhood_count", 0)),
        "goal_connectivity_ok": bool(report.get("goal_connectivity_ok", True)),
        "substitution_removed_non_unknown_cells": int(subst.removed_non_unknown_cells),
        "substitution_removed_unknown_cells": int(subst.removed_unknown_cells),
        "substitution_added_only_prototype_cells": int(subst.added_only_prototype_cells),
        "classifier_event_class": str(report.get("event_class", "NO_RELEVANT_CHANGE")),
        "event_class": str(report.get("event_class", "NO_RELEVANT_CHANGE")),
        "classifier_reference_blocked_source": str(report.get("reference_blocked_source", "none")),
        "classifier_reference_future_slice_fallback": bool(report.get("reference_future_slice_fallback", False)),
        "path_collision_monitor_requested": bool(path_collision_monitor_requested),
        "path_collision_monitor_available": path_monitor_available,
        "path_collision_is_collision": bool(report.get("path_collision_is_collision", False)),
        "path_collision_monitor_is_collision": bool(report.get("path_collision_is_collision", False)),
        "path_collision_colliding_cells_count": int(report.get("path_collision_colliding_cells_count", 0)),
        "path_collision_checked_sample_count": int(report.get("path_collision_checked_sample_count", 0)),
        "reference_blocked_count": int(report.get("reference_blocked_count", 0)),
        "goal_blocked_count": int(report.get("goal_blocked_count", 0)),
        "current_pose_blocked_count": int(report.get("current_pose_blocked_count", 0)),
        "mask_changed_cell_count": int(report.get("mask_changed_cell_count", 0)),
        "hardening_reason_code": reason_code,
        "hardened_reference_blocked_reason": reason_code,
        "replanning_triggered": str(report.get("event_class", "NO_RELEVANT_CHANGE")) not in {
            "NO_RELEVANT_CHANGE",
            "MASK_CHANGED_NONCRITICAL",
        },
        "prototype_error_count": 0,
        "base_feasible_count": int(np.count_nonzero(np.asarray(base_feasible, dtype=bool))),
        "final_active_count": int(np.count_nonzero(np.asarray(final_active, dtype=bool))),
    }


def _hardening_reason_code(
    *,
    event_class: str,
    path_collision_monitor_requested: bool,
    path_collision_monitor_available: bool,
    path_collision_is_collision: bool,
    reference_blocked_source: str,
    reference_future_slice_fallback: bool,
    goal_feasible_neighborhood_count: int,
    goal_connectivity_ok: bool,
    substitution_removed_non_unknown_cells: int,
) -> str:
    event = str(event_class)
    if event in {"NO_RELEVANT_CHANGE", "MASK_CHANGED_NONCRITICAL"}:
        return "no_replan_trigger"
    if int(substitution_removed_non_unknown_cells) > 0:
        return "substitution_artifact_non_unknown_removal"
    if event == "GOAL_CORRIDOR_BLOCKED" or int(goal_feasible_neighborhood_count) <= 0 or not bool(goal_connectivity_ok):
        return "goal_corridor_neighborhood_loss"
    if bool(reference_future_slice_fallback):
        return "classifier_future_slice_fallback"
    if bool(path_collision_monitor_requested) and not bool(path_collision_monitor_available) and event == "REFERENCE_BLOCKED":
        return "path_collision_monitor_unavailable"
    if bool(path_collision_is_collision) and str(reference_blocked_source) == "path_collision_monitor":
        return "true_capsule_future_reference_collision"
    if str(reference_blocked_source) == "corridor_radius_fallback":
        return "substitution_artifact_corridor_radius_fallback"
    return "no_replan_trigger"


def _future_reference_proxy_overlap(
    handle: MapHandle,
    prototype_cells: set[tuple[int, int]],
    report: Mapping[str, Any],
) -> int:
    return int(report.get("reference_blocked_count", 0)) if prototype_cells else 0


def _current_pose_proxy_overlap(
    handle: MapHandle,
    prototype_cells: set[tuple[int, int]],
    report: Mapping[str, Any],
) -> int:
    return int(report.get("current_pose_blocked_count", 0)) if prototype_cells else 0


def _load_accepted_reference(run_path: Path) -> Dict[str, np.ndarray] | None:
    path = run_path / "accepted_reference.npz"
    if not path.exists():
        return None
    with np.load(path, allow_pickle=False) as data:
        if "s" not in data.files or "xz" not in data.files:
            return None
        out: Dict[str, np.ndarray] = {
            "s": np.asarray(data["s"], dtype=np.float64),
            "xz": np.asarray(data["xz"], dtype=np.float64),
        }
        for key in ("q_traj", "q_samples", "q_active_samples"):
            if key in data.files:
                out[key] = np.asarray(data[key], dtype=np.float64)
        return out


def _load_episode_q_traj(path: Path) -> np.ndarray | None:
    if not path.exists():
        return None
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = []
        for row in reader:
            try:
                rows.append([float(row["q2"]), float(row["q3"]), float(row["q5"])])
            except Exception:
                continue
    if not rows:
        return None
    return np.asarray(rows, dtype=np.float64).reshape(-1, 3)


def audit_reference_alignment(run_dir: Path) -> Dict[str, Any]:
    accepted = _load_accepted_reference(run_dir)
    episode_q = _load_episode_q_traj(run_dir / "initial_control" / "episode.csv")
    first_meta = next(iter(sorted((run_dir / "frames").glob("frame_*.json"))), None)
    handle_q_grid_available = False
    map_shape = ""
    if first_meta is not None:
        metadata = _read_json(first_meta)
        handle = _handle(metadata["map_geometry"])
        handle_q_grid_available = getattr(handle, "q_grid", None) is not None
        map_shape = "x".join(str(v) for v in handle.shape)
    keys: list[str] = []
    s_len = 0
    xz_len = 0
    q_len = 0
    stored_can_slice = False
    resampled_can_slice = False
    if accepted is not None:
        keys = sorted(str(k) for k in accepted.keys())
        s_len = int(np.asarray(accepted["s"]).reshape(-1).shape[0]) if "s" in accepted else 0
        xz_len = int(np.asarray(accepted["xz"]).reshape(-1, 2).shape[0]) if "xz" in accepted else 0
        q = _first_q_array(accepted)
        q_len = 0 if q is None else int(np.asarray(q).reshape(-1, 3).shape[0])
        stored_can_slice = bool(s_len > 0 and xz_len == s_len and (q_len == 0 or q_len == s_len))
        resampled_can_slice = bool(s_len > 0 and xz_len == s_len and episode_q is not None and np.asarray(episode_q).reshape(-1, 3).shape[0] > 0)
    episode_len = 0 if episode_q is None else int(np.asarray(episode_q).reshape(-1, 3).shape[0])
    return {
        "accepted_reference_exists": accepted is not None,
        "accepted_reference_keys": ",".join(keys),
        "s_length": int(s_len),
        "xz_length": int(xz_len),
        "q_traj_length": int(q_len),
        "episode_q_sample_count": int(episode_len),
        "stored_q_can_slice_without_fallback": bool(stored_can_slice),
        "episode_q_can_be_resampled_to_len_s": bool(resampled_can_slice),
        "handle_q_grid_available": bool(handle_q_grid_available),
        "map_shape": map_shape,
    }


def _audit_json(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    return {
        "row_count": int(len(rows)),
        "required_row_count_30x3": int(len(rows) == 90),
        "classifier_reference_blocked_source_counts": _counts(rows, "classifier_reference_blocked_source"),
        "classifier_reference_future_slice_fallback_count": int(
            sum(bool(r.get("classifier_reference_future_slice_fallback", False)) for r in rows)
        ),
        "path_collision_monitor_available_count": int(sum(bool(r.get("path_collision_monitor_available", False)) for r in rows)),
        "event_counts": _counts(rows, "classifier_event_class"),
        "reason_counts": _counts(rows, "hardening_reason_code"),
    }


def _summary(
    *,
    rows_by_scope: Mapping[str, Sequence[Mapping[str, Any]]],
    requested_scope: str,
    run_dir: Path,
    stage_b_dir: Path | None,
    stage_c_dir: Path | None,
    enable_path_collision_monitor: bool,
    adjacency_radius_px: int,
) -> Dict[str, Any]:
    requested_rows = list(rows_by_scope[requested_scope])
    full_rows = list(rows_by_scope["full_unknown_layer"])
    per_scope = {
        scope: {
            "row_count": int(len(rows)),
            "per_variant": _per_variant_summary(rows),
            "reason_counts": _counts(rows, "hardening_reason_code"),
            "event_counts": _counts(rows, "classifier_event_class"),
        }
        for scope, rows in rows_by_scope.items()
    }
    median = [r for r in requested_rows if str(r.get("prototype_variant")) == "median_adjacent"]
    median_full = [r for r in full_rows if str(r.get("prototype_variant")) == "median_adjacent"]
    median_triggers = sum(bool(r.get("replanning_triggered", False)) for r in median)
    median_full_triggers = sum(bool(r.get("replanning_triggered", False)) for r in median_full)
    median_trigger_reduction = 0.0
    if median_full_triggers > 0:
        median_trigger_reduction = 1.0 - float(median_triggers) / float(median_full_triggers)
    reason_counts = _counts(median, "hardening_reason_code")
    summary: Dict[str, Any] = {
        "goal": "INVALID_DEPTH_UNKNOWN_ALTERNATIVE_PROJECTION_HARDENING_NO_COMMAND",
        "run_dir": str(run_dir),
        "stage_b_dir": "" if stage_b_dir is None else str(stage_b_dir),
        "stage_c_dir": "" if stage_c_dir is None else str(stage_c_dir),
        "adjacency_radius_px": int(adjacency_radius_px),
        "enable_path_collision_monitor": bool(enable_path_collision_monitor),
        "requested_substitution_scope": str(requested_scope),
        "substitution_scope_modes_present": list(SUBSTITUTION_SCOPES),
        "frames_analyzed": len({int(r.get("frame_id", -1)) for r in requested_rows if int(r.get("frame_id", -1)) >= 0}),
        "row_count": int(len(requested_rows)),
        "classifier_reconstruction_audit_row_count": int(len(full_rows)),
        "per_scope": per_scope,
        "median_adjacent_true_collision_frames": int(reason_counts.get("true_capsule_future_reference_collision", 0)),
        "median_adjacent_substitution_artifact_frames": int(
            reason_counts.get("substitution_artifact_corridor_radius_fallback", 0)
            + reason_counts.get("substitution_artifact_non_unknown_removal", 0)
        ),
        "median_adjacent_path_monitor_unavailable_frames": int(reason_counts.get("path_collision_monitor_unavailable", 0)),
        "median_adjacent_no_replan_trigger_frames": int(reason_counts.get("no_replan_trigger", 0)),
        "median_adjacent_trigger_frames": int(median_triggers),
        "median_adjacent_full_unknown_layer_trigger_frames": int(median_full_triggers),
        "median_adjacent_trigger_reduction_vs_full_unknown_layer": float(median_trigger_reduction),
    }
    return summary


def _aligned_summary(
    *,
    rows: Sequence[Mapping[str, Any]],
    run_dir: Path,
    stage_b_dir: Path | None,
    stage_c_dir: Path | None,
    audit: Mapping[str, Any],
    accepted_reference_modes: Sequence[str],
    substitution_scope: str,
    enable_path_collision_monitor: bool,
    adjacency_radius_px: int,
) -> Dict[str, Any]:
    per_mode: Dict[str, Any] = {}
    for mode in sorted({str(r.get("accepted_reference_mode", "")) for r in rows if r.get("accepted_reference_mode")}):
        mode_rows = [r for r in rows if str(r.get("accepted_reference_mode")) == mode]
        per_mode[mode] = {
            "row_count": int(len(mode_rows)),
            "event_counts": _counts(mode_rows, "classifier_event_class"),
            "reason_counts": _counts(mode_rows, "hardening_reason_code"),
            "q_alignment_status_counts": _counts(mode_rows, "q_alignment_status"),
            "future_slice_fallback_count": int(sum(bool(r.get("future_slice_fallback", False)) for r in mode_rows)),
            "per_variant": _per_variant_summary(mode_rows),
        }
    median_by_mode: Dict[str, Any] = {}
    for mode, mode_summary in per_mode.items():
        median = [
            r for r in rows
            if str(r.get("accepted_reference_mode")) == mode and str(r.get("prototype_variant")) == "median_adjacent"
        ]
        median_by_mode[mode] = {
            "event_counts": _counts(median, "classifier_event_class"),
            "reason_counts": _counts(median, "hardening_reason_code"),
            "future_slice_fallback_count": int(sum(bool(r.get("future_slice_fallback", False)) for r in median)),
            "true_capsule_future_reference_collision": int(
                sum(str(r.get("hardening_reason_code")) == "true_capsule_future_reference_collision" for r in median)
            ),
            "trigger_frames": int(sum(bool(r.get("replanning_triggered", False)) for r in median)),
        }
    return {
        "goal": "INVALID_DEPTH_UNKNOWN_ALTERNATIVE_PROJECTION_HARDENING_NO_COMMAND",
        "stage": "stage_e_reference_alignment",
        "run_dir": str(run_dir),
        "stage_b_dir": "" if stage_b_dir is None else str(stage_b_dir),
        "stage_c_dir": "" if stage_c_dir is None else str(stage_c_dir),
        "accepted_reference_modes": list(accepted_reference_modes),
        "substitution_scope": str(substitution_scope),
        "adjacency_radius_px": int(adjacency_radius_px),
        "enable_path_collision_monitor": bool(enable_path_collision_monitor),
        "frames_analyzed": len({int(r.get("frame_id", -1)) for r in rows if int(r.get("frame_id", -1)) >= 0}),
        "row_count": int(len(rows)),
        "reference_alignment_audit": dict(audit),
        "per_mode": per_mode,
        "median_adjacent_by_mode": median_by_mode,
        "prototype_error_count": int(sum(int(r.get("prototype_error_count", 0)) for r in rows)),
    }


def _per_variant_summary(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    variants = sorted({str(r.get("prototype_variant", "")) for r in rows if r.get("prototype_variant")})
    return {
        variant: {
            "row_count": len([r for r in rows if str(r.get("prototype_variant")) == variant]),
            "prototype_rect_cells": _stats([r for r in rows if str(r.get("prototype_variant")) == variant], "prototype_rect_cells"),
            "production_shell_minus_prototype_cells": _stats(
                [r for r in rows if str(r.get("prototype_variant")) == variant],
                "production_shell_minus_prototype_cells",
            ),
            "goal_corridor_overlap_cells_proxy": _stats(
                [r for r in rows if str(r.get("prototype_variant")) == variant],
                "goal_corridor_overlap_cells_proxy",
            ),
            "event_counts": _counts([r for r in rows if str(r.get("prototype_variant")) == variant], "classifier_event_class"),
            "reason_counts": _counts([r for r in rows if str(r.get("prototype_variant")) == variant], "hardening_reason_code"),
            "q_alignment_status_counts": _counts(
                [r for r in rows if str(r.get("prototype_variant")) == variant],
                "q_alignment_status",
            ),
            "future_slice_fallback_count": int(
                sum(bool(r.get("future_slice_fallback", False)) for r in rows if str(r.get("prototype_variant")) == variant)
            ),
            "path_collision_monitor_available_count": int(
                sum(bool(r.get("path_collision_monitor_available", False)) for r in rows if str(r.get("prototype_variant")) == variant)
            ),
        }
        for variant in variants
    }


def _replan_trigger_rows(
    rows: Sequence[Mapping[str, Any]],
    *,
    group_keys: Sequence[str] = ("prototype_variant",),
) -> list[Dict[str, Any]]:
    output = []
    events = ("GOAL_CORRIDOR_BLOCKED", "REFERENCE_BLOCKED", "NO_RELEVANT_CHANGE", "CURRENT_POSE_UNSAFE", "MASK_CHANGED_NONCRITICAL")
    for group, subset in _group_rows(rows, group_keys):
        counts = _counts(subset, "classifier_event_class")
        row = {key: value for key, value in zip(group_keys, group)}
        row["total_frames"] = len(subset)
        for event in events:
            row[event] = int(counts.get(event, 0))
        output.append(row)
    return output


def _reason_rows(
    rows: Sequence[Mapping[str, Any]],
    *,
    group_keys: Sequence[str] = ("prototype_variant",),
) -> list[Dict[str, Any]]:
    output = []
    for group, subset in _group_rows(rows, group_keys):
        counts = _counts(subset, "hardening_reason_code")
        row = {key: value for key, value in zip(group_keys, group)}
        row["total_frames"] = len(subset)
        for reason in REASON_CODES:
            row[reason] = int(counts.get(reason, 0))
        output.append(row)
    return output


def _group_rows(
    rows: Sequence[Mapping[str, Any]],
    group_keys: Sequence[str],
) -> list[tuple[tuple[str, ...], list[Mapping[str, Any]]]]:
    groups: Dict[tuple[str, ...], list[Mapping[str, Any]]] = {}
    for row in rows:
        key = tuple(str(row.get(k, "")) for k in group_keys)
        if all(key):
            groups.setdefault(key, []).append(row)
    return [(key, groups[key]) for key in sorted(groups)]


def _counts(rows: Sequence[Mapping[str, Any]], key: str) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for row in rows:
        value = str(row.get(key, ""))
        if value:
            counts[value] = counts.get(value, 0) + 1
    return counts


def _no_command_record() -> Dict[str, bool]:
    return {
        "robot_command_endpoint_touched": False,
        "control_module_called_in_live_loop": False,
        "production_module_modified": False,
        "live_capture_performed": False,
        "artifact_read_only": True,
        "subscribes_to_ros_topics": False,
        "publishes_robot_commands": False,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--stage-b-dir", default=None)
    parser.add_argument("--stage-c-dir", default=None)
    parser.add_argument("--variants", nargs="+", default=list(VARIANTS))
    parser.add_argument("--substitution-scope", choices=SUBSTITUTION_SCOPES, default="rect_specific")
    parser.add_argument("--adjacency-radius-px", type=int, default=5)
    parser.add_argument("--enable-path-collision-monitor", action="store_true")
    parser.add_argument("--accepted-reference-mode", nargs="+", choices=ACCEPTED_REFERENCE_MODES, default=["stored_npz"])
    parser.add_argument("--out", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = run_hardening(
        run_dir=Path(args.run_dir),
        stage_b_dir=None if args.stage_b_dir is None else Path(args.stage_b_dir),
        stage_c_dir=None if args.stage_c_dir is None else Path(args.stage_c_dir),
        variants=[str(v) for v in args.variants],
        substitution_scope=str(args.substitution_scope),
        adjacency_radius_px=int(args.adjacency_radius_px),
        enable_path_collision_monitor=bool(args.enable_path_collision_monitor),
        accepted_reference_modes=[str(v) for v in args.accepted_reference_mode],
        out_dir=Path(args.out),
    )
    print(json.dumps(_jsonable(summary), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
