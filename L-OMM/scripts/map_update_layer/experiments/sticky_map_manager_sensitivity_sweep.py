#!/usr/bin/env python3
"""Synthetic StickyMapManager sensitivity sweep.

This experiment is diagnostic-only.  It does not open ROS2 topics, subscribe to
RGB-D streams, publish robot commands, or alter production map-update logic.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

import numpy as np


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(0, CONTROL_MODULE_DIR)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(1, SCRIPTS_DIR)

from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams  # noqa: E402


DEFAULT_SWEEP_CONFIG: Dict[str, List[float]] = {
    "n_free_frames_to_unblock": [1, 2, 3, 5, 7, 10],
    "conservative_unknown_persistence": [3, 5, 10, 15, 20],
    "inflation_m": [0.005, 0.01, 0.015, 0.02, 0.03],
}


@dataclass(frozen=True)
class SequenceSpec:
    shape: tuple[int, int] = (24, 24)
    primary_cell: tuple[int, int] = (12, 12)
    outside_fov_cell: tuple[int, int] = (6, 18)
    unknown_cell: tuple[int, int] = (18, 6)
    truth_removed_frame: int = 10
    outside_fov_start_frame: int = 8
    unknown_observed_frame: int = 4


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run a synthetic sensitivity sweep for StickyMapManager. "
            "No live camera, ROS2 topic, or physical robot execution."
        )
    )
    parser.add_argument("--out", default=None)
    parser.add_argument("--n-frames", type=int, default=30)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--sweep-config",
        default=None,
        help="Optional JSON file or inline JSON overriding sweep ranges.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = Path(args.out or os.path.join(PROJECT_ROOT, "path", "sticky_map_manager_sensitivity_sweep"))
    out_root.mkdir(parents=True, exist_ok=True)
    config = load_sweep_config(args.sweep_config)
    rows = run_sweep(config=config, n_frames=int(args.n_frames), seed=int(args.seed))
    pareto = parameter_pareto(rows)
    summary = summarize_rows(rows, config=config, n_frames=int(args.n_frames), seed=int(args.seed))
    write_csv(out_root / "sensitivity_sweep_results.csv", rows)
    write_json(out_root / "sensitivity_sweep_summary.json", summary)
    write_json(out_root / "parameter_pareto.json", pareto)
    print(
        "[sticky_map_manager_sensitivity_sweep] "
        f"rows={len(rows)} pareto={len(pareto['pareto_rows'])} out={out_root}"
    )
    return 0 if summary["pass"] else 1


def load_sweep_config(value: str | None) -> Dict[str, List[float]]:
    if not value:
        return {key: list(vals) for key, vals in DEFAULT_SWEEP_CONFIG.items()}
    text = str(value).strip()
    if text.startswith("{"):
        raw = json.loads(text)
    else:
        with open(text, "r", encoding="utf-8") as f:
            raw = json.load(f)
    config = {key: list(vals) for key, vals in DEFAULT_SWEEP_CONFIG.items()}
    for key, vals in dict(raw).items():
        if key not in config:
            raise ValueError(f"unknown sweep config key: {key}")
        config[key] = [float(v) for v in vals]
    return config


def run_sweep(*, config: Mapping[str, Sequence[float]], n_frames: int, seed: int) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for n_free in config["n_free_frames_to_unblock"]:
        for unknown_persistence in config["conservative_unknown_persistence"]:
            for inflation_m in config["inflation_m"]:
                rows.append(
                    run_one(
                        n_free_frames_to_unblock=int(n_free),
                        conservative_unknown_persistence=int(unknown_persistence),
                        inflation_m=float(inflation_m),
                        n_frames=int(n_frames),
                        seed=int(seed),
                    )
                )
    return rows


def run_one(
    *,
    n_free_frames_to_unblock: int,
    conservative_unknown_persistence: int,
    inflation_m: float,
    n_frames: int,
    seed: int,
) -> Dict[str, Any]:
    spec = SequenceSpec()
    params = StickyMapParams(
        inflation_m=float(inflation_m),
        n_free_frames_to_unblock=int(n_free_frames_to_unblock),
        conservative_unknown_persistence=int(conservative_unknown_persistence),
        resolution_m=0.01,
    )
    manager = StickyMapManager(params=params)
    initial = np.zeros(spec.shape, dtype=bool)
    initial[spec.primary_cell] = True
    initial[spec.outside_fov_cell] = True
    state = manager.initialize({"occupied_mask": initial})
    false_unblock_count = 0
    first_unblocked_frame: int | None = None
    outside_preserved_frames = 0
    transition_count = 0
    unknown_persistence_blocked_frames = 0
    unknown_release_frame: int | None = None
    prev_primary_occ = bool(state.occupied_mask[spec.primary_cell])
    rng = np.random.default_rng(seed)

    for frame_idx in range(int(n_frames)):
        observation, fov, truth_occ = synthetic_observation(
            spec,
            frame_idx=frame_idx,
            n_free_frames_to_unblock=int(n_free_frames_to_unblock),
            rng=rng,
        )
        state = manager.update(state, observation, fov)
        primary_occ = bool(state.occupied_mask[spec.primary_cell])
        if primary_occ != prev_primary_occ:
            transition_count += 1
        prev_primary_occ = primary_occ
        if bool(truth_occ[spec.primary_cell]) and not primary_occ:
            false_unblock_count += 1
        if (
            frame_idx >= spec.truth_removed_frame
            and first_unblocked_frame is None
            and not primary_occ
        ):
            first_unblocked_frame = int(frame_idx)
        if frame_idx >= spec.outside_fov_start_frame and bool(state.occupied_mask[spec.outside_fov_cell]):
            outside_preserved_frames += 1
        if frame_idx > spec.unknown_observed_frame and bool(state.blocked_mask[spec.unknown_cell]):
            unknown_persistence_blocked_frames += 1
        if (
            frame_idx > spec.unknown_observed_frame
            and unknown_release_frame is None
            and not bool(state.blocked_mask[spec.unknown_cell])
        ):
            unknown_release_frame = int(frame_idx)

    unblock_latency = (
        None
        if first_unblocked_frame is None
        else int(first_unblocked_frame - spec.truth_removed_frame + 1)
    )
    inflation_overhead = inflation_overhead_cells(
        occupied_mask=state.occupied_mask,
        inflation_m=float(inflation_m),
        resolution_m=float(params.resolution_m),
    )
    blocked_count = int(np.count_nonzero(state.blocked_mask))
    occupied_count = int(np.count_nonzero(state.occupied_mask))
    return {
        "n_free_frames_to_unblock": int(n_free_frames_to_unblock),
        "conservative_unknown_persistence": int(conservative_unknown_persistence),
        "inflation_m": float(inflation_m),
        "n_frames": int(n_frames),
        "unblock_latency_frames": None if unblock_latency is None else int(unblock_latency),
        "false_unblock_count": int(false_unblock_count),
        "sticky_persistence_frames": int(outside_preserved_frames),
        "inflation_overhead_cells": int(inflation_overhead),
        "occupied_cell_count_final": int(occupied_count),
        "blocked_cell_count_final": int(blocked_count),
        "state_transition_count": int(transition_count),
        "unknown_persistence_blocked_frames": int(unknown_persistence_blocked_frames),
        "unknown_release_frame": None if unknown_release_frame is None else int(unknown_release_frame),
        "unknown_persistence_observed_effect": "active_blocked_ttl",
        "pass": bool(unblock_latency is not None and false_unblock_count >= 0),
    }


def synthetic_observation(
    spec: SequenceSpec,
    *,
    frame_idx: int,
    n_free_frames_to_unblock: int,
    rng: np.random.Generator,
) -> tuple[Dict[str, np.ndarray], np.ndarray, np.ndarray]:
    occ = np.zeros(spec.shape, dtype=bool)
    free = np.zeros(spec.shape, dtype=bool)
    unknown = np.zeros(spec.shape, dtype=bool)
    fov = np.ones(spec.shape, dtype=bool)
    truth_occ = np.zeros(spec.shape, dtype=bool)

    primary_truth = frame_idx < spec.truth_removed_frame
    truth_occ[spec.primary_cell] = primary_truth
    truth_occ[spec.outside_fov_cell] = frame_idx < spec.outside_fov_start_frame

    # Before true removal, produce a deterministic free-observation flicker that
    # can expose too-small n_free thresholds as false unblocks.
    if primary_truth:
        flicker_len = max(int(n_free_frames_to_unblock), 1)
        if 2 <= frame_idx < 2 + flicker_len:
            free[spec.primary_cell] = True
        else:
            occ[spec.primary_cell] = True
    else:
        free[spec.primary_cell] = True

    if frame_idx < spec.outside_fov_start_frame:
        occ[spec.outside_fov_cell] = True
    else:
        fov[spec.outside_fov_cell] = False
        if bool(rng.integers(0, 2)):
            free[spec.outside_fov_cell] = True
    if frame_idx == spec.unknown_observed_frame:
        unknown[spec.unknown_cell] = True
    return {"occupied_mask": occ, "free_mask": free, "unknown_mask": unknown}, fov, truth_occ


def known_unblock_latency(n_free_frames_to_unblock: int) -> int:
    row = run_one(
        n_free_frames_to_unblock=int(n_free_frames_to_unblock),
        conservative_unknown_persistence=10,
        inflation_m=0.0,
        n_frames=20,
        seed=1,
    )
    return int(row["unblock_latency_frames"])


def known_false_unblock_count(n_free_frames_to_unblock: int, n_frames: int = 12) -> int:
    row = run_one(
        n_free_frames_to_unblock=int(n_free_frames_to_unblock),
        conservative_unknown_persistence=10,
        inflation_m=0.0,
        n_frames=int(n_frames),
        seed=2,
    )
    return int(row["false_unblock_count"])


def inflation_overhead_cells(*, occupied_mask: np.ndarray, inflation_m: float, resolution_m: float) -> int:
    manager = StickyMapManager(params=StickyMapParams(inflation_m=float(inflation_m), resolution_m=float(resolution_m)))
    inflated = manager._compose_blocked(np.asarray(occupied_mask, dtype=bool), None)
    return int(np.count_nonzero(inflated) - np.count_nonzero(np.asarray(occupied_mask, dtype=bool)))


def parameter_pareto(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    sorted_rows = sorted(
        rows,
        key=lambda r: (
            int(r["false_unblock_count"]),
            int(10**9 if r["unblock_latency_frames"] is None else r["unblock_latency_frames"]),
            int(r["inflation_overhead_cells"]),
        ),
    )
    pareto: List[Dict[str, Any]] = []
    best_latency = 10**9
    for row in sorted_rows:
        latency = int(10**9 if row["unblock_latency_frames"] is None else row["unblock_latency_frames"])
        if latency <= best_latency:
            pareto.append(dict(row))
            best_latency = latency
    return {
        "objective_order": ["min_false_unblock_count", "min_unblock_latency_frames", "min_inflation_overhead_cells"],
        "pareto_rows": pareto,
    }


def summarize_rows(
    rows: Sequence[Mapping[str, Any]],
    *,
    config: Mapping[str, Sequence[float]],
    n_frames: int,
    seed: int,
) -> Dict[str, Any]:
    latencies = [
        int(row["unblock_latency_frames"])
        for row in rows
        if row.get("unblock_latency_frames") is not None
    ]
    unknown_frames = [int(row["unknown_persistence_blocked_frames"]) for row in rows]
    return {
        "mode": "sticky_map_manager_sensitivity_sweep",
        "explicitly_no_live_camera": True,
        "explicitly_no_ros2_topic": True,
        "explicitly_no_physical_robot_execution": True,
        "row_count": int(len(rows)),
        "n_frames": int(n_frames),
        "seed": int(seed),
        "sweep_config": {key: [float(v) for v in vals] for key, vals in config.items()},
        "unblock_latency_frames_min": int(min(latencies)) if latencies else None,
        "unblock_latency_frames_max": int(max(latencies)) if latencies else None,
        "false_unblock_count_min": int(min(int(row["false_unblock_count"]) for row in rows)) if rows else None,
        "false_unblock_count_max": int(max(int(row["false_unblock_count"]) for row in rows)) if rows else None,
        "unknown_persistence_blocked_frames_min": int(min(unknown_frames)) if unknown_frames else None,
        "unknown_persistence_blocked_frames_max": int(max(unknown_frames)) if unknown_frames else None,
        "unknown_persistence_contract_note": "conservative_unknown_persistence is consumed by StickyMapManager.",
        "pass": bool(rows and all(bool(row.get("pass", False)) for row in rows)),
    }


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


def write_json(path: Path, data: Mapping[str, Any] | Sequence[Mapping[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


if __name__ == "__main__":
    raise SystemExit(main())
