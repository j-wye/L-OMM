#!/usr/bin/env python3
"""Camera-free validation harness for the map update layer.

This runner deliberately avoids live camera, perception, and BT supervisor
logic.  It builds synthetic semantic map-update requests, converts them into
ActiveMapSnapshot objects, and can optionally pass each snapshot into the
ControlModule run-once evaluator.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, Optional, Sequence

import numpy as np


SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)
if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(1, CONTROL_MODULE_DIR)

from control_module.constants import (  # noqa: E402
    DEFAULT_COST_MODE,
    DEFAULT_MANIP_KAPPA,
    DEFAULT_MANIP_WEIGHT,
    DEFAULT_MU_MIN,
    DEFAULT_OBSTACLE_INFLATION_M,
)
from control_module.control_module import ControlModule  # noqa: E402
from control_module.mission_request import MissionRequest  # noqa: E402
from control_module.path_planning import PathPlanning  # noqa: E402
from map_update_layer.map_update_layer import MapUpdateLayer  # noqa: E402
from map_update_layer.snapshot_validator import SnapshotValidator  # noqa: E402
from map_update_layer.synthetic_cases import SyntheticMapUpdateCases  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate synthetic map-update layer cases without camera or BT supervisor."
    )
    parser.add_argument("--map-path", default=None, help="Optional explicit reduced active map .npy path.")
    parser.add_argument("--mu-min", type=float, default=DEFAULT_MU_MIN)
    parser.add_argument("--inflation-m", type=float, default=DEFAULT_OBSTACLE_INFLATION_M)
    parser.add_argument("--cost-mode", choices=("distance", "manipulability"), default=DEFAULT_COST_MODE)
    parser.add_argument("--w", "--manip-weight", dest="manip_weight", type=float, default=DEFAULT_MANIP_WEIGHT)
    parser.add_argument("--kappa", "--manip-kappa", dest="manip_kappa", type=float, default=DEFAULT_MANIP_KAPPA)
    parser.add_argument("--reference-backend", choices=("linear", "c2_quintic"), default="c2_quintic")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--v-ref", type=float, default=0.04)
    parser.add_argument("--terminal-hold-s", type=float, default=1.0)
    parser.add_argument("--out", default=None, help="Output root for synthetic snapshots and summaries.")
    parser.add_argument("--no-control", action="store_true", help="Only build/validate snapshots; skip ControlModule.")
    parser.add_argument("--no-plot", action="store_true", help="Disable ControlModule plots when control check is on.")
    return parser


def _json_safe(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {str(k): _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.generic):
        return obj.item()
    return obj


def _case_dir(root: str, name: str) -> str:
    path = os.path.join(root, name)
    os.makedirs(path, exist_ok=True)
    return path


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "map_update_layer_checks", args.cost_mode))
    os.makedirs(out_root, exist_ok=True)

    planner = PathPlanning(map_path=args.map_path)
    handle = planner.load_map(args.map_path)
    updater = MapUpdateLayer()
    validator = SnapshotValidator()
    cases = SyntheticMapUpdateCases(inflation_m=float(args.inflation_m)).all_cases()
    clear_snapshot = updater.build(handle, mu_min=float(args.mu_min), request=cases["clear"])
    validator.assert_valid(clear_snapshot)

    control = None if args.no_control else ControlModule(output_root=out_root)
    summary: Dict[str, Any] = {
        "map_path": handle.map_path,
        "map_tag": handle.tag,
        "mu_min": float(args.mu_min),
        "inflation_m": float(args.inflation_m),
        "cost_mode": str(args.cost_mode),
        "manip_weight": float(args.manip_weight),
        "manip_kappa": float(args.manip_kappa),
        "control_check_enabled": not bool(args.no_control),
        "cases": {},
    }

    for name, request in cases.items():
        snapshot = updater.build(handle, mu_min=float(args.mu_min), request=request)
        validator.assert_valid(snapshot)
        cdir = _case_dir(out_root, name)
        np.save(os.path.join(cdir, "base_feasible_mask.npy"), snapshot.base_feasible_mask)
        np.save(os.path.join(cdir, "final_active_mask.npy"), snapshot.final_active_mask)
        np.save(os.path.join(cdir, "blocked_mask.npy"), snapshot.blocked_mask)
        for layer_name, mask in snapshot.layer_masks.items():
            np.save(os.path.join(cdir, f"layer_{layer_name}.npy"), mask)

        case_summary: Dict[str, Any] = {
            "snapshot_source": snapshot.source,
            "snapshot_stats": dict(snapshot.stats),
        }
        if control is not None:
            mission = MissionRequest(
                scenario="clear" if name == "clear" else "changed",
                clear_snapshot=clear_snapshot,
                active_snapshot=snapshot,
                cost_mode=str(args.cost_mode),
                manip_weight=float(args.manip_weight),
                manip_kappa=float(args.manip_kappa),
                reference_backend=str(args.reference_backend),
                dt=float(args.dt),
                v_ref=float(args.v_ref),
                terminal_hold_s=float(args.terminal_hold_s),
                make_plots=not bool(args.no_plot),
                write_artifacts=True,
            )
            case_summary["control_summary"] = control.run(mission, out_dir=cdir)
        summary["cases"][name] = case_summary
        with open(os.path.join(cdir, "snapshot_summary.json"), "w", encoding="utf-8") as f:
            json.dump(_json_safe(case_summary), f, indent=2, ensure_ascii=False)
        print(
            f"[{name}] final={int(snapshot.stats['final_feasible_cells'])} "
            f"blocked={int(snapshot.stats['blocked_cells'])} "
            f"valid={snapshot.stats['snapshot_valid'] >= 1.0}"
        )

    with open(os.path.join(out_root, "map_update_layer_check_summary.json"), "w", encoding="utf-8") as f:
        json.dump(_json_safe(summary), f, indent=2, ensure_ascii=False)
    print(f"[map_update_layer_checks] out={out_root}")
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
