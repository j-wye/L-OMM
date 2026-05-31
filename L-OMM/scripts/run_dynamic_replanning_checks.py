#!/usr/bin/env python3
"""Synthetic dynamic map-update and replanning dry-run runner.

This runner intentionally excludes live camera, ROS2 subscriptions, full BT
supervisor logic, mobile-base recovery, and physical robot execution.
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional, Sequence


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
from map_update_layer.dynamic_replanning_harness import DynamicReplanningHarness  # noqa: E402
from map_update_layer.synthetic_dynamic_sequences import SyntheticDynamicSequences  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run synthetic dynamic map-update/replanning checks. "
            "No live camera, no ROS2 topic, no full BT supervisor, no hardware execution."
        )
    )
    parser.add_argument("--map-path", default=None, help="Optional explicit reduced active map .npy path.")
    parser.add_argument("--mu-min", type=float, default=DEFAULT_MU_MIN)
    parser.add_argument("--inflation-m", type=float, default=DEFAULT_OBSTACLE_INFLATION_M)
    parser.add_argument("--cost-mode", choices=("distance", "manipulability", "all"), default=DEFAULT_COST_MODE)
    parser.add_argument("--w", "--manip-weight", dest="manip_weight", type=float, default=DEFAULT_MANIP_WEIGHT)
    parser.add_argument("--kappa", "--manip-kappa", dest="manip_kappa", type=float, default=DEFAULT_MANIP_KAPPA)
    parser.add_argument("--reference-backend", choices=("linear", "c2_quintic"), default="c2_quintic")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--v-ref", type=float, default=0.04)
    parser.add_argument("--terminal-hold-s", type=float, default=1.0)
    parser.add_argument("--out", default=None, help="Output root. Defaults to path/dynamic_replanning_checks.")
    parser.add_argument("--scenario", action="append", default=None,
                        help="Run one synthetic scenario by name. Can be repeated. Default: all scenarios.")
    parser.add_argument("--list-scenarios", action="store_true", help="List available synthetic scenarios and exit.")
    parser.add_argument("--make-plots", action="store_true", help="Write optional PNG masks and control plots.")
    parser.add_argument("--use-reference-diff", action="store_true",
                        help="Use ReferenceSnapshotDiff instead of previous-frame mask xor for runtime diff stats.")
    parser.add_argument("--reference-diff-use-task-fov", action="store_true",
                        help="Use the task-plane FOV mask in the synthetic harness. Default keeps full-FOV camera-free regression semantics.")
    parser.add_argument("--reference-diff-pixel-band-px", type=float, default=100.0)
    parser.add_argument("--reference-diff-tau-diff-cells", type=int, default=3)
    parser.add_argument("--reference-diff-tau-cluster-cells", type=int, default=5)
    parser.add_argument("--include-p2-validation-scenarios", action="store_true",
                        help="Include non-baseline P2 validation scenarios such as target_inflation_vs_grasp_window.")
    return parser


def _modes(selected: str) -> List[str]:
    if selected == "all":
        return ["distance", "manipulability"]
    return [selected]


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    requested = set(str(name) for name in (args.scenario or ()))
    p2_scenarios = {"capsule_link_collision_box", "target_inflation_vs_grasp_window"}
    scenarios = SyntheticDynamicSequences(inflation_m=float(args.inflation_m)).all_scenarios(
        include_reference_diff_scenarios=True,
        include_p2_validation_scenarios=True,
    )
    if args.list_scenarios:
        for name, scenario in scenarios.items():
            print(f"{name}: {scenario.description}")
        return 0

    out_base = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "dynamic_replanning_checks"))
    scenario_names = args.scenario if args.scenario else None
    if scenario_names and any(str(name).startswith("reference_snapshot_") for name in scenario_names):
        if not bool(args.use_reference_diff):
            raise SystemExit("reference_snapshot_* scenarios require --use-reference-diff")
    include_p2_validation = bool(args.include_p2_validation_scenarios or (requested & p2_scenarios))
    for mode in _modes(str(args.cost_mode)):
        out_root = os.path.join(out_base, mode)
        harness = DynamicReplanningHarness(
            output_root=out_root,
            map_path=args.map_path,
            mu_min=float(args.mu_min),
            inflation_m=float(args.inflation_m),
            cost_mode=mode,
            manip_weight=float(args.manip_weight),
            manip_kappa=float(args.manip_kappa),
            reference_backend=str(args.reference_backend),
            dt=float(args.dt),
            v_ref=float(args.v_ref),
            terminal_hold_s=float(args.terminal_hold_s),
            make_plots=bool(args.make_plots),
            use_reference_diff=bool(args.use_reference_diff),
            reference_diff_use_task_fov=bool(args.reference_diff_use_task_fov),
            reference_diff_pixel_band_px=float(args.reference_diff_pixel_band_px),
            reference_diff_tau_diff_cells=int(args.reference_diff_tau_diff_cells),
            reference_diff_tau_cluster_cells=int(args.reference_diff_tau_cluster_cells),
            include_p2_validation_scenarios=include_p2_validation,
        )
        summary = harness.run_all(scenario_names=scenario_names)
        print(
            f"[dynamic_replanning_checks] mode={mode} "
            f"scenarios={len(summary['scenarios'])} out={out_root}"
        )
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
