#!/usr/bin/env python3
"""Experiment runner for the reduced-active control module.

This file is intentionally the only command-line entry point.  The
``control_module`` package is treated as a library: it receives a
``MissionRequest`` and returns a structured summary dictionary.
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import Any, Dict, Optional, Sequence, Tuple


SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)
if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(1, CONTROL_MODULE_DIR)

from control_module.control_module import ControlModule
from control_module.constants import (
    DEFAULT_COST_MODE,
    DEFAULT_GOAL_PROJECTION_FACTOR,
    DEFAULT_MANIP_KAPPA,
    DEFAULT_MANIP_MU_SAFE,
    DEFAULT_MANIP_MU_SAFE_PERCENTILE,
    DEFAULT_MANIP_WEIGHT,
    DEFAULT_MAP_UPDATE_HZ,
    DEFAULT_MU_MIN,
    DEFAULT_OBSTACLE,
    DEFAULT_OBSTACLE_INFLATION_M,
    DEFAULT_TRACKING_GAP_M,
    Rect,
)
from control_module.mission_request import MissionRequest
from control_module.path_planning import PathPlanning
from map_update_layer.map_update_layer import MapUpdateLayer
from map_update_layer.map_update_request import MapUpdateRequest


def _parse_rect(raw: str) -> Rect:
    vals = [float(v.strip()) for v in raw.replace(";", ",").split(",") if v.strip()]
    if len(vals) != 4:
        raise argparse.ArgumentTypeError("rectangle must be x_l,z_t,x_r,z_b")
    x_l, z_t, x_r, z_b = vals
    if x_r <= x_l:
        raise argparse.ArgumentTypeError("rectangle requires x_r > x_l")
    if z_t <= z_b:
        raise argparse.ArgumentTypeError("rectangle requires z_t > z_b")
    return x_l, z_t, x_r, z_b


def _parse_xz(x: Optional[float], z: Optional[float], name: str) -> Optional[Tuple[float, float]]:
    if x is None and z is None:
        return None
    if x is None or z is None:
        raise ValueError(f"{name} override requires both x and z")
    return float(x), float(z)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run reduced-active active-map planning, reference generation, "
            "and DLS validation."
        )
    )
    parser.add_argument(
        "--scenario",
        choices=("clear", "changed", "all"),
        default="clear",
        help="Execution scenario. Default is clear for quick validation.",
    )
    parser.add_argument(
        "--cost-mode",
        choices=("distance", "manipulability"),
        default=DEFAULT_COST_MODE,
        help="A* edge policy. Default is the tuned manipulability cost.",
    )
    parser.add_argument(
        "--reference-backend",
        choices=("linear", "c2_quintic"),
        default="c2_quintic",
        help="Reference generator used for single-scenario runs.",
    )
    parser.add_argument(
        "--scheduler-mode",
        choices=("path_following", "time_envelope"),
        default="path_following",
        help="DLS reference scheduler. The default is gap-aware path following.",
    )
    parser.add_argument("--map-path", default=None, help="Optional explicit map .npy path.")
    parser.add_argument(
        "--obstacle",
        action="append",
        type=_parse_rect,
        default=None,
        help="Map-update occupied rectangle x_l,z_t,x_r,z_b for changed scenario. Repeatable.",
    )
    parser.add_argument(
        "--target-obstacle",
        action="append",
        type=_parse_rect,
        default=None,
        help="Map-update target-object rectangle treated as blocked.",
    )
    parser.add_argument(
        "--occluded",
        action="append",
        type=_parse_rect,
        default=None,
        help="Map-update occluded rectangle treated as blocked.",
    )
    parser.add_argument(
        "--unknown",
        action="append",
        type=_parse_rect,
        default=None,
        help="Unknown rectangle treated conservatively as blocked.",
    )
    parser.add_argument("--inflation-m", type=float, default=DEFAULT_OBSTACLE_INFLATION_M)
    parser.add_argument("--map-update-hz", type=float, default=DEFAULT_MAP_UPDATE_HZ)
    parser.add_argument("--occlusion-shadow-m", type=float, default=0.0)
    parser.add_argument("--occlusion-direction", choices=("none", "+x", "-x", "+z", "-z"), default="none")
    parser.add_argument("--mu-min", type=float, default=DEFAULT_MU_MIN)
    parser.add_argument("--w", "--manip-weight", dest="manip_weight", type=float, default=DEFAULT_MANIP_WEIGHT)
    parser.add_argument("--kappa", "--manip-kappa", dest="manip_kappa", type=float, default=DEFAULT_MANIP_KAPPA)
    parser.add_argument(
        "--mu-safe",
        dest="manip_mu_safe",
        type=float,
        default=DEFAULT_MANIP_MU_SAFE,
        help="Absolute mu_safe threshold. If <= 0, percentile-based mu_safe is used.",
    )
    parser.add_argument(
        "--mu-safe-percentile",
        dest="manip_mu_safe_percentile",
        type=float,
        default=DEFAULT_MANIP_MU_SAFE_PERCENTILE,
    )
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--v-ref", type=float, default=0.04)
    parser.add_argument("--terminal-hold-s", type=float, default=1.0)
    parser.add_argument("--t-accel", type=float, default=0.30)
    parser.add_argument("--t-decel", type=float, default=0.30)
    parser.add_argument("--gap-target-m", type=float, default=DEFAULT_TRACKING_GAP_M)
    parser.add_argument("--terminal-gap-eps-m", type=float, default=DEFAULT_TRACKING_GAP_M)
    parser.add_argument("--k-gap-per-s", type=float, default=8.0)
    parser.add_argument("--sdot-max-factor", type=float, default=1.25)
    parser.add_argument("--sdot-floor-factor", type=float, default=0.05)
    parser.add_argument(
        "--goal-projection-factor",
        type=float,
        default=DEFAULT_GOAL_PROJECTION_FACTOR,
        help="Allowed start/goal projection distance in map-resolution units.",
    )
    parser.add_argument("--start-x", type=float, default=None)
    parser.add_argument("--start-z", type=float, default=None)
    parser.add_argument("--goal-x", type=float, default=None)
    parser.add_argument("--goal-z", type=float, default=None)
    parser.add_argument(
        "--out",
        default=None,
        help=(
            "Output directory. For --scenario all, this is a branch root. "
            "For a single scenario, this is the exact case directory."
        ),
    )
    parser.add_argument("--no-plot", action="store_true")
    return parser


def build_snapshots(args: argparse.Namespace,
                    scenario: str,
                    planner: PathPlanning,
                    updater: MapUpdateLayer):
    handle = planner.load_map(args.map_path)
    clear_snapshot = updater.build(
        handle,
        mu_min=float(args.mu_min),
        request=MapUpdateRequest(
            inflation_m=float(args.inflation_m),
            map_update_hz=float(args.map_update_hz),
            source="clear",
        ),
    )
    if args.obstacle is None:
        occupied = (DEFAULT_OBSTACLE,) if scenario == "changed" else ()
    else:
        occupied = tuple(args.obstacle) if scenario == "changed" else ()
    active_snapshot = updater.build(
        handle,
        mu_min=float(args.mu_min),
        request=MapUpdateRequest(
            occupied_rects=occupied,
            target_rects=tuple(args.target_obstacle or ()),
            occluded_rects=tuple(args.occluded or ()),
            unknown_rects=tuple(args.unknown or ()),
            inflation_m=float(args.inflation_m),
            map_update_hz=float(args.map_update_hz),
            occlusion_shadow_m=float(args.occlusion_shadow_m),
            occlusion_direction=str(args.occlusion_direction),
            source=scenario,
        ),
    )
    return clear_snapshot, active_snapshot


def build_request(args: argparse.Namespace,
                  scenario: str,
                  backend: str,
                  planner: PathPlanning,
                  updater: MapUpdateLayer) -> MissionRequest:
    clear_snapshot, active_snapshot = build_snapshots(args, scenario, planner, updater)
    return MissionRequest(
        scenario=scenario,
        clear_snapshot=clear_snapshot,
        active_snapshot=active_snapshot,
        start_xz=_parse_xz(args.start_x, args.start_z, "start"),
        goal_xz=_parse_xz(args.goal_x, args.goal_z, "goal"),
        cost_mode=str(args.cost_mode),
        manip_weight=float(args.manip_weight),
        manip_kappa=float(args.manip_kappa),
        manip_mu_safe=float(args.manip_mu_safe),
        manip_mu_safe_percentile=float(args.manip_mu_safe_percentile),
        reference_backend=backend,
        scheduler_mode=str(args.scheduler_mode),
        dt=float(args.dt),
        v_ref=float(args.v_ref),
        terminal_hold_s=float(args.terminal_hold_s),
        t_accel=float(args.t_accel),
        t_decel=float(args.t_decel),
        gap_target_m=float(args.gap_target_m),
        terminal_gap_eps_m=float(args.terminal_gap_eps_m),
        k_gap_per_s=float(args.k_gap_per_s),
        sdot_max_factor=float(args.sdot_max_factor),
        sdot_floor_factor=float(args.sdot_floor_factor),
        goal_projection_factor=float(args.goal_projection_factor),
        make_plots=not bool(args.no_plot),
        write_artifacts=True,
    )


def _single_case_output_root(args: argparse.Namespace) -> Optional[str]:
    if args.out is not None:
        return None
    return os.path.join(PROJECT_ROOT, "path", str(args.cost_mode))


def _all_case_output_root(args: argparse.Namespace) -> Optional[str]:
    if args.out is not None:
        return os.path.abspath(args.out)
    return os.path.join(PROJECT_ROOT, "path", str(args.cost_mode))


def _format_optional_mm(value: Any) -> str:
    if value is None:
        return "N/A"
    return f"{float(value) * 1000.0:.2f}mm"


def _format_optional_float(value: Any, suffix: str = "", precision: int = 3) -> str:
    if value is None:
        return "N/A"
    return f"{float(value):.{precision}f}{suffix}"


def print_summary(summary: Dict[str, Any]) -> None:
    case = summary["case"]
    plan = case["plan"]
    ref = case["reference"]
    ctrl = case["control"]
    policy = case["candidate_policy"]
    final_pos = _format_optional_mm(ctrl.get("final_pos_err_m"))
    final_rot = _format_optional_float(ctrl.get("final_rot_err_deg"), "deg", 2)
    lq = _format_optional_float(ctrl.get("L_q"), "", 3)
    rho = _format_optional_float(ctrl.get("rho_ref"), "", 3)
    print(f"[control_module:{summary.get('cost_mode')}] run_dir={summary['run_dir']}")
    print(
        f"[{case['label']}] "
        f"found={plan['found']} valid_grasp={plan['valid_grasp']} "
        f"collision_free={plan['collision_free']} "
        f"cost={summary.get('cost_mode')} "
        f"w={float(summary.get('manip_weight')):.3f} "
        f"kappa={float(summary.get('manip_kappa')):.3f} "
        f"len={float(plan['path_length_m']):.4f} "
        f"nodes={int(plan['nodes_expanded'])} "
        f"ref={ref.get('reference_backend')} "
        f"ref_feasible={ref.get('reference_samples_feasible', True)} "
        f"basic={ctrl['success_basic']} strong={ctrl['success_strong']} "
        f"final={final_pos}/{final_rot} "
        f"Lq={lq} rho={rho} "
        f"accepted={policy['candidate_accepted']} "
        f"reason={policy['reject_reason']}"
    )


def run_all(args: argparse.Namespace) -> int:
    module = ControlModule(output_root=_all_case_output_root(args))
    planner = PathPlanning(map_path=args.map_path)
    updater = MapUpdateLayer()
    cases = (
        ("clear", "linear"),
        ("clear", "c2_quintic"),
        ("changed", "linear"),
        ("changed", "c2_quintic"),
    )
    for scenario, backend in cases:
        summary = module.run(build_request(args, scenario, backend, planner, updater))
        print_summary(summary)
    return 0


def run_single(args: argparse.Namespace) -> int:
    output_root = _single_case_output_root(args)
    module = ControlModule(output_root=output_root)
    planner = PathPlanning(map_path=args.map_path)
    updater = MapUpdateLayer()
    summary = module.run(
        build_request(args, args.scenario, args.reference_backend, planner, updater),
        out_dir=args.out,
    )
    print_summary(summary)
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.scenario == "all":
        return run_all(args)
    return run_single(args)


if __name__ == "__main__":
    raise SystemExit(main())
