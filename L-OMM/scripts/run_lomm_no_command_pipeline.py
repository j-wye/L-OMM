#!/usr/bin/env python3
"""Top-level entry for the no-command real-map / cell-projection orchestrator.

This is the only command-line entry point for the no-command runtime seam. By
default it runs the REAL baked active map through the real ControlModule with the
PathCollisionMonitor enabled (REAL_MAP_CONTROLMODULE_NO_COMMAND_ORCHESTRATOR_
CLOSURE). The ``--synthetic-map`` flag selects the original fast synthetic seam.

It runs no Jetson, ROS2, camera, controller, action client, arm, base, gripper, or
physical robot command. All RGB-D depth and all camera transforms are synthetic
and deterministic; only the map and the planning/control stack are real.
"""
from __future__ import annotations

import argparse
import json
import os
import sys


SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
for _path in (SCRIPTS_DIR, CONTROL_MODULE_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the deterministic no-command sticky + cell-projection runtime seam."
    )
    parser.add_argument(
        "--out",
        default=None,
        help="Output directory for the JSON report artifact (defaults depend on mode).",
    )
    parser.add_argument(
        "--synthetic-map",
        action="store_true",
        help="Run the original fast synthetic seam instead of the real baked map.",
    )
    parser.add_argument(
        "--map-path",
        default=None,
        help="Override the baked map .npy path (real mode only).",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.synthetic_map:
        from map_update_layer.experiments.no_command_orchestrator import run_orchestrator

        out = args.out or "path/no_command_orchestrator"
        result = run_orchestrator(out=str(out))
        print(result.status)
        print(json.dumps(result.invariants, indent=2, ensure_ascii=False))
        return 0 if result.invariants.get("all_pass", False) else 1

    from map_update_layer.experiments.no_command_orchestrator import run_real_map_orchestrator

    out = args.out or "path/no_command_orchestrator_real"
    report = run_real_map_orchestrator(out=str(out), map_path=args.map_path)
    print(report["status"])
    print(json.dumps(report["invariants"], indent=2, ensure_ascii=False))
    return 0 if report["invariants"].get("all_pass", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
