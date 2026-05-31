from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import numpy as np

from .execution_supervisor import ExecutionCandidate


class ControlArtifactBridgeError(ValueError):
    """Raised when ControlModule artifacts cannot form an execution candidate."""


def execution_candidate_from_control_artifacts(
    control_out_dir: str | Path,
    *,
    require_accepted: bool = True,
    execution_dt_s: float | None = None,
) -> ExecutionCandidate:
    """Convert one ControlModule artifact directory into an ExecutionCandidate.

    The control module writes the geometric reference and the DLS rollout as
    separate CSVs.  The execution supervisor needs q, s, xz, and timing on one
    common support.  We use the episode support as the authority for q and s,
    then interpolate the accepted reference xz table onto those s samples.
    """

    out_dir = Path(control_out_dir)
    summary = _read_summary(out_dir / "summary.json")
    metrics = _candidate_metrics(summary)
    if require_accepted and not bool(metrics.get("candidate_accepted", False)):
        raise ControlArtifactBridgeError("ControlModule candidate is not accepted")

    episode = _read_episode(out_dir / "episode.csv")
    reference = _read_reference(out_dir / "reference.csv")
    if episode["q"].shape[0] == 0:
        raise ControlArtifactBridgeError("episode.csv has no q2/q3/q5 trajectory")
    if reference["xz"].shape[0] < 2:
        raise ControlArtifactBridgeError("reference.csv has fewer than two xz samples")

    s_table = episode["s"]
    xz_table = _reference_xz_at_s(reference["s"], reference["xz"], s_table)
    explicit_dt = execution_dt_s is not None
    dt_s = _execution_dt(execution_dt_s) if explicit_dt else _summary_dt(summary)
    summary_dt = _summary_dt(summary)
    timestamps_s = None
    if dt_s is None:
        timestamps_s = _episode_timestamps_from_completion(summary, episode["q"].shape[0])
    # When no explicit execution dt is supplied, the candidate inherits the
    # control internal dt (e.g. 0.01 s = 100 Hz). That cadence is a simulation
    # rate, not a hardware command rate; sending it verbatim would flood the
    # arm controller. The value is never changed silently here, but the fallback
    # is surfaced so callers can retime explicitly via
    # execution_candidate_from_control_artifacts(..., execution_dt_s=0.1).
    dt_fallback_to_control_dt = (not explicit_dt) and dt_s is not None
    metadata = {
        "source": "control_module_artifacts",
        "control_out_dir": str(out_dir),
        "active_joint_order": ["q2", "q3", "q5"],
        "reference_support": "episode_s_interpolated_reference_xz",
        "summary_dt_s": summary_dt,
        "execution_dt_s": dt_s,
        "execution_dt_explicit": bool(explicit_dt),
        "execution_dt_fallback_to_control_dt": bool(dt_fallback_to_control_dt),
        "control_n_steps": int(episode["q"].shape[0]),
    }
    if dt_fallback_to_control_dt:
        hz = (1.0 / dt_s) if dt_s else float("nan")
        metadata["execution_dt_warning"] = (
            f"execution_dt_s not provided; using control internal dt {dt_s} s "
            f"(~{hz:.0f} Hz). Pass execution_dt_s (e.g. 0.1) to retime before "
            "hardware execution."
        )
    control = _nested(summary, ("case", "control"), {})
    if isinstance(control, Mapping):
        metadata["control_completion_time_s"] = control.get("completion_time_s")
        metadata["control_peak_qdot"] = control.get("peak_qdot")

    return ExecutionCandidate(
        q_active_trajectory=episode["q"],
        s_table=s_table,
        xz_table=xz_table,
        dt_s=dt_s,
        timestamps_s=timestamps_s,
        candidate_metrics=metrics,
        metadata=metadata,
    )


def _read_summary(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise ControlArtifactBridgeError(f"missing summary artifact: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ControlArtifactBridgeError(f"cannot parse summary artifact: {path}") from exc
    if not isinstance(data, Mapping):
        raise ControlArtifactBridgeError("summary artifact must be a JSON object")
    return dict(data)


def _read_reference(path: Path) -> Dict[str, np.ndarray]:
    if not path.exists():
        raise ControlArtifactBridgeError(f"missing reference artifact: {path}")
    s_vals: list[float] = []
    xz_vals: list[tuple[float, float]] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                s_vals.append(float(row["s"]))
                xz_vals.append((float(row["x"]), float(row["z"])))
            except Exception as exc:
                raise ControlArtifactBridgeError(f"malformed reference row in {path}") from exc
    s = np.asarray(s_vals, dtype=np.float64).reshape(-1)
    xz = np.asarray(xz_vals, dtype=np.float64).reshape(-1, 2) if xz_vals else np.zeros((0, 2), dtype=np.float64)
    if s.shape[0] != xz.shape[0]:
        raise ControlArtifactBridgeError("reference s/xz length mismatch")
    if s.shape[0] > 1 and np.any(np.diff(s) < -1.0e-12):
        raise ControlArtifactBridgeError("reference s must be monotone nondecreasing")
    return {"s": s, "xz": xz}


def _read_episode(path: Path) -> Dict[str, np.ndarray]:
    if not path.exists():
        raise ControlArtifactBridgeError(f"missing episode artifact: {path}")
    s_vals: list[float] = []
    q_vals: list[list[float]] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fields = set(reader.fieldnames or [])
        required = {"s", "q2", "q3", "q5"}
        if not required.issubset(fields):
            raise ControlArtifactBridgeError(f"episode.csv missing required columns: {sorted(required - fields)}")
        for row in reader:
            try:
                s_vals.append(float(row["s"]))
                q_vals.append([float(row["q2"]), float(row["q3"]), float(row["q5"])])
            except Exception as exc:
                raise ControlArtifactBridgeError(f"malformed episode row in {path}") from exc
    q = np.asarray(q_vals, dtype=np.float64).reshape(-1, 3) if q_vals else np.zeros((0, 3), dtype=np.float64)
    s = np.asarray(s_vals, dtype=np.float64).reshape(-1)
    if s.shape[0] != q.shape[0]:
        raise ControlArtifactBridgeError("episode s/q length mismatch")
    if q.size and not np.all(np.isfinite(q)):
        raise ControlArtifactBridgeError("episode q trajectory contains non-finite values")
    return {"s": s, "q": q}


def _reference_xz_at_s(reference_s: np.ndarray, reference_xz: np.ndarray, s_query: np.ndarray) -> np.ndarray:
    if reference_s.shape[0] < 2:
        raise ControlArtifactBridgeError("cannot interpolate reference with fewer than two samples")
    s = np.asarray(reference_s, dtype=np.float64).reshape(-1)
    xz = np.asarray(reference_xz, dtype=np.float64).reshape(-1, 2)
    query = np.asarray(s_query, dtype=np.float64).reshape(-1)
    if query.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float64)
    q_clip = np.clip(query, float(s[0]), float(s[-1]))
    return np.stack(
        [
            np.interp(q_clip, s, xz[:, 0]),
            np.interp(q_clip, s, xz[:, 1]),
        ],
        axis=1,
    )


def _candidate_metrics(summary: Mapping[str, Any]) -> Dict[str, object]:
    case = _nested(summary, ("case",), {})
    plan = _nested(summary, ("case", "plan"), {})
    reference = _nested(summary, ("case", "reference"), {})
    control = _nested(summary, ("case", "control"), {})
    policy = _nested(summary, ("case", "candidate_policy"), {})
    metrics: Dict[str, object] = {}
    if isinstance(policy, Mapping):
        metrics.update(policy)
    if isinstance(plan, Mapping):
        metrics.update(
            {
                "plan_found": bool(plan.get("found", False)),
                "valid_grasp": bool(plan.get("valid_grasp", False)),
                "collision_free": bool(plan.get("collision_free", False)),
                "path_length_m": plan.get("path_length_m"),
                "waypoint_count": plan.get("waypoint_count"),
                "invalid_reason_code": plan.get("invalid_reason_code"),
            }
        )
    if isinstance(reference, Mapping):
        metrics["reference_samples_feasible"] = reference.get("reference_samples_feasible")
    if isinstance(control, Mapping):
        metrics["control_success"] = bool(control.get("success", False))
        metrics["control_n_steps"] = control.get("n_steps")
    if "candidate_accepted" not in metrics:
        metrics["candidate_accepted"] = False
    if not isinstance(case, Mapping):
        metrics["bridge_warning"] = "summary.case_missing"
    return metrics


def _summary_dt(summary: Mapping[str, Any]) -> float | None:
    try:
        dt = float(summary.get("dt"))
    except Exception:
        return None
    if not np.isfinite(dt) or dt <= 0.0:
        return None
    return dt


def _execution_dt(value: float | None) -> float:
    try:
        dt = float(value)
    except Exception as exc:
        raise ControlArtifactBridgeError("execution_dt_s must be numeric") from exc
    if not np.isfinite(dt) or dt <= 0.0:
        raise ControlArtifactBridgeError("execution_dt_s must be positive and finite")
    return dt


def _episode_timestamps_from_completion(summary: Mapping[str, Any], n: int) -> np.ndarray | None:
    control = _nested(summary, ("case", "control"), {})
    if not isinstance(control, Mapping) or n <= 0:
        return None
    try:
        completion = float(control.get("completion_time_s"))
    except Exception:
        return None
    if not np.isfinite(completion) or completion <= 0.0:
        return None
    if n == 1:
        return np.asarray([0.0], dtype=np.float64)
    return np.linspace(0.0, completion, n, dtype=np.float64)


def _nested(data: Mapping[str, Any], path: Sequence[str], default: Any) -> Any:
    cur: Any = data
    for key in path:
        if not isinstance(cur, Mapping) or key not in cur:
            return default
        cur = cur[key]
    return cur
