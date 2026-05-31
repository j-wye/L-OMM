#!/usr/bin/env python3
"""FULL_NO_COMMAND_ORCHESTRATOR_WITH_STICKY_CELL_PROJECTION.

This module wires every already-closed component into one deterministic,
software-only runtime seam and proves the contract survives the seam:

    synthetic RGB-D depth + explicit T_base_cam(t)
        -> CameraFrameMapUpdateBridge  (speed prefilter + cell-projection union)
        -> StickyMapManager.apply_to_snapshot   (TRAP 1: sticky applied, never the pass-through)
        -> ReferenceSnapshotDiff / ReferenceBlockageClassifier / PathCollisionMonitor
        -> MovingArmExecutionSupervisor (fake action only; no real command)

It runs no Jetson, ROS2, camera, controller, action, arm, base, gripper, or
physical robot command. All depth and all transforms are synthetic/deterministic.

The five seam traps from claude_opinion.md are closed explicitly and asserted:
    TRAP 1: sticky is applied via apply_to_snapshot (sticky_runtime_applied == 1.0).
    TRAP 2: ReferenceSnapshotDiff.capture fires only on accept / successful replan
            (the supervisor's reference_recorder is the diff, so capture is automatic).
    TRAP 3: the sticky FOV source is the cell_projection_fov layer.
    TRAP 4: every ExecutionCandidate is retimed to dt_s == 0.1 (never the 0.01 control dt).
    TRAP 5: require_explicit_transform is True and handle.q_grid is not None.
Plus the box -> camera safety transfer: the target reaches blocked_mask from camera evidence.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

# This module is imported with L-OMM/scripts and L-OMM/scripts/control_module on
# sys.path (see run_lomm_no_command_pipeline.py).  We support both import roots.
try:
    from control_module.constants import (
        DEFAULT_MU_MIN,
        Q_HOME_DEFAULT,
        X_EE_GOAL,
        X_TARGET_CONTACT,
        Y_PLANE_FIXED,
    )
    from control_module.map_handle import MapHandle
except Exception:  # pragma: no cover - direct execution fallback
    from constants import (  # type: ignore
        DEFAULT_MU_MIN,
        Q_HOME_DEFAULT,
        X_EE_GOAL,
        X_TARGET_CONTACT,
        Y_PLANE_FIXED,
    )
    from map_handle import MapHandle  # type: ignore

from map_update_layer.active_map_snapshot import ActiveMapSnapshot
from map_update_layer.blockage_classifier import ReferenceBlockageClassifier
from map_update_layer.camera_frame_bridge import CameraFrameMapUpdateBridge
from map_update_layer.cell_projection_occupancy import (
    CellProjectionOccupancyConfig,
    CellProjectionOccupancyEstimator,
)
from map_update_layer.path_collision_monitor import PathCollisionMonitor
from map_update_layer.perception_to_map import CameraIntrinsics, PerceptionToMapAdapter
from map_update_layer.recovery_contract import RecoveryHandoffContract
from map_update_layer.reference_snapshot_diff import ReferenceSnapshotDiff
from map_update_layer.runtime_contract import RuntimeEventClass
from map_update_layer.snapshot_validator import SnapshotValidator
from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

try:  # control_module/ exposes a control_module.py module that can shadow the package
    from control_module.capsule_collision import CapsuleCollision
except Exception:  # pragma: no cover - direct execution fallback
    from capsule_collision import CapsuleCollision  # type: ignore

try:  # real planner + control facade (bare import avoids the package-shadow trap)
    from path_planning import PathPlanning  # type: ignore
    from control_module import ControlModule  # type: ignore
    from mission_request import MissionRequest  # type: ignore
except Exception:  # pragma: no cover - package import fallback
    from control_module.path_planning import PathPlanning  # type: ignore
    from control_module.control_module import ControlModule  # type: ignore
    from control_module.mission_request import MissionRequest  # type: ignore

from robot_execution.control_bridge import (
    ControlArtifactBridgeError,
    execution_candidate_from_control_artifacts,
)
from robot_execution.execution_supervisor import ExecutionCandidate, MovingArmExecutionSupervisor
from robot_execution.fake_action_server import FakeFollowJointTrajectoryServer
from robot_execution.joint_mapping import ReducedJointMapper
from robot_execution.ros2_trajectory_client import SafeTrajectoryClient
from robot_execution.safety_gate import ExecutionSafetyConfig, ExecutionSafetyGate
from robot_execution.trajectory_builder import ReducedTrajectoryBuilder


# --- Synthetic-scene geometry constants (all depths < speed-prefilter cap 1.0 m) ---
DEPTH_CAP_GUARD_M = 1.0
SYNTH_BACKGROUND_FREE_M = 0.95  # < cap, > every cell expected depth -> free
EXECUTION_DT_S = 0.1


@dataclass(frozen=True)
class FrameSpec:
    """One synthetic frame in a deterministic dynamic scenario."""

    label: str
    occupied_xz: Tuple[Tuple[float, float], ...] = ()
    unknown_xz: Tuple[Tuple[float, float], ...] = ()
    current_s_fraction: float = 0.0
    expected_event: str = ""
    note: str = ""


@dataclass
class FrameRecord:
    frame_index: int
    label: str
    event_class: str
    replanning_triggered: bool
    next_action: str
    snapshot_valid: float
    sticky_runtime_applied: float
    sticky_blocked_cells: int
    sticky_occupied_cells: int
    cell_fov_cells: int
    cell_free_cells: int
    cell_occupied_cells: int
    cell_occluded_cells: int
    cell_unknown_cells: int
    reference_capture_count: int
    fake_goal_count: int
    fake_cancel_count: int
    real_action_client_created: bool
    recovery_request_present: bool
    target_blocked_by_camera_evidence: bool
    supervisor_action: str
    # G3: PathCollisionMonitor attribution.
    reference_blocked_source: str = "none"
    path_collision_monitor_enabled: bool = False
    # G2/TRAP4: real control-bridge execution-candidate provenance.
    control_artifact_dir: str = ""
    control_candidate_accepted: bool = False
    execution_dt_s: float = 0.0
    execution_dt_fallback: bool = False
    # G4: sticky unknown persistence visibility.
    sticky_unknown_cells: int = 0
    # G5: target evidence source attribution (not just a boolean).
    target_source_detection: bool = False
    target_source_cell_projection: bool = False
    target_source_sticky: bool = False
    target_source_inflation: bool = False
    # G6: connected-component bounding-rect over-cover magnitude on the real map.
    overcover_occupied_cells: int = 0
    overcover_occluded_cells: int = 0
    overcover_unknown_cells: int = 0
    overcover_source_cells: int = 0
    # H1: cumulative count of actual ControlModule.run invocations up to and including
    # this frame (proves a same-snapshot retry frame burns no new replan compute).
    control_run_invocations: int = 0
    # H2 (B(b) claim precision): the semantic detection target_rect footprint actually
    # produced under the fronto-parallel camera, with the image-space y-band prefilter
    # outcome, documenting why target_source_detection is False (detection emitted but
    # deterministically dropped before rect generation) while cell-projection blocks it.
    detection_target_rect_count: int = 0
    detection_prefilter_kept: int = 0
    target_candidate_count: int = 0

    def as_dict(self) -> Dict[str, Any]:
        return {k: _jsonable(v) for k, v in self.__dict__.items()}


# ----------------------------------------------------------------------------
# Synthetic map + depth scene (geometrically consistent with intrinsics and T)
# ----------------------------------------------------------------------------

def build_synthetic_handle() -> MapHandle:
    """A small all-feasible reduced active map with a finite q_grid (TRAP 5)."""

    n_x, n_z = 12, 12
    res = 0.04
    x0, z0 = 0.30, 0.46
    target_y = 0.0  # identity-transform plane membership for the synthetic camera
    mu_grid = np.ones((n_x, n_z), dtype=np.float64)
    pitch_grid = np.zeros((n_x, n_z), dtype=np.float64)
    q_home = np.asarray(Q_HOME_DEFAULT, dtype=np.float64).reshape(3)
    q_grid = np.tile(q_home, (n_x, n_z, 1)).astype(np.float64)
    x_max = x0 + (n_x - 1) * res
    z_max = z0 + (n_z - 1) * res
    z_target = z0 + (n_z // 2) * res
    meta = {
        "resolution_m": res,
        "x_range": [x0, x_max],
        "z_range": [z0, z_max],
        "target_y_fixed_m": target_y,
        "target_height": z_target,
        "target_z": z_target,
        "x_opt_ee_goal_m": float(X_EE_GOAL),
        "target_contact_x_m": float(X_TARGET_CONTACT),
        "tool_offset_m": float(X_TARGET_CONTACT - X_EE_GOAL),
    }
    return MapHandle(
        map_path="synthetic_no_command_orchestrator",
        meta_path="synthetic",
        mu_grid=mu_grid,
        pitch_grid=pitch_grid,
        q_grid=q_grid,
        meta=meta,
        resolution_m=res,
        x0=x0,
        z0=z0,
        target_y=target_y,
        tag="synthetic_no_command_orchestrator",
    )


def synthetic_intrinsics(handle: MapHandle) -> CameraIntrinsics:
    """Intrinsics chosen so plane cells project inside a 320x240 image."""

    return CameraIntrinsics(fx=80.0, fy=80.0, cx=160.0, cy=120.0, width=320, height=240)


def synthetic_T_base_cam() -> np.ndarray:
    """Identity camera pose: plane y=0 cells keep y_cam=0 and project to row cy."""

    return np.eye(4, dtype=np.float64)


# --- Real baked-map geometry (box-free 0.50 active-manifold map) -------------------
REAL_DEPTH_PLANE_M = 0.6  # constant fronto-parallel depth of every active-plane cell
# Cells below the reference corridor that the arm still sweeps but that leave the
# goal grasp reachable (empirically verified accepted-replan offset on the 0.50 map).
ONPATH_REPLAN_Z_OFFSET_CELLS = 6


def build_real_handle(map_path: Optional[str] = None) -> MapHandle:
    """Load the box-free baked active map and assert the q_grid sidecar exists."""

    planner = PathPlanning(map_path=map_path)
    handle = planner.load_map(map_path)
    if handle.q_grid is None:
        raise ValueError(
            f"real baked map lacks q_grid sidecar (PathCollisionMonitor needs it): {handle.map_path}"
        )
    return handle


def frontoparallel_camera(
    handle: MapHandle,
    *,
    depth_plane_m: float = REAL_DEPTH_PLANE_M,
    roi_x: Optional[Tuple[float, float]] = None,
    roi_z: Optional[Tuple[float, float]] = None,
    width: int = 320,
    height: int = 240,
    margin_px: int = 24,
) -> Tuple[CameraIntrinsics, np.ndarray]:
    """Build a deterministic fronto-parallel camera that views the active plane.

    The optical axis (+z_cam) points along base -y, so every cell on the task
    plane (y = target_y) lies at the *same* depth ``depth_plane_m``.  The map's
    (x, z) coordinates then map affinely (no perspective foreshortening) into the
    image, which makes the cell-projection occupancy classification deterministic:
    free background sits behind the plane, an occupied surface sits exactly on it,
    and an invalid pixel becomes an unknown candidate.  Intrinsics are fitted from
    the region of interest so the path/target neighbourhood lands inside the image
    with several pixels of separation per cell (fov_cells > 0 by construction).
    """

    n_x, n_z = handle.shape
    x_max = float(handle.x0) + (n_x - 1) * float(handle.resolution_m)
    z_max = float(handle.z0) + (n_z - 1) * float(handle.resolution_m)
    if roi_x is None:
        roi_x = (float(handle.x0), x_max)
    if roi_z is None:
        # default: the target-height neighbourhood (where the planner operates)
        z_t = float(handle.meta.get("target_height", handle.meta.get("target_z", 0.5)))
        roi_z = (max(float(handle.z0), z_t - 0.05), min(z_max, z_t + 0.35))
    rx0, rx1 = float(roi_x[0]), float(roi_x[1])
    rz0, rz1 = float(roi_z[0]), float(roi_z[1])
    if rx1 <= rx0 or rz1 <= rz0:
        raise ValueError("camera ROI must be non-degenerate")
    depth = float(depth_plane_m)
    fx = (float(width) - 2.0 * margin_px) * depth / (rx1 - rx0)
    fy = (float(height) - 2.0 * margin_px) * depth / (rz1 - rz0)
    cx = float(margin_px) - fx * rx0 / depth
    cy = float(margin_px) - fy * rz0 / depth
    intr = CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=int(width), height=int(height))
    # Camera frame axes expressed in base: x_cam=+x_base, y_cam=+z_base, z_cam=-y_base.
    R = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]], dtype=np.float64)
    t = np.array([0.0, float(handle.target_y) + depth, 0.0], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return intr, T


def read_reference_csv(path: "os.PathLike[str] | str") -> Dict[str, np.ndarray]:
    """Read a ControlModule reference.csv into aligned s/xz arrays."""

    import csv

    p = Path(path)
    s_vals: List[float] = []
    xz_vals: List[Tuple[float, float]] = []
    with p.open("r", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            s_vals.append(float(row["s"]))
            xz_vals.append((float(row["x"]), float(row["z"])))
    s = np.asarray(s_vals, dtype=np.float64).reshape(-1)
    xz = np.asarray(xz_vals, dtype=np.float64).reshape(-1, 2) if xz_vals else np.zeros((0, 2), dtype=np.float64)
    q = np.tile(np.asarray(Q_HOME_DEFAULT, dtype=np.float64).reshape(3), (xz.shape[0], 1))
    return {"s": s, "xz": xz, "q": q}


class SyntheticPlaneDepthRenderer:
    """Render a geometrically consistent depth image for active-plane cells.

    For each surface point on the plane we use the same pinhole projection the
    estimator uses, so the estimator recovers the intended occupancy by
    construction.  Hand-painting masks is never used.
    """

    def __init__(self, handle: MapHandle, intrinsics: CameraIntrinsics, T_base_cam: np.ndarray) -> None:
        self.handle = handle
        self.intr = intrinsics
        self.T = np.asarray(T_base_cam, dtype=np.float64)
        self.height = int(intrinsics.height)
        self.width = int(intrinsics.width)

    def _project(self, x: float, z: float) -> Optional[Tuple[int, int, float]]:
        p_base = np.array([float(x), float(self.handle.target_y), float(z)], dtype=np.float64)
        R = self.T[:3, :3]
        t = self.T[:3, 3]
        p_cam = R.T @ (p_base - t)
        depth = float(p_cam[2])
        if depth <= 0.0:
            return None
        u = int(round(float(self.intr.fx) * float(p_cam[0]) / depth + float(self.intr.cx)))
        v = int(round(float(self.intr.fy) * float(p_cam[1]) / depth + float(self.intr.cy)))
        if u < 0 or u >= self.width or v < 0 or v >= self.height:
            return None
        return u, v, depth

    def render(
        self,
        occupied_xz: Sequence[Tuple[float, float]],
        unknown_xz: Sequence[Tuple[float, float]],
    ) -> np.ndarray:
        """Background = far-but-uncapped free; occupied = exact expected depth; unknown = invalid 0."""

        depth = np.zeros((self.height, self.width), dtype=np.float64)
        # 1) Free background along every column a plane cell can reach.
        n_x, n_z = self.handle.shape
        for ix in range(n_x):
            for iz in range(n_z):
                x = self.handle.x0 + ix * self.handle.resolution_m
                z = self.handle.z0 + iz * self.handle.resolution_m
                hit = self._project(x, z)
                if hit is None:
                    continue
                u, v, _ = hit
                if depth[v, u] == 0.0:
                    depth[v, u] = SYNTH_BACKGROUND_FREE_M
        # 2) Occupied surfaces: nearest (smallest expected depth) wins per pixel.
        for x, z in occupied_xz:
            hit = self._project(float(x), float(z))
            if hit is None:
                continue
            u, v, d = hit
            if depth[v, u] == 0.0 or d < depth[v, u]:
                depth[v, u] = d
        # 3) Unknown: genuine invalid sensor pixels (inside the kept lateral band).
        for x, z in unknown_xz:
            hit = self._project(float(x), float(z))
            if hit is None:
                continue
            u, v, _ = hit
            depth[v, u] = 0.0
        return depth


# ----------------------------------------------------------------------------
# Deterministic synthetic reference (stands in for ControlModule on the small map)
# ----------------------------------------------------------------------------

def build_reference_along_z(handle: MapHandle, ix: int, iz_lo: int, iz_hi: int) -> Dict[str, np.ndarray]:
    """A straight reference at a fixed x column from iz_lo..iz_hi (free corridor)."""

    xs = []
    for iz in range(iz_lo, iz_hi + 1):
        x = handle.x0 + ix * handle.resolution_m
        z = handle.z0 + iz * handle.resolution_m
        xs.append((x, z))
    xz = np.asarray(xs, dtype=np.float64).reshape(-1, 2)
    seg = np.linalg.norm(np.diff(xz, axis=0), axis=1) if xz.shape[0] > 1 else np.zeros(0)
    s = np.concatenate([[0.0], np.cumsum(seg)]) if xz.shape[0] > 1 else np.zeros(1)
    q = np.tile(np.asarray(Q_HOME_DEFAULT, dtype=np.float64).reshape(3), (xz.shape[0], 1))
    return {"xz": xz, "s": s, "q": q}


def execution_candidate_from_reference(ref: Dict[str, np.ndarray]) -> ExecutionCandidate:
    """Build a no-command ExecutionCandidate at the hardware execution dt (TRAP 4)."""

    return ExecutionCandidate(
        q_active_trajectory=ref["q"],
        s_table=ref["s"],
        xz_table=ref["xz"],
        dt_s=EXECUTION_DT_S,
        candidate_metrics={"candidate_accepted": True},
        metadata={"source": "synthetic_no_command_reference", "execution_dt_s": EXECUTION_DT_S},
    )


# ----------------------------------------------------------------------------
# Orchestrator
# ----------------------------------------------------------------------------

@dataclass
class OrchestratorResult:
    status: str
    frames: List[Dict[str, Any]] = field(default_factory=list)
    summary: Dict[str, Any] = field(default_factory=dict)
    invariants: Dict[str, Any] = field(default_factory=dict)


class NoCommandOrchestrator:
    """Run the full no-command seam over a deterministic synthetic frame sequence."""

    def __init__(
        self,
        handle: MapHandle,
        *,
        mu_min: float = 0.0,
        use_path_collision_monitor: bool = True,
        intrinsics: Optional[CameraIntrinsics] = None,
        T_base_cam: Optional[np.ndarray] = None,
        control_module: Optional["ControlModule"] = None,
        control_out_root: Optional[str] = None,
        control_cost_mode: str = "distance",
        control_reference_backend: str = "c2_quintic",
    ) -> None:
        if handle.q_grid is None:  # TRAP 5
            raise ValueError("handle.q_grid is required for the live path-collision monitor")
        self.handle = handle
        self.mu_min = float(mu_min)
        self.use_path_collision_monitor = bool(use_path_collision_monitor)
        # The synthetic seam keeps its identity camera; the real-map run injects a
        # fronto-parallel camera (intrinsics + T) that views the active plane.
        self.intr = intrinsics if intrinsics is not None else synthetic_intrinsics(handle)
        self.T = np.asarray(T_base_cam, dtype=np.float64) if T_base_cam is not None else synthetic_T_base_cam()
        # control_module is None for the synthetic stand-in; a real ControlModule for G2.
        self.control = control_module
        self.control_out_root = Path(control_out_root) if control_out_root is not None else None
        self.control_cost_mode = str(control_cost_mode)
        self.control_reference_backend = str(control_reference_backend)
        self._clear_snapshot_for_control: Optional[ActiveMapSnapshot] = None
        # H1 measurability: incremented only when ControlModule.run is actually called.
        self.control_run_invocations = 0
        self.renderer = SyntheticPlaneDepthRenderer(handle, self.intr, self.T)

        cell_cfg = CellProjectionOccupancyConfig(
            depth_min_m=0.05,
            depth_max_m=DEPTH_CAP_GUARD_M,
            y_plane=float(handle.target_y),
            footprint_radius_px=0,
            transform_slack_m=0.004,
            cell_size_slack_scale=0.5,
        )
        self.bridge = CameraFrameMapUpdateBridge(
            adapter=PerceptionToMapAdapter(
                y_plane=float(handle.target_y),
                delta_y_static_m=0.02,
                sensor_inflation_floor_m=0.0,
                max_active_detections=10,
            ),
            depth_geometry_enabled=False,
            speed_prefilter_enabled=True,           # production default (TRAP polarity context)
            cell_projection=CellProjectionOccupancyEstimator(cell_cfg),
            cell_projection_enabled=True,
            require_explicit_transform=True,        # TRAP 5
        )
        self.validator = SnapshotValidator()
        self.sticky = StickyMapManager(handle=handle, params=StickyMapParams(inflation_m=0.0))
        d_max_task = max(0.2, abs(handle.x0) + handle.shape[0] * handle.resolution_m)
        self.reference_diff = ReferenceSnapshotDiff(
            handle=handle,
            intrinsics=self.intr,
            y_plane=float(handle.target_y),
            d_min=0.05,
            d_max_task=d_max_task,
            tau_diff_cells=3,
            tau_cluster_cells=5,
        )
        # The monitor is always wired into the seam; on the synthetic map the dummy
        # q_grid makes capsule geometry meaningless, so the synthetic scenario routes
        # events through the reference-snapshot diff (the classifier's supported
        # dry-run path).  The real-map report run enables the monitor.
        self.path_monitor = PathCollisionMonitor(CapsuleCollision(), handle=handle)
        self.classifier = ReferenceBlockageClassifier(
            path_collision_monitor=self.path_monitor if self.use_path_collision_monitor else None
        )
        self.recovery = RecoveryHandoffContract()

        gate = ExecutionSafetyGate(ExecutionSafetyConfig(mode="fake_action"))
        self.fake_server = FakeFollowJointTrajectoryServer()
        client = SafeTrajectoryClient(gate, fake_server=self.fake_server)
        self.supervisor = MovingArmExecutionSupervisor(
            trajectory_builder=ReducedTrajectoryBuilder(ReducedJointMapper()),
            trajectory_client=client,
            reference_recorder=self.reference_diff,   # TRAP 2: capture fires through the supervisor
            T_base_cam=self.T,
        )
        self.client = client

        self._sticky_state = None
        self._reference: Optional[Dict[str, np.ndarray]] = None
        self._previous_blocked: Optional[np.ndarray] = None
        self._target_xz: Optional[Tuple[float, float]] = None

    # -- per-frame seam --------------------------------------------------------

    def _build_snapshot(self, frame_spec: FrameSpec, idx: int) -> ActiveMapSnapshot:
        # The target is rendered as an active-plane surface every frame so that both
        # the detection path (depth at the bbox is the true target depth) and the
        # cell-projection path mark it occupied -> blocked (the box->camera transfer).
        occupied = list(frame_spec.occupied_xz)
        if self._target_xz is not None:
            occupied.append(self._target_xz)
        depth = self.renderer.render(occupied, frame_spec.unknown_xz)
        detections: List[Dict[str, Any]] = []
        if self._target_xz is not None:
            hit = self.renderer._project(*self._target_xz)
            if hit is not None:
                u, v, _ = hit
                detections.append({"semantic_type": "target", "bbox": (u, v, u, v), "score": 1.0})
        frame = {"depth": depth, "intrinsics": self.intr, "timestamp_s": float(idx)}
        result = self.bridge.process(
            frame,
            detections,
            self.handle,
            mu_min=self.mu_min,
            T_base_cam=self.T,
            sequence_id=int(idx),
        )
        return result.snapshot

    def _apply_sticky(self, raw_snapshot: ActiveMapSnapshot) -> ActiveMapSnapshot:
        # TRAP 3: the sticky FOV source is the cell-projection FOV layer.
        fov = raw_snapshot.layer_masks.get("cell_projection_fov")
        if fov is None:
            raise ValueError("cell_projection_fov layer missing; cell projection did not run")
        if self._sticky_state is None:
            self._sticky_state = self.sticky.initialize(raw_snapshot)
        else:
            self._sticky_state = self.sticky.update(self._sticky_state, raw_snapshot, current_fov_mask=fov)
        # TRAP 1: explicit sticky application, never the pass-through snapshot.
        return self.sticky.apply_to_snapshot(raw_snapshot, self._sticky_state)

    def _target_blocked_by_camera(self, snapshot: ActiveMapSnapshot) -> bool:
        if self._target_xz is None:
            return False
        ix = int(round((self._target_xz[0] - self.handle.x0) / self.handle.resolution_m))
        iz = int(round((self._target_xz[1] - self.handle.z0) / self.handle.resolution_m))
        n_x, n_z = self.handle.shape
        if not (0 <= ix < n_x and 0 <= iz < n_z):
            return False
        return bool(np.asarray(snapshot.blocked_mask, dtype=bool)[ix, iz])

    # -- run -------------------------------------------------------------------

    def run(self, frames: Sequence[FrameSpec], *, target_xz: Optional[Tuple[float, float]] = None) -> OrchestratorResult:
        self._target_xz = target_xz
        records: List[FrameRecord] = []
        for idx, spec in enumerate(frames):
            raw = self._build_snapshot(spec, idx)
            sticky_snapshot = self._apply_sticky(raw)
            # G1: the standard SnapshotValidator now passes on sticky-applied
            # snapshots (apply_to_snapshot keeps layer_masks["blocked"] == blocked_mask),
            # so the run validates with it directly — no private _core_invariant_ok bypass.
            validation = self.validator.validate(sticky_snapshot)
            snapshot_valid = float(validation.get("snapshot_valid", 0.0)) >= 1.0
            if not snapshot_valid:
                raise ValueError(f"SnapshotValidator rejected sticky snapshot at frame {idx}: {validation}")

            event_class = "NO_REFERENCE"
            replanning = False
            next_action = "await_reference"
            supervisor_action = ""
            recovery_present = False
            report = None
            provenance: Dict[str, Any] = {}

            if self._reference is None:
                candidate, ref, provenance = self._acquire_initial_reference(sticky_snapshot, idx)
                if candidate.dt_s != EXECUTION_DT_S:  # TRAP 4 guard
                    raise ValueError("execution candidate must be retimed to the hardware dt")
                start_result = self.supervisor.start(candidate, sticky_snapshot)
                self._reference = ref
                supervisor_action = start_result.action
                event_class = "INITIAL_ACCEPT"
                next_action = "switch_to_accepted_reference"
            else:
                ref = self._reference
                current_s = float(spec.current_s_fraction) * float(ref["s"][-1] if ref["s"].size else 0.0)
                diff = None
                if self.reference_diff.has_reference:
                    diff = self.reference_diff.evaluate(
                        sticky_snapshot,
                        self.T,
                        current_corridor=ref["xz"],
                        fov_mask_override=raw.layer_masks.get("cell_projection_fov"),
                    )
                report = self.classifier.classify(
                    sticky_snapshot,
                    s_table=ref["s"],
                    xz_table=ref["xz"],
                    current_s_m=current_s,
                    previous_blocked_mask=self._previous_blocked,
                    reference_snapshot_diff=diff,
                    accepted_reference={"xz_path": ref["xz"]},
                )
                event_class = report.event_class
                replanning = bool(report.replanning_triggered)

                if event_class == RuntimeEventClass.CURRENT_POSE_UNSAFE:
                    res = self.supervisor.handle_event(report, snapshot=sticky_snapshot, candidate=None)
                elif replanning and self.supervisor.retry_would_be_prevented(sticky_snapshot):
                    # H1: a same-snapshot retry must NOT burn a ControlModule.run or write a
                    # control artifact dir. We gate BEFORE _make_replan_candidate; the
                    # supervisor's identical hash gate still returns same_snapshot_retry_prevented
                    # for candidate=None, so the event/action outcome is unchanged.
                    res = self.supervisor.handle_event(report, snapshot=sticky_snapshot, candidate=None)
                elif replanning:
                    candidate, provenance = self._make_replan_candidate(sticky_snapshot, idx)
                    res = self.supervisor.handle_event(report, snapshot=sticky_snapshot, candidate=candidate)
                    if candidate is not None and candidate.accepted:
                        self._reference = self._candidate_reference(candidate)
                else:
                    res = self.supervisor.handle_event(report, snapshot=sticky_snapshot, candidate=None)
                supervisor_action = res.action
                next_action = res.action
                recovery_present = res.recovery_request is not None

            self._previous_blocked = np.asarray(sticky_snapshot.blocked_mask, dtype=bool).copy()
            records.append(
                self._record(
                    idx, spec, sticky_snapshot, event_class, replanning, next_action,
                    supervisor_action, recovery_present, snapshot_valid,
                    report=report, provenance=provenance,
                )
            )

        invariants = self._post_run_invariants(records, target_xz)
        status = "GOAL_STATUS: COMPLETE" if invariants["all_pass"] else "GOAL_STATUS: IN_PROGRESS"
        return OrchestratorResult(
            status=status,
            frames=[r.as_dict() for r in records],
            summary=self._summary(records),
            invariants=invariants,
        )

    # -- helpers ---------------------------------------------------------------

    def _free_column(self) -> int:
        return self.handle.shape[0] // 2

    def _replan_candidate(self, snapshot: ActiveMapSnapshot) -> Optional[ExecutionCandidate]:
        # Deterministic synthetic replan: try a column to the left of the original,
        # away from the occluded-shadow over-cover (which extends to higher ix/iz).
        alt = self._free_column() - 3
        if alt < 0:
            return None
        ref = build_reference_along_z(self.handle, ix=alt, iz_lo=2, iz_hi=8)
        blocked = np.asarray(snapshot.blocked_mask, dtype=bool)
        for x, z in ref["xz"]:
            ix = int(round((x - self.handle.x0) / self.handle.resolution_m))
            iz = int(round((z - self.handle.z0) / self.handle.resolution_m))
            if 0 <= ix < blocked.shape[0] and 0 <= iz < blocked.shape[1] and blocked[ix, iz]:
                # alternative corridor also blocked -> reject (UNGRASPABLE path)
                return ExecutionCandidate(
                    q_active_trajectory=ref["q"],
                    s_table=ref["s"],
                    xz_table=ref["xz"],
                    dt_s=EXECUTION_DT_S,
                    candidate_metrics={"candidate_accepted": False, "rejection_reason": "alt_corridor_blocked"},
                    metadata={"source": "synthetic_replan_rejected"},
                )
        return ExecutionCandidate(
            q_active_trajectory=ref["q"],
            s_table=ref["s"],
            xz_table=ref["xz"],
            dt_s=EXECUTION_DT_S,
            candidate_metrics={"candidate_accepted": True},
            metadata={"source": "synthetic_replan_accepted"},
        )

    def _candidate_reference(self, candidate: ExecutionCandidate) -> Dict[str, np.ndarray]:
        return {"xz": candidate.xz_table.copy(), "s": candidate.s_table.copy(), "q": candidate.q_active_trajectory.copy()}

    # -- reference acquisition (synthetic stand-in vs real ControlModule) -------

    def _acquire_initial_reference(
        self, snapshot: ActiveMapSnapshot, idx: int
    ) -> Tuple[ExecutionCandidate, Dict[str, np.ndarray], Dict[str, Any]]:
        if self.control is None:
            ref = build_reference_along_z(self.handle, ix=self._free_column(), iz_lo=2, iz_hi=8)
            return execution_candidate_from_reference(ref), ref, {}
        # G2: real ControlModule.run for the initial accept on the real baked map.
        out_dir = self._control_out_dir(idx, "clear")
        self.control_run_invocations += 1  # H1: count an actual ControlModule.run
        self.control.run(
            MissionRequest(
                scenario="clear",
                active_snapshot=snapshot,
                clear_snapshot=snapshot,
                cost_mode=self.control_cost_mode,
                reference_backend=self.control_reference_backend,
                make_plots=False,
                write_artifacts=True,
            ),
            out_dir=str(out_dir),
        )
        self._clear_snapshot_for_control = snapshot
        candidate = execution_candidate_from_control_artifacts(out_dir, execution_dt_s=EXECUTION_DT_S)
        self._assert_execution_dt(candidate)
        ref = read_reference_csv(out_dir / "reference.csv")
        return candidate, ref, self._control_provenance(candidate, out_dir, accepted=True)

    def _make_replan_candidate(
        self, snapshot: ActiveMapSnapshot, idx: int
    ) -> Tuple[Optional[ExecutionCandidate], Dict[str, Any]]:
        if self.control is None:
            return self._replan_candidate(snapshot), {}
        # G2: real ControlModule.run(scenario="changed") for every replan.
        out_dir = self._control_out_dir(idx, "changed")
        clear = self._clear_snapshot_for_control if self._clear_snapshot_for_control is not None else snapshot
        self.control_run_invocations += 1  # H1: count an actual ControlModule.run
        summary = self.control.run(
            MissionRequest(
                scenario="changed",
                active_snapshot=snapshot,
                clear_snapshot=clear,
                cost_mode=self.control_cost_mode,
                reference_backend=self.control_reference_backend,
                make_plots=False,
                write_artifacts=True,
            ),
            out_dir=str(out_dir),
        )
        policy = (summary.get("case", {}) or {}).get("candidate_policy", {}) or {}
        accepted = bool(policy.get("candidate_accepted", False))
        if accepted:
            candidate = execution_candidate_from_control_artifacts(out_dir, execution_dt_s=EXECUTION_DT_S)
            self._assert_execution_dt(candidate)
            return candidate, self._control_provenance(candidate, out_dir, accepted=True)
        # Rejected replan: the artifacts may be incomplete (no episode rollout), so we
        # build a minimal, structurally-valid rejected candidate. Its q/s/xz are never
        # sent — handle_event reads only candidate_metrics on the recovery path.
        candidate = self._rejected_candidate(policy)
        return candidate, self._control_provenance(candidate, out_dir, accepted=False)

    def _rejected_candidate(self, policy: Mapping[str, Any]) -> ExecutionCandidate:
        q = np.tile(np.asarray(Q_HOME_DEFAULT, dtype=np.float64).reshape(3), (2, 1))
        gx = float(self.handle.meta.get("x_opt_ee_goal_m", X_EE_GOAL))
        gz = float(self.handle.meta.get("target_height", self.handle.meta.get("target_z", 0.5)))
        xz = np.asarray([[gx, gz], [gx, gz + self.handle.resolution_m]], dtype=np.float64)
        s = np.asarray([0.0, float(self.handle.resolution_m)], dtype=np.float64)
        return ExecutionCandidate(
            q_active_trajectory=q,
            s_table=s,
            xz_table=xz,
            dt_s=EXECUTION_DT_S,
            candidate_metrics={
                "candidate_accepted": False,
                "rejection_reason": str(policy.get("reject_reason", "UNGRASPABLE")),
                "invalid_reason_code": str(policy.get("reject_diagnostic", "") or ""),
            },
            metadata={"source": "control_module_rejected_replan"},
        )

    def _control_out_dir(self, idx: int, kind: str) -> Path:
        root = self.control_out_root if self.control_out_root is not None else Path("path/no_command_orchestrator_real/control")
        out = root / f"frame{idx:02d}_{kind}"
        out.mkdir(parents=True, exist_ok=True)
        return out

    @staticmethod
    def _assert_execution_dt(candidate: ExecutionCandidate) -> None:
        if candidate.dt_s is None or abs(float(candidate.dt_s) - EXECUTION_DT_S) > 1.0e-9:
            raise ValueError(f"control execution candidate dt must be {EXECUTION_DT_S}s, got {candidate.dt_s}")
        if bool(candidate.metadata.get("execution_dt_fallback_to_control_dt", False)):
            raise ValueError("control execution candidate must not silently inherit the control internal dt")

    @staticmethod
    def _control_provenance(candidate: ExecutionCandidate, out_dir: Path, *, accepted: bool) -> Dict[str, Any]:
        md = dict(candidate.metadata)
        return {
            "control_artifact_dir": str(out_dir),
            "control_candidate_accepted": bool(accepted),
            "execution_dt_s": float(candidate.dt_s if candidate.dt_s is not None else 0.0),
            "execution_dt_fallback": bool(md.get("execution_dt_fallback_to_control_dt", False)),
        }

    def _target_source_attribution(self, snapshot: ActiveMapSnapshot) -> Dict[str, bool]:
        # G5: which evidence source(s) put the target cell into blocked_mask.
        out = {"detection": False, "cell_projection": False, "sticky": False, "inflation": False}
        if self._target_xz is None:
            return out
        ix = int(round((self._target_xz[0] - self.handle.x0) / self.handle.resolution_m))
        iz = int(round((self._target_xz[1] - self.handle.z0) / self.handle.resolution_m))
        n_x, n_z = self.handle.shape
        if not (0 <= ix < n_x and 0 <= iz < n_z):
            return out
        layers = snapshot.layer_masks

        def at(name: str) -> bool:
            mask = layers.get(name)
            return bool(mask is not None and np.asarray(mask, dtype=bool)[ix, iz])

        out["detection"] = at("target")
        out["cell_projection"] = at("cell_projection_occupied")
        out["sticky"] = at("sticky_occupied") or at("sticky_blocked")
        out["inflation"] = at("inflation_added") or at("sensor_inflation_added")
        return out

    def _record(self, idx, spec, snapshot, event_class, replanning, next_action, supervisor_action,
                recovery_present, snapshot_valid, *, report=None, provenance=None) -> FrameRecord:
        stats = snapshot.stats
        # cell-projection counters live under the nested adapter_stats block.
        adapter = stats.get("adapter_stats", {})
        if not isinstance(adapter, dict):
            adapter = {}
        provenance = provenance or {}
        client_dict = self.client.as_dict()
        target_source = self._target_source_attribution(snapshot)
        overcover_source = (
            int(adapter.get("cell_projection_occupied_rect_source_cells", 0))
            + int(adapter.get("cell_projection_occluded_rect_source_cells", 0))
            + int(adapter.get("cell_projection_unknown_rect_source_cells", 0))
        )
        return FrameRecord(
            frame_index=idx,
            label=spec.label,
            event_class=event_class,
            replanning_triggered=replanning,
            next_action=next_action,
            snapshot_valid=1.0 if snapshot_valid else 0.0,
            sticky_runtime_applied=float(stats.get("sticky_runtime_applied", 0.0)),
            sticky_blocked_cells=int(stats.get("sticky_blocked_cells", 0.0)),
            sticky_occupied_cells=int(stats.get("sticky_occupied_cells", 0.0)),
            cell_fov_cells=int(adapter.get("cell_projection_fov_cells", 0.0)),
            cell_free_cells=int(adapter.get("cell_projection_free_cells", 0.0)),
            cell_occupied_cells=int(adapter.get("cell_projection_occupied_cells", 0.0)),
            cell_occluded_cells=int(adapter.get("cell_projection_occluded_cells", 0.0)),
            cell_unknown_cells=int(adapter.get("cell_projection_unknown_cells", 0.0)),
            reference_capture_count=int(self.supervisor.reference_capture_count),
            fake_goal_count=int(self.fake_server.goal_count),
            fake_cancel_count=int(self.fake_server.cancel_count),
            real_action_client_created=bool(client_dict.get("real_action_client_created", False)),
            recovery_request_present=bool(recovery_present),
            target_blocked_by_camera_evidence=self._target_blocked_by_camera(snapshot),
            supervisor_action=str(supervisor_action),
            reference_blocked_source=str(getattr(report, "reference_blocked_source", "none")),
            path_collision_monitor_enabled=bool(getattr(report, "path_collision_monitor_enabled", False)),
            control_artifact_dir=str(provenance.get("control_artifact_dir", "")),
            control_candidate_accepted=bool(provenance.get("control_candidate_accepted", False)),
            execution_dt_s=float(provenance.get("execution_dt_s", 0.0)),
            execution_dt_fallback=bool(provenance.get("execution_dt_fallback", False)),
            sticky_unknown_cells=int(stats.get("sticky_unknown_cells", 0.0)),
            target_source_detection=bool(target_source["detection"]),
            target_source_cell_projection=bool(target_source["cell_projection"]),
            target_source_sticky=bool(target_source["sticky"]),
            target_source_inflation=bool(target_source["inflation"]),
            overcover_occupied_cells=int(adapter.get("cell_projection_occupied_rect_overcovered_cells", 0)),
            overcover_occluded_cells=int(adapter.get("cell_projection_occluded_rect_overcovered_cells", 0)),
            overcover_unknown_cells=int(adapter.get("cell_projection_unknown_rect_overcovered_cells", 0)),
            overcover_source_cells=int(overcover_source),
            control_run_invocations=int(self.control_run_invocations),
            # target_rect_count lives on the outer snapshot stats; the prefilter/candidate
            # counters live under the nested adapter_stats block.
            detection_target_rect_count=int(stats.get("target_rect_count", 0)),
            detection_prefilter_kept=int(adapter.get("image_detection_count_prefilter_kept", 0)),
            target_candidate_count=int(adapter.get("target_candidate_count", 0)),
        )

    def _summary(self, records: List[FrameRecord]) -> Dict[str, Any]:
        return {
            "frame_count": len(records),
            "max_reference_capture_count": max((r.reference_capture_count for r in records), default=0),
            "fake_goal_count": int(self.fake_server.goal_count),
            "fake_cancel_count": int(self.fake_server.cancel_count),
            "real_action_client_created": bool(self.client.as_dict().get("real_action_client_created", False)),
            "events": [r.event_class for r in records],
        }

    def _post_run_invariants(self, records: List[FrameRecord], target_xz) -> Dict[str, Any]:
        snapshot_valid_all = all(r.snapshot_valid == 1.0 for r in records)
        sticky_applied_all = all(r.sticky_runtime_applied == 1.0 for r in records)
        dense_nonzero = any((r.cell_occupied_cells + r.cell_occluded_cells + r.cell_free_cells) > 0 for r in records)
        no_real_client = all(not r.real_action_client_created for r in records)
        target_blocked = (target_xz is None) or any(r.target_blocked_by_camera_evidence for r in records)
        checks = {
            "snapshot_valid_every_frame": snapshot_valid_all,            # acceptance 7
            "sticky_applied_every_frame": sticky_applied_all,           # TRAP 1
            "cell_projection_dense_nonzero": dense_nonzero,             # acceptance 11
            "no_real_action_client": no_real_client,                   # acceptance 10
            "target_blocked_by_camera_evidence": target_blocked,       # safety transfer
        }
        return {"all_pass": all(checks.values()), **checks}


def run_orchestrator(*, out: str = "path/no_command_orchestrator") -> OrchestratorResult:
    """Run the default synthetic dynamic scenario and write a JSON artifact."""

    handle = build_synthetic_handle()
    # Synthetic determinism: route events through the reference-snapshot diff. The
    # monitor is still wired into the seam (orch.path_monitor) for the real-map run.
    orch = NoCommandOrchestrator(handle, mu_min=0.0, use_path_collision_monitor=False)

    def cell(ix: int, iz: int) -> Tuple[float, float]:
        return (handle.x0 + ix * handle.resolution_m, handle.z0 + iz * handle.resolution_m)

    free_col = handle.shape[0] // 2          # original reference column (ix=6)
    alt_col = free_col - 3                    # the replan target column (ix=3), left of the shadow
    # The target sits two cells beyond the reference end (iz 2..8), so the reference
    # goal cell stays free while the target itself is blocked by camera evidence.
    target_xz = cell(free_col, 10)
    off_path_corner = (cell(handle.shape[0] - 1, 0),)   # far corner, opposite the alt column
    # Obstacle clusters (>= tau_cluster cells) at deterministic positions:
    goal_block = tuple(cell(free_col + dx, 7 + dz) for dx in (-1, 0, 1) for dz in (-1, 0, 1))   # blocks ix6 goal
    alt_block = tuple(cell(alt_col + dx, 6 + dz) for dx in (-1, 0, 1) for dz in (-1, 0, 1))      # blocks ix3 mid
    pose_block = (cell(alt_col, 2),)                                                              # on the ix3 current pose

    frames = [
        # 1. first accepted candidate -> reference captured -> supervisor.start
        FrameSpec(label="initial_clear", current_s_fraction=0.0, expected_event="INITIAL_ACCEPT"),
        # 2. non-critical off-path change -> no replan, reference unchanged
        FrameSpec(label="noncritical_off_path", occupied_xz=off_path_corner, current_s_fraction=0.0,
                  expected_event="MASK_CHANGED_NONCRITICAL|NO_RELEVANT_CHANGE"),
        # 3. ix6 goal blocked -> replan accepted onto ix8 (switch + capture #2)
        FrameSpec(label="goal_blocked_replan_accept", occupied_xz=goal_block, current_s_fraction=0.0,
                  expected_event="GOAL_CORRIDOR_BLOCKED|REFERENCE_BLOCKED"),
        # 4. ix8 future reference blocked and the only alt is also blocked -> UNGRASPABLE + recovery
        FrameSpec(label="reference_blocked_ungraspable", occupied_xz=alt_block, current_s_fraction=0.0,
                  expected_event="REFERENCE_BLOCKED"),
        # 5. identical blocked snapshot -> same-snapshot retry prevented (no infinite replan)
        FrameSpec(label="same_snapshot_retry", occupied_xz=alt_block, current_s_fraction=0.0,
                  expected_event="REFERENCE_BLOCKED"),
        # 6. current pose itself unsafe -> safety recovery boundary, no replan attempted
        FrameSpec(label="current_pose_unsafe", occupied_xz=pose_block, current_s_fraction=0.0,
                  expected_event="CURRENT_POSE_UNSAFE"),
    ]

    result = orch.run(frames, target_xz=target_xz)
    out_dir = Path(out)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "report.json").write_text(
        json.dumps(
            {"status": result.status, "summary": result.summary, "invariants": result.invariants, "frames": result.frames},
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return result


# ----------------------------------------------------------------------------
# Real baked-map driver (G2-G6): real map + real ControlModule + monitor on
# ----------------------------------------------------------------------------

def _cell_xz(handle: MapHandle, ix: int, iz: int) -> Tuple[float, float]:
    return (float(handle.x0) + ix * float(handle.resolution_m), float(handle.z0) + iz * float(handle.resolution_m))


def _cluster(handle: MapHandle, ix: int, iz: int, radius: int) -> Tuple[Tuple[float, float], ...]:
    n_x, n_z = handle.shape
    out: List[Tuple[float, float]] = []
    for dx in range(-radius, radius + 1):
        for dz in range(-radius, radius + 1):
            cx, cz = ix + dx, iz + dz
            if 0 <= cx < n_x and 0 <= cz < n_z:
                out.append(_cell_xz(handle, cx, cz))
    return tuple(out)


def _xz_to_cell(handle: MapHandle, x: float, z: float) -> Tuple[int, int]:
    return (
        int(round((float(x) - float(handle.x0)) / float(handle.resolution_m))),
        int(round((float(z) - float(handle.z0)) / float(handle.resolution_m))),
    )


def _point_at_fraction(xz: np.ndarray, fraction: float) -> Tuple[float, float]:
    arr = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
    if arr.shape[0] == 0:
        return (0.0, 0.0)
    idx = int(np.clip(round(float(fraction) * (arr.shape[0] - 1)), 0, arr.shape[0] - 1))
    return (float(arr[idx, 0]), float(arr[idx, 1]))


def _camera_in_fov(handle: MapHandle, intr: CameraIntrinsics, T: np.ndarray, ix: int, iz: int) -> bool:
    x, z = _cell_xz(handle, ix, iz)
    p_base = np.array([x, float(handle.target_y), z], dtype=np.float64)
    p_cam = T[:3, :3].T @ (p_base - T[:3, 3])
    d = float(p_cam[2])
    if d <= 0.0:
        return False
    u = int(round(float(intr.fx) * float(p_cam[0]) / d + float(intr.cx)))
    v = int(round(float(intr.fy) * float(p_cam[1]) / d + float(intr.cy)))
    return 0 <= u < int(intr.width) and 0 <= v < int(intr.height)


def _off_path_cell_in_fov(
    handle: MapHandle,
    intr: CameraIntrinsics,
    T: np.ndarray,
    base_feasible: np.ndarray,
    ref_xz: np.ndarray,
) -> Tuple[int, int]:
    """Feasible, in-FOV cell maximally far from the reference corridor (no replan)."""

    feasible = np.asarray(base_feasible, dtype=bool)
    path = np.asarray(ref_xz, dtype=np.float64).reshape(-1, 2)
    n_x, n_z = handle.shape
    best_cell = None
    best_dist = -1.0
    for ix, iz in zip(*np.nonzero(feasible)):
        ix, iz = int(ix), int(iz)
        if not _camera_in_fov(handle, intr, T, ix, iz):
            continue
        edge = min(ix, iz, n_x - 1 - ix, n_z - 1 - iz)
        if edge < 2:
            continue
        x, z = _cell_xz(handle, ix, iz)
        dist = float(np.min(np.linalg.norm(path - np.array([x, z])[None, :], axis=1))) if path.size else 1.0e9
        if dist > best_dist:
            best_dist = dist
            best_cell = (ix, iz)
    if best_cell is None:  # pragma: no cover - real map always has in-FOV feasible cells
        raise ValueError("no in-FOV feasible off-path cell found for the non-critical frame")
    return best_cell


def _real_dynamic_frames(
    handle: MapHandle,
    intr: CameraIntrinsics,
    T: np.ndarray,
    base_feasible: np.ndarray,
    ref_xz: np.ndarray,
) -> Tuple[List[FrameSpec], Tuple[float, float]]:
    """Build the cumulative six-event dynamic scenario from the real reference."""

    start_x, start_z = _point_at_fraction(ref_xz, 0.0)
    goal_x, goal_z = _point_at_fraction(ref_xz, 1.0)
    mid_x, mid_z = _point_at_fraction(ref_xz, 0.5)
    cur_ix, cur_iz = _xz_to_cell(handle, start_x, start_z)
    goal_ix, goal_iz = _xz_to_cell(handle, goal_x, goal_z)
    mid_ix, mid_iz = _xz_to_cell(handle, mid_x, mid_z)
    off_ix, off_iz = _off_path_cell_in_fov(handle, intr, T, base_feasible, ref_xz)

    off_path = _cluster(handle, off_ix, off_iz, radius=1)
    # Accepted-replan obstacle: a small cluster offset BELOW the reference corridor.
    # In the reduced active manifold the arm reaching along the reference sweeps this
    # region (so the PathCollisionMonitor flags a collision -> REFERENCE_BLOCKED), yet it
    # is outside the goal cell's arm volume, so ControlModule(changed) finds an accepted
    # detour to the same grasp. The cluster (not a single cell) forces a wide-enough
    # detour that the re-planned trajectory is proxy-collision-free, so the seven-condition
    # acceptance policy accepts it (empirically verified on the 0.50 baked map).
    on_path = _cluster(handle, mid_ix, max(1, mid_iz - ONPATH_REPLAN_Z_OFFSET_CELLS), radius=1)
    goal_block = _cluster(handle, goal_ix, goal_iz, radius=2)
    pose_block = _cluster(handle, cur_ix, cur_iz, radius=1)
    # the physical target object sits at target_contact_x, target_height (beyond the EE goal)
    tgt_ix, tgt_iz = _xz_to_cell(
        handle,
        float(handle.meta.get("target_contact_x_m", X_TARGET_CONTACT)),
        float(handle.meta.get("target_height", handle.meta.get("target_z", 0.5))),
    )
    target_xz = _cell_xz(handle, tgt_ix, tgt_iz)

    # The accepted-replan obstacle is introduced BEFORE the far off-path obstacle so
    # the re-planned detour candidate is capsule-collision-free (the off-path obstacle,
    # while harmless to connectivity, perturbs the proxy-collision validation and would
    # otherwise make the policy correctly reject the detour).
    frames = [
        FrameSpec(label="F0_clear", current_s_fraction=0.0, expected_event="INITIAL_ACCEPT"),
        FrameSpec(label="F1_onpath_replan_accept", occupied_xz=on_path, current_s_fraction=0.0,
                  expected_event="REFERENCE_BLOCKED"),
        FrameSpec(label="F2_noncritical_off_path", occupied_xz=on_path + off_path, current_s_fraction=0.0,
                  expected_event="MASK_CHANGED_NONCRITICAL|NO_RELEVANT_CHANGE"),
        FrameSpec(label="F3_goal_block_reject", occupied_xz=on_path + off_path + goal_block, current_s_fraction=0.0,
                  expected_event="GOAL_CORRIDOR_BLOCKED"),
        FrameSpec(label="F4_same_snapshot_retry", occupied_xz=on_path + off_path + goal_block, current_s_fraction=0.0,
                  expected_event="GOAL_CORRIDOR_BLOCKED"),
        FrameSpec(label="F5_current_pose_unsafe", occupied_xz=on_path + off_path + goal_block + pose_block,
                  current_s_fraction=0.0, expected_event="CURRENT_POSE_UNSAFE"),
    ]
    return frames, target_xz


def _real_unknown_frames(
    handle: MapHandle,
    intr: CameraIntrinsics,
    T: np.ndarray,
    base_feasible: np.ndarray,
    ref_xz: np.ndarray,
    n_free: int = 5,
) -> List[FrameSpec]:
    """Inject an off-path invalid-depth region, then release it only by repeated free."""

    off_ix, off_iz = _off_path_cell_in_fov(handle, intr, T, base_feasible, ref_xz)
    unknown_region = _cluster(handle, off_ix, off_iz, radius=1)
    frames = [FrameSpec(label="U0_clear", current_s_fraction=0.0, expected_event="INITIAL_ACCEPT")]
    frames.append(
        FrameSpec(label="U1_unknown_injected", unknown_xz=unknown_region, current_s_fraction=0.0,
                  expected_event="MASK_CHANGED_NONCRITICAL|NO_RELEVANT_CHANGE")
    )
    for k in range(n_free + 1):  # one extra free frame to show release is stable
        frames.append(
            FrameSpec(label=f"U{2 + k}_free_over_unknown", current_s_fraction=0.0,
                      expected_event="NO_RELEVANT_CHANGE|MASK_CHANGED_NONCRITICAL")
        )
    return frames


def run_real_map_orchestrator(
    *, out: str = "path/no_command_orchestrator_real", map_path: Optional[str] = None
) -> Dict[str, Any]:
    """G2-G6: run the full no-command seam on the real baked map with real control."""

    out_root = Path(out)
    out_root.mkdir(parents=True, exist_ok=True)
    handle = build_real_handle(map_path)
    intr, T = frontoparallel_camera(handle)

    # Probe ControlModule once to obtain the deterministic clear reference geometry.
    probe_control = ControlModule(output_root=str(out_root / "control_root"))
    probe = NoCommandOrchestrator(
        handle, mu_min=DEFAULT_MU_MIN, use_path_collision_monitor=True,
        intrinsics=intr, T_base_cam=T, control_module=probe_control,
        control_out_root=str(out_root / "control_probe"),
    )
    raw0 = probe._build_snapshot(FrameSpec(label="probe_clear"), 0)
    fov_cells = int(np.count_nonzero(np.asarray(raw0.layer_masks.get("cell_projection_fov"), dtype=bool)))
    if fov_cells <= 0:
        raise ValueError("fronto-parallel camera projects no active-plane cells (fov_cells == 0)")
    sticky0 = probe._apply_sticky(raw0)
    _cand0, ref0, _prov0 = probe._acquire_initial_reference(sticky0, 0)
    ref_xz = ref0["xz"]
    base_feasible = np.asarray(sticky0.base_feasible_mask, dtype=bool)

    # Phase 1: six-event dynamic scenario with real ControlModule replans + monitor on.
    # The target object is NOT rendered as a hard obstacle here: in the reduced active
    # manifold a hard target cell perturbs the re-planned detour's proxy-collision
    # validation and would make the policy correctly reject an otherwise-valid detour.
    # The target box->camera safety transfer (G5) is shown in Phase 2 instead.
    dyn_frames, target_xz = _real_dynamic_frames(handle, intr, T, base_feasible, ref_xz)
    dyn = NoCommandOrchestrator(
        handle, mu_min=DEFAULT_MU_MIN, use_path_collision_monitor=True,
        intrinsics=intr, T_base_cam=T, control_module=ControlModule(output_root=str(out_root / "control_root")),
        control_out_root=str(out_root / "control_dynamic"),
    )
    dyn_result = dyn.run(dyn_frames, target_xz=None)

    # Phase 2: unknown / invalid-depth persistence + repeated-free release (G4), and the
    # target box->camera safety transfer with source attribution (G5) rendered every frame.
    unk_frames = _real_unknown_frames(handle, intr, T, base_feasible, ref_xz)
    unk = NoCommandOrchestrator(
        handle, mu_min=DEFAULT_MU_MIN, use_path_collision_monitor=True,
        intrinsics=intr, T_base_cam=T, control_module=ControlModule(output_root=str(out_root / "control_root")),
        control_out_root=str(out_root / "control_unknown"),
    )
    unk_result = unk.run(unk_frames, target_xz=target_xz)

    invariants = _real_invariants(dyn_result, unk_result, fov_cells=fov_cells)
    status = "GOAL_STATUS: COMPLETE" if invariants["all_pass"] else "GOAL_STATUS: IN_PROGRESS"
    report = {
        "status": status,
        "mode": "real_baked_map",
        "map_path": handle.map_path,
        "map_shape": list(handle.shape),
        "fov_cells_first_frame": fov_cells,
        "camera": {
            "model": "fronto_parallel_optical_axis_minus_y",
            "fx": float(intr.fx), "fy": float(intr.fy), "cx": float(intr.cx), "cy": float(intr.cy),
            "width": int(intr.width), "height": int(intr.height),
            "depth_plane_m": REAL_DEPTH_PLANE_M,
        },
        "invariants": invariants,
        "dynamic_summary": dyn_result.summary,
        "dynamic_frames": dyn_result.frames,
        "unknown_frames": unk_result.frames,
    }
    (out_root / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return report


def _real_invariants(
    dyn_result: OrchestratorResult, unk_result: OrchestratorResult, *, fov_cells: int
) -> Dict[str, Any]:
    dyn = dyn_result.frames
    unk = unk_result.frames

    def actions(frames):
        return [str(f["supervisor_action"]) for f in frames]

    def events(frames):
        return [str(f["event_class"]) for f in frames]

    dyn_actions = actions(dyn)
    dyn_events = events(dyn)
    # G2: real control candidate provenance (dt == 0.1, no fallback) on accepted frames.
    accepted_provenanced = [
        f for f in dyn if f["supervisor_action"] in {"started", "switched_to_accepted_reference"}
    ]
    control_dt_ok = bool(accepted_provenanced) and all(
        abs(float(f["execution_dt_s"]) - EXECUTION_DT_S) < 1e-9 and not bool(f["execution_dt_fallback"])
        and bool(f["control_candidate_accepted"]) and str(f["control_artifact_dir"])
        for f in accepted_provenanced
    )
    # G3: PathCollisionMonitor participated on at least one replan-triggering frame.
    monitor_participated = any(
        str(f["reference_blocked_source"]) == "path_collision_monitor" for f in dyn
    )
    monitor_enabled_any = any(bool(f["path_collision_monitor_enabled"]) for f in dyn)
    # G4: unknown observed, sticky held, then released by repeated free.
    unk_obs = [int(f["cell_unknown_cells"]) for f in unk]
    sticky_unk = [int(f["sticky_unknown_cells"]) for f in unk]
    unknown_observed = any(v > 0 for v in unk_obs)
    inj_idx = next((i for i, v in enumerate(unk_obs) if v > 0), None)
    unknown_held = bool(inj_idx is not None and inj_idx + 1 < len(sticky_unk) and sticky_unk[inj_idx + 1] > 0)
    unknown_released = bool(sticky_unk and sticky_unk[-1] == 0 and any(v > 0 for v in sticky_unk))
    # G5: target attribution recorded with at least one concrete source on some frame
    # (the target box->camera safety transfer runs in the unknown phase).
    target_attributed = any(
        bool(f["target_source_detection"]) or bool(f["target_source_cell_projection"])
        or bool(f["target_source_sticky"]) or bool(f["target_source_inflation"])
        for f in unk
    )
    # G6: over-cover measured (recorded on every frame; magnitude reported separately).
    overcover_recorded = all("overcover_source_cells" in f for f in dyn)
    # snapshot validity + no-command across both phases.
    snapshot_valid_all = all(f["snapshot_valid"] == 1.0 for f in dyn + unk)
    no_real_client = all(not bool(f["real_action_client_created"]) for f in dyn + unk)
    sticky_all = all(f["sticky_runtime_applied"] == 1.0 for f in dyn + unk)
    target_blocked = any(bool(f["target_blocked_by_camera_evidence"]) for f in unk)

    # H1: the same-snapshot retry frame must invoke NO ControlModule.run (the cumulative
    # control_run_invocations count does not increase relative to the prior frame) and must
    # create NO new control artifact dir; and overall fewer ControlModule runs than replan
    # frames (the prevented retry frame is the measurable saving).
    retry_idx = next(
        (i for i, f in enumerate(dyn) if str(f["supervisor_action"]) == "same_snapshot_retry_prevented"),
        None,
    )
    h1_retry_no_control_run = bool(
        retry_idx is not None and retry_idx > 0
        and int(dyn[retry_idx]["control_run_invocations"]) == int(dyn[retry_idx - 1]["control_run_invocations"])
    )
    h1_retry_no_artifact_dir = bool(retry_idx is not None and not str(dyn[retry_idx]["control_artifact_dir"]))
    replan_frames = [i for i, f in enumerate(dyn) if bool(f["replanning_triggered"])]
    control_ran_on_replan = sum(
        1 for i in replan_frames
        if i > 0 and int(dyn[i]["control_run_invocations"]) > int(dyn[i - 1]["control_run_invocations"])
    )
    h1_fewer_runs = bool(replan_frames) and control_ran_on_replan < len(replan_frames)
    # H2 (B(b) claim precision): a target detection IS emitted (target_candidate_count > 0)
    # yet produces NO target_rect (detection_target_rect_count == 0) under the fronto-parallel
    # camera because the image-space y-band prefilter deterministically drops an off-cx
    # on-plane detection. The target blockage is correctly carried by the cell-projection path.
    h2_detection_path_scoped = any(
        int(f["target_candidate_count"]) > 0 and int(f["detection_target_rect_count"]) == 0 for f in unk
    )
    h2_target_carried_by_cell_projection = any(bool(f["target_source_cell_projection"]) for f in unk)

    checks = {
        "fov_cells_positive": fov_cells > 0,
        "initial_accept": "started" in dyn_actions and "INITIAL_ACCEPT" in dyn_events,
        "noncritical_no_replan": "continue_current_reference" in dyn_actions,
        "accepted_replan": "switched_to_accepted_reference" in dyn_actions,
        "rejected_replan_recovery": "recovery_request" in dyn_actions,
        "same_snapshot_retry_prevented": "same_snapshot_retry_prevented" in dyn_actions,
        "current_pose_unsafe": "safety_recovery_boundary" in dyn_actions and "CURRENT_POSE_UNSAFE" in dyn_events,
        "control_dt_retimed_no_fallback": control_dt_ok,
        "path_collision_monitor_participated": monitor_participated,
        "path_collision_monitor_enabled": monitor_enabled_any,
        "unknown_observed": unknown_observed,
        "unknown_persistence_held": unknown_held,
        "unknown_released_by_repeated_free": unknown_released,
        "target_source_attributed": target_attributed,
        "overcover_recorded": overcover_recorded,
        "snapshot_valid_every_frame": snapshot_valid_all,
        "no_real_action_client": no_real_client,
        "sticky_applied_every_frame": sticky_all,
        "target_blocked_by_camera_evidence": target_blocked,
        # H1 retry-guard ordering (measurable saving).
        "h1_retry_no_control_run": h1_retry_no_control_run,
        "h1_retry_no_artifact_dir": h1_retry_no_artifact_dir,
        "h1_fewer_runs_than_replan_frames": h1_fewer_runs,
        # H2 detection-path scope (B(b)).
        "h2_detection_path_scoped": h2_detection_path_scoped,
        "h2_target_carried_by_cell_projection": h2_target_carried_by_cell_projection,
    }
    overcover_total = {
        "occupied": sum(int(f["overcover_occupied_cells"]) for f in dyn),
        "occluded": sum(int(f["overcover_occluded_cells"]) for f in dyn),
        "unknown": sum(int(f["overcover_unknown_cells"]) for f in dyn),
        "source": sum(int(f["overcover_source_cells"]) for f in dyn),
    }
    control_runs = {
        "dynamic_total": int(dyn[-1]["control_run_invocations"]) if dyn else 0,
        "dynamic_replan_frames": len(replan_frames),
        "dynamic_control_ran_on_replan": int(control_ran_on_replan),
        "retry_frame_index": int(retry_idx) if retry_idx is not None else -1,
        "unknown_total": int(unk[-1]["control_run_invocations"]) if unk else 0,
    }
    return {
        "all_pass": all(checks.values()),
        **checks,
        "overcover_total": overcover_total,
        "control_run_invocations": control_runs,
    }


def _jsonable(value: Any) -> Any:
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


if __name__ == "__main__":  # pragma: no cover
    res = run_orchestrator()
    print(res.status)
    print(json.dumps(res.invariants, indent=2, ensure_ascii=False))
