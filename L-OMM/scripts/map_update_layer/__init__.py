#!/usr/bin/env python3
"""Map update layer package for active-manifold planning."""

from .active_map_snapshot import ActiveMapSnapshot
from .blockage_classifier import BlockageReport, ReferenceBlockageClassifier
from .camera_frame_bridge import CameraBridgeResult, CameraFrameMapUpdateBridge
from .cell_projection_occupancy import (
    CellProjectionOccupancyConfig,
    CellProjectionOccupancyEstimator,
    CellProjectionOccupancyEvidence,
)
from .depth_geometry_evidence import DepthGeometryConfig, DepthGeometryEvidence, DepthGeometryEvidenceBuilder
from .dynamic_frame import SyntheticDynamicFrame, SyntheticDynamicScenario
from .fov_mask import compute_task_plane_fov_mask, rasterize_xz_polygon
from .layered_mask_result import LayeredMaskResult
from .lightweight_supervisor import LightweightSupervisorDryRun
from .map_update_layer import MapUpdateLayer
from .map_update_request import MapUpdateRequest
from .offline_perception_adapter import OfflineDetection, OfflinePerceptionToMapAdapter
from .path_collision_monitor import CollisionReport, PathCollisionMonitor
from .perception_to_map import CameraIntrinsics, Detection2D, PerceptionToMapAdapter
from .rect_rasterizer import RectRasterizer
from .reference_snapshot_diff import DiffReport, ReferenceSnapshotDiff
from .recovery_contract import RecoveryHandoffContract, RecoveryRequest, SyntheticRecoveryResponse
from .runtime_contract import NextAction, PlanOutcome, RuntimeDecision, RuntimeEventClass
from .runtime_integration_checklist import JetsonRuntimeIntegrationChecklist
from .snapshot_validator import SnapshotValidator
from .sticky_map_manager import StickyMapManager, StickyMapParams, StickyMapState
from .synthetic_cases import SyntheticMapUpdateCases
from .synthetic_dynamic_sequences import SyntheticDynamicSequences

__all__ = [
    "ActiveMapSnapshot",
    "BlockageReport",
    "CameraBridgeResult",
    "CameraFrameMapUpdateBridge",
    "CameraIntrinsics",
    "CellProjectionOccupancyConfig",
    "CellProjectionOccupancyEstimator",
    "CellProjectionOccupancyEvidence",
    "DepthGeometryConfig",
    "DepthGeometryEvidence",
    "DepthGeometryEvidenceBuilder",
    "DiffReport",
    "Detection2D",
    "JetsonRuntimeIntegrationChecklist",
    "LayeredMaskResult",
    "LightweightSupervisorDryRun",
    "MapUpdateLayer",
    "MapUpdateRequest",
    "NextAction",
    "OfflineDetection",
    "OfflinePerceptionToMapAdapter",
    "PerceptionToMapAdapter",
    "PlanOutcome",
    "RectRasterizer",
    "ReferenceBlockageClassifier",
    "ReferenceSnapshotDiff",
    "RecoveryHandoffContract",
    "RecoveryRequest",
    "RuntimeDecision",
    "RuntimeEventClass",
    "SnapshotValidator",
    "SyntheticRecoveryResponse",
    "SyntheticDynamicFrame",
    "SyntheticDynamicScenario",
    "SyntheticDynamicSequences",
    "SyntheticMapUpdateCases",
    "compute_task_plane_fov_mask",
    "rasterize_xz_polygon",
]
