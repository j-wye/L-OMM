# Jetson Sensitivity Sweep Protocol

Runtime mode: `MAP_UPDATE_LAYER_JETSON_NO_COMMAND_SENSITIVITY_SWEEP_PREP`

This protocol is no-command and read-only with respect to the robot.  It may
run synthetic map-update sweeps, camera-only RGB-D contract checks, and metric
collection scripts.  It must not create robot command publishers, send action
goals, publish `/cmd_vel`, send arm trajectories, send gripper commands, run
recovery actions, or run canonical accepted-candidate reruns.

## 1. Environment

Every ROS-related command must use:

```bash
export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/orin/vlm/install/setup.bash
```

The standard remote shell wrapper is:

```bash
ssh jetson-orin "cd ~/vlm/src && bash -lc 'export ROS_DOMAIN_ID=15; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; source /opt/ros/humble/setup.bash; source /home/orin/vlm/install/setup.bash; <command>'"
```

## 2. Execution Order

### Step 1: Camera-only RGB-D contract check

Purpose: verify the live CameraPreprocessor RGB-D ingress without adding a new
camera subscriber.

```bash
ssh jetson-orin "cd ~/vlm/src && bash -lc 'export ROS_DOMAIN_ID=15; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; source /opt/ros/humble/setup.bash; source /home/orin/vlm/install/setup.bash; python3 L-OMM/scripts/map_update_layer/experiments/camera_only_rgbd_contract_check.py --out path/map_update_layer_jetson_sensitivity_sweep_prep/jetson_live/camera_only_rgbd_contract_check --iterations 30 --live-samples 10 --live-timeout-sec 12'"
```

### Step 2: StickyMapManager sensitivity sweep

Purpose: synthetic FOV/sticky-state sweep over free-frame threshold, unknown
persistence parameter recording, and inflation.

```bash
ssh jetson-orin "cd ~/vlm/src && bash -lc 'python3 L-OMM/scripts/map_update_layer/experiments/sticky_map_manager_sensitivity_sweep.py --out path/map_update_layer_jetson_sensitivity_sweep_prep/jetson_live/sticky_map_manager_sensitivity_sweep --n-frames 30 --seed 42'"
```

### Step 3: PathCollisionMonitor metrics

Purpose: synthetic trigger latency, confusion matrix, and per-call check
latency measurement.

```bash
ssh jetson-orin "cd ~/vlm/src && bash -lc 'python3 L-OMM/scripts/map_update_layer/experiments/path_collision_monitor_metrics.py --out path/map_update_layer_jetson_sensitivity_sweep_prep/jetson_live/path_collision_monitor_metrics --seed 42 --frames 24 --iterations 100'"
```

### Step 4: Depth cap and lateral crop sweep

Purpose: synthetic DEPTH_RELEVANT_MAX_M and CROP_HALF_PIXELS load-reduction
measurement.

```bash
ssh jetson-orin "cd ~/vlm/src && bash -lc 'python3 L-OMM/scripts/map_update_layer/experiments/depth_cap_lateral_crop_sweep.py --out path/map_update_layer_jetson_sensitivity_sweep_prep/jetson_live/depth_cap_lateral_crop_sweep --seed 42'"
```

### Step 5: Latency sweep with prefilter extension

Purpose: Jetson CPU-side synthetic adapter timing with prefilter ON/OFF and
sticky/path monitor latency columns.

```bash
ssh jetson-orin "cd ~/vlm/src && bash -lc 'python3 L-OMM/scripts/map_update_layer/experiments/latency_sweep.py --out path/map_update_layer_jetson_sensitivity_sweep_prep/jetson_live/latency_sweep_prefilter --iterations 10 --enable-prefilter --depth-relevant-max-m 1.0 --crop-half-pixels 200'"
```

### Step 6: Y-band and depth cap cross-product sweep

Purpose: synthetic task-plane y-band trade-off with DEPTH_RELEVANT_MAX_M as an
additional sweep dimension.

```bash
ssh jetson-orin "cd ~/vlm/src && bash -lc 'python3 L-OMM/scripts/map_update_layer/experiments/yband_param_sweep.py --out path/map_update_layer_jetson_sensitivity_sweep_prep/jetson_live/yband_param_sweep_depth_cap --depth-relevant-max-m 0.5,0.75,1.0,1.25,1.5,2.0 --seed 7'"
```

### Step 7: Replan trigger metrics with PathCollisionMonitor evidence

Purpose: synthetic PathCollisionMonitor trigger evidence, false/missed replan
counts, and trigger latency frames.  Live obstacle insertion remains prohibited
unless a separate owner command explicitly authorizes the physical setup; this
cycle uses the synthetic no-command mode.

```bash
ssh jetson-orin "cd ~/vlm/src && bash -lc 'python3 L-OMM/scripts/map_update_layer/experiments/replan_trigger_metrics.py --out path/map_update_layer_jetson_sensitivity_sweep_prep/jetson_live/replan_trigger_metrics_path_collision --synthetic-path-collision-monitor --synthetic-frames 24'"
```

## 3. Artifact Retrieval

From the Windows checkout, retrieve Jetson artifacts with:

```bash
scp -r jetson-orin:~/vlm/src/path/map_update_layer_jetson_sensitivity_sweep_prep/jetson_live path/map_update_layer_jetson_sensitivity_sweep_prep/stage_i_jetson_execution_protocol/
```

If `scp` is unavailable, use:

```bash
rsync -av jetson-orin:~/vlm/src/path/map_update_layer_jetson_sensitivity_sweep_prep/jetson_live/ path/map_update_layer_jetson_sensitivity_sweep_prep/stage_i_jetson_execution_protocol/jetson_live/
```

## 4. Expected Outputs

- `camera_only_rgbd_contract_check/summary.json`
- `sticky_map_manager_sensitivity_sweep/sensitivity_sweep_summary.json`
- `path_collision_monitor_metrics/confusion_matrix.json`
- `depth_cap_lateral_crop_sweep/depth_cap_lateral_crop_sweep_summary.json`
- `latency_sweep_prefilter/latency_sweep_summary.json`
- `yband_param_sweep_depth_cap/yband_param_sweep_summary.json`
- `replan_trigger_metrics_path_collision/replan_trigger_metrics_summary.json`

Expected runtime is dominated by the camera-only check and plotting.  The
synthetic sweeps are expected to complete within minutes on a Jetson-class CPU.

## 5. Safety Gate

Before reporting results, run static endpoint checks over the changed scripts:

```bash
python3 - <<'PY'
import ast
from pathlib import Path
names = {"create_publisher", "send_goal", "ActionClient", "publish"}
hits = []
for path in Path("L-OMM/scripts/map_update_layer/experiments").glob("*.py"):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            name = ""
            if isinstance(node.func, ast.Name):
                name = node.func.id
            elif isinstance(node.func, ast.Attribute):
                name = node.func.attr
            if name in names:
                hits.append((str(path), name, node.lineno))
print(hits)
raise SystemExit(1 if hits else 0)
PY
```
