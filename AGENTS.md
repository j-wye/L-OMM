아래의 규칙과 명령을 반드시 읽고 항상 형식에 맞춰서 반드시 한국어로 답변을 진행해. 그리고 답변을 진행할 때 파일 경로가 출력돼서 사용자에게 보이지 않도록 진행해(글을 읽는데 방해되지 않도록)

- 답변에 대한 지침은 `docs/INSTRUCTIONS.md` 파일에 작성됐어
- 연구주제는 `docs/Research_Topic.md` 파일에 작성됐어
- `.claudeignore` 파일에 작성된 폴더 및 파일들은 직접 요청할 때 말고 읽지 마
- 파일 구조는 `docs/file_structure.md` 파일에 작성됐으니 읽고 지금의 폴더와 하위 폴더의 목적을 기준으로 맥락을 파악한 뒤 검색을 진행해
- Perception Module의 설계 철학, 최적화 과정, 최종 성능 및 실험 해석은 `docs/perception_module_analysis.md` 파일에 작성됐으니, Perception 관련 요청에서만 우선 참조해.
- Decision Module의 설계 철학, 구현 구조, 최적화 과정, 최종 성능 및 실험 해석은 `docs/decision_module_analysis.md` 파일에 작성됐으니, Decision 관련 요청에서만 우선 참조해.
- Perception-Decision Pipeline의 설계 구조, 병목 분석, 최적화 과정, 최종 성능 및 실험 해석은 `docs/perception_decision_pipeline.md` 파일에 작성됐으니, 관련 요청에서만 우선 참조해.
- 코드 실행은 Jetson 에서 따로 진행할꺼니, 직접 실행하지말고 검증만 진행해

## Jetson ROS / RealSense Runtime Defaults

When the user asks to inspect, verify, or operate the Jetson camera, RealSense,
ROS2 topics, CameraPreprocessor, RGB-D ingress, or camera-only map-update
checks, treat the following environment as mandatory by default.

```bash
export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/orin/vlm/install/setup.bash
```

Use this pattern for Jetson ROS checks:

```bash
ssh jetson-orin "cd ~/vlm/src && bash -lc 'export ROS_DOMAIN_ID=15; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; source /opt/ros/humble/setup.bash; source /home/orin/vlm/install/setup.bash; <command>'"
```

Do not conclude that the camera or RealSense node is unavailable from a ROS graph
check unless the command was run with `ROS_DOMAIN_ID=15` and
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`. A prior false negative happened because
`ros2 topic list` was executed without this domain/RMW environment, even though
the RealSense D435 and `/gripper_camera/gripper_camera` node were running.

Expected live camera topics:

```text
/gripper_camera/gripper_camera/color/image_raw
/gripper_camera/gripper_camera/aligned_depth_to_color/image_raw
/gripper_camera/gripper_camera/aligned_depth_to_color/camera_info
```

For map-update camera-only validation, prefer:

```bash
python3 L-OMM/scripts/map_update_layer/experiments/camera_only_rgbd_contract_check.py --out path/camera_only_rgbd_contract_check_live_rerun --iterations 30 --live-samples 10 --live-timeout-sec 12
```

The known-good live validation result after applying the ROS environment was:

```text
camera_only_validation = PASS
live_sample_count = 10
snapshot_valid = 1.0
latency_status = CONTRACTUAL_PASS
```
