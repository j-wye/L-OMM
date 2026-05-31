# Preprocessing Pipeline: Integrated Perception-Decision Architecture

> 문서 목적: 현재 `perception_decision_pipeline.py`의 설계, 실행 구조, 출력 계약, 성능 해석을 정리하는 단일 기준 문서.
> 기준 구현: Camera ingress + PerceptionModule + DecisionModule 통합 runner.
> 현재 역할: standalone diagnostic 실행도 가능하지만, 최종적으로는 `main.py`가 import하여 Control Module에 `T_map_target`을 넘기는 upstream pipeline.

---

## 1. 핵심 결론

현재 perception-decision pipeline은 단순 RGB-D adapter가 아니다. 최종 구조는 다음과 같다.

```text
RealSense RGB-D topics
    -> CameraPreprocessor
    -> PerceptionDecisionPipeline
        -> PerceptionModule
        -> thin boundary ROI preparation
        -> DecisionModule
    -> T_map_target, control_ready, diagnostic metrics
```

이 구조의 핵심 출력은 DecisionModule이 반환하는

```text
T_map_target = ^{map}T_{target}
```

이다. 이 값이 Control Module에 전달될 기본 target pose다. `p_map_raw`는 object center 진단값이고, control target으로 쓰는 값은 아니다.

현재 실측 로그는 통합 pipeline이 거의 카메라 주기와 동기화되어 동작함을 보인다.

| 항목 | 현재 관측값 | 해석 |
|---|---:|---|
| Loop rate | 26-29 Hz | 30 Hz camera에 거의 동기 |
| Camera rate | 27-30 Hz | RealSense input 정상 |
| Detection | 전 프레임 valid | Perception result 안정 |
| Transform | fail 0 | standalone xform 또는 TF path 정상 |
| Decision status | all fresh | held/rejected 없이 매 프레임 갱신 |
| control_ready | all valid frames | Control에 넘길 pose가 지속 생성 |
| Perception infer | 약 15.5-16.0 ms | 통합 runner 기준 안정 |
| Boundary | 약 0.7-1.0 ms | bbox 4-scalar CPU read + exact ROI slicing |
| Decision core | 약 17.3-19.4 ms | 현재 주된 compute component |
| Overwrite | 0-2 / sec | 최신 프레임 정책상 허용 가능한 경미한 밀림 |

따라서 현재 상태는 **quasi-static grasping과 mobile manipulation target update에는 실용적으로 충분**하다. 다만 30 Hz 모든 카메라 프레임을 빠짐없이 처리하는 구조는 아니다. 최신 프레임 overwrite 정책을 사용하므로, 처리하지 못한 과거 프레임을 쌓지 않고 가장 최신 observation으로 갱신한다.

---

## 2. 역할 분리

### 2.1 CameraPreprocessor

`CameraPreprocessor`는 ROS2 RealSense 토픽을 수신하고 동기화된 최신 RGB-D frame을 제공한다.

책임은 다음으로 제한된다.

- RGB image topic 수신
- aligned depth topic 수신
- camera info 수신
- RGB-depth approximate sync
- direct numpy conversion
- latest-frame overwrite buffer
- timestamp, freshness, overwrite metric 유지
- `FrameData` 반환

이 계층에는 image filtering, segmentation postprocessing, grasp geometry, control policy를 넣지 않는다.

### 2.2 TransformProvider

`TransformProvider`는 TF2에서

```text
T_map_optical = ^{map}T_{gripper_camera_color_optical_frame}
```

을 조회한다.

중요한 정책은 다음이다.

- 기본 target frame은 `map`
- 기본 source frame은 `gripper_camera_color_optical_frame`
- 성공 시 `torch.float32` 4x4 tensor 반환
- 실패 시 `None` 반환
- identity fallback 금지
- latest fallback은 debug flag가 켜졌을 때만 허용

이 정책은 DecisionModule의 frame contract와 일치한다. raw RGB-D backprojection은 optical frame에서 정의되고, 최종 state/output은 map frame으로 표현된다.

### 2.3 StaticTransformProvider

현재 mobile base를 켜지 않는 실험에서는 TF2가 아니라 fixed standalone transform을 사용한다.

```text
R_map_optical =
[[ 0,  0,  1],
 [-1,  0,  0],
 [ 0, -1,  0]]
```

translation은 0으로 둔다. 이는 robotless test에서 `decision_module.py` standalone과 같은 frame convention을 보장하기 위한 실험용 provider다.

### 2.4 PerceptionDecisionPipeline

`PerceptionDecisionPipeline`은 `main.py`가 import해서 사용할 핵심 class다.

주요 interface는 다음이다.

- `setup(first_frame)`
- `warmup(frame, n)`
- `step(frame) -> dict`
- `latest_grasp`
- `latest_T_map_target`
- `latest_control_ready`

`step()`은 한 frame에 대해 다음 순서를 수행한다.

1. PerceptionModule inference
2. perception event synchronize
3. detection invalid이면 Decision skip
4. `T_map_optical` lookup
5. transform unavailable이면 Decision skip
6. bbox 4 scalar만 host로 읽어 exact ROI crop 범위 계산
7. RGB/depth/mask/ray-base ROI는 GPU tensor slice로 유지
8. DecisionModule ROI-ready API 호출
9. `T_map_target`와 `control_ready` 저장

---

## 3. Threading 구조 검증

현재 실행 구조는 의도한 형태와 맞다.

```text
ROS2 executor thread
    MultiThreadedExecutor.spin()
        CameraPreprocessor callbacks
        TransformProvider callbacks if tf2 mode

Main thread
    wait_for_frame()
    PerceptionDecisionPipeline.step()
    diagnostic logging
```

`CameraPreprocessor`는 `ReentrantCallbackGroup`과 `MultiThreadedExecutor`에 의해 main compute loop와 분리되어 계속 callback을 처리한다. main thread가 perception/decision을 수행하는 동안에도 ROS2 callback thread는 다음 frame을 받을 수 있다.

이 구조가 필요한 이유는 다음이다.

```text
T_frame = 33.3 ms at 30 Hz
T_compute = T_infer + T_boundary + T_decision
          ~= 15.7 + 0.8 + 18.1
          ~= 34.6 ms
```

compute time이 camera period와 거의 같기 때문에, callback 수신을 main thread와 같은 루프에 묶으면 sensor ingress가 쉽게 지연된다. 현재처럼 executor thread를 분리하면 입력 수신은 계속 유지되고, 처리 루프는 최신 frame만 소비한다.

---

## 4. 최신 프레임 정책

현재 정책은 latest-frame overwrite다.

```text
if new frame arrives before previous frame is consumed:
    overwrite previous frame
```

이것은 real-time robotics에서 queue보다 더 적절하다. 이유는 control이 과거 observation을 순차적으로 모두 처리하는 것보다, 최신 observation을 사용하는 편이 안정적이기 때문이다.

현재 로그의 `overwrite 1-2`는 치명적 문제가 아니다. 의미는 다음이다.

- 특정 1초 window에서 카메라는 약 29-30 frame을 제공
- compute loop는 약 27-29 frame을 처리
- 처리 중 도착한 일부 과거 frame이 최신 frame으로 교체됨
- backlog는 쌓이지 않음

따라서 이 상태에서 추가 최적화를 위해 구조를 복잡하게 만들 필요는 낮다. 연구 범위가 quasi-static side grasp라면 pose update가 26-29 Hz로 들어오는 것은 충분하다.

---

## 5. Output Contract

`PerceptionDecisionPipeline.step()`의 핵심 반환값은 다음이다.

| Field | 의미 | Downstream 사용 |
|---|---|---|
| `control_ready` | control에 target을 넘겨도 되는지 | command gate |
| `grasp` | DecisionModule output dict | diagnostic / metadata |
| `T_map_target` | `^{map}T_{target}` | control target pose |
| `perception_result` | Perception output | detection diagnostic |
| `status` | fresh/held/rejected 등 사람이 읽는 상태 | log |
| `reason` | skip/reject 이유 | log |
| `xform_valid` | transform lookup 성공 여부 | integration diagnostic |
| `infer_sec` | perception 시간 | performance log |
| `boundary_sec` | thin boundary 시간 | performance log |
| `decision_sec` | decision core 시간 | performance log |

`latest_control_ready`의 기본 정의는 다음이다.

```text
output_valid == True
stale == False
feasible == True
```

따라서 `main.py`는 기본적으로 다음 원칙을 따른다.

```text
if pipeline.latest_control_ready:
    T_map_target = pipeline.latest_T_map_target
    ControlModule receives T_map_target
```

held 또는 stale output은 log에는 남길 수 있지만, 기본 control command에는 사용하지 않는다.

---

## 6. Timing Metric 정의

### 6.1 `wait`

`wait`는 main thread가 `wait_for_frame()`에서 새 frame을 기다린 평균 시간이다.

```text
wait = time spent blocked in wait_for_frame()
```

해석은 다음과 같다.

- `wait`가 크다: compute가 camera보다 빠르며 새 frame을 기다리는 idle 시간이 있다.
- `wait`가 작다: compute가 camera period에 거의 붙어 있으며, 다음 frame이 이미 준비되어 있다.

현재 로그에서 `wait`가 0.04-2 ms 수준이라는 것은 compute loop가 거의 camera-bound 상태라는 의미다.

### 6.2 `buffer_age`

`buffer_age`는 sync callback이 frame을 latest buffer에 저장한 시각부터 main thread가 그 frame을 꺼내 처리하기 시작한 시각까지의 시간이다.

```text
buffer_age = t_main_consumes_frame - t_callback_stores_frame
```

현재 `buffer_age 16-22 ms`는 preprocessing callback이 느리다는 뜻이 아니다. main thread가 이전 frame의 perception+decision을 수행하는 동안 다음 frame이 buffer에 먼저 들어와 있었음을 의미한다.

즉, 현재 병목은 camera ingress가 아니라 통합 compute path다.

### 6.3 `infer`

PerceptionModule inference 시간이다. 현재 약 15.5-16.0 ms로 안정적이다.

### 6.4 `boundary`

Perception 결과와 Decision 입력 사이의 thin boundary 시간이다.

구성은 다음이다.

- bbox 4 scalar host read
- RGB tensor normalization buffer copy
- depth mm -> m conversion
- mask buffer copy
- exact ROI slice 생성

현재 0.7-1.0 ms 수준으로 작다. 이 부분을 더 줄이는 작업은 가능하더라도 전체 성능에 대한 기대효과는 제한적이다.

### 6.5 `decision_core`

ROI-ready tensor를 DecisionModule에 넣은 뒤 grasp output을 받을 때까지의 시간이다. 현재 약 17-19 ms다.

전체 compute budget에서 가장 큰 항목은 perception과 decision core이며, boundary는 부차적이다.

---

## 7. 성능 판단

현재 통합 compute 시간은 대략 다음과 같다.

```text
T_total = T_infer + T_boundary + T_decision
        ~= 15.7 ms + 0.8 ms + 18.1 ms
        ~= 34.6 ms
```

30 Hz camera period는 다음이다.

```text
T_30Hz = 33.3 ms
```

따라서 현재 pipeline은 이론적으로 30 Hz보다 약간 무겁다. 하지만 latest overwrite 정책 때문에 지연 queue가 쌓이지 않는다. 실제 출력은 26-29 Hz이고, 이는 mobile manipulation의 perception-decision target update로 충분히 실용적이다.

추가 최적화를 강제로 진행해야 하는 조건은 다음 정도다.

- overwrite가 지속적으로 5-10 이상 발생
- loop가 20 Hz 이하로 떨어짐
- `control_ready`가 불연속적으로 끊김
- `held/rejected` 비율이 증가
- fast moving object를 추적해야 함

현재 로그는 이 조건에 해당하지 않는다. 따라서 지금 단계에서는 추가 보정보다 `main.py`와 Control Module 통합을 진행하는 것이 더 합리적이다.

---

## 8. Failure Policy

현재 failure policy는 보수적이다.

| Failure | 처리 |
|---|---|
| camera frame 없음 | loop timeout count 증가, Decision 호출 안 함 |
| detection invalid | Decision 호출 안 함 |
| TF lookup failure | Decision 호출 안 함 |
| standalone mode | fixed `T_map_optical` 사용 |
| latest TF fallback | debug flag에서만 허용 |
| identity fallback | 금지 |

이 정책은 false target command를 줄이는 방향이다. 특히 transform failure에서 identity를 사용하는 것은 map-frame control target을 오염시키므로 금지해야 한다.

---

## 9. 왜 ROS topic으로 GPU Tensor를 보내지 않는가

`T_map_target`는 GPU tensor로 유지된다. 최종 구조에서 `main.py`가 같은 Python process에서 pipeline을 import하면 GPU tensor를 그대로 Control Module에 넘길 수 있다.

반대로 perception-decision output을 ROS topic으로 publish하면 다음 문제가 생긴다.

- GPU tensor를 host로 내려야 함
- serialization 비용 발생
- timestamp와 freshness 관리가 복잡해짐
- Control Module이 같은 process에서 사용할 수 있는 zero-copy 이점을 잃음

따라서 최종 control integration은 다음 구조가 맞다.

```text
main.py process
    CameraPreprocessor
    PerceptionDecisionPipeline
    ControlModule
```

diagnostic 목적으로만 `perception_decision_pipeline.py`를 직접 실행한다.

---

## 10. Main Integration Contract

`main.py`는 다음 책임을 가진다.

- pipeline lifecycle 관리
- Control Module lifecycle 관리
- `control_ready` gate 확인
- `T_map_target` 전달
- safety policy 적용
- command publish

perception-decision pipeline은 control command를 직접 발행하지 않는다.

최소 control gate는 다음이다.

```text
control_ready == True
xform_valid == True
status == fresh
```

필요하면 metadata를 함께 사용할 수 있다.

- width
- depth
- safe_width_margin
- quality_metric
- axis_quality
- status_code
- reason_code

그러나 기본 target pose는 오직 `T_map_target`이다.

---

## 11. 현재 구조에 대한 최종 판단

현재 perception-decision pipeline 구조는 연구 파이프라인의 다음 단계로 넘어가기에 충분히 정리되어 있다.

기술적으로 중요한 점은 다음이다.

1. Camera 수신은 executor thread에서 계속 진행된다.
2. Main compute loop는 최신 frame만 소비한다.
3. Perception과 Decision은 순차 의존성을 유지한다.
4. Thin boundary는 bbox 4 scalar host read로 제한된다.
5. Decision output은 `T_map_target`으로 control-ready 형태다.
6. TF failure에서 identity fallback을 쓰지 않는다.
7. direct execution은 diagnostic이고, 최종 control 통합은 import 기반이다.

남은 개선은 최적화라기보다 통합이다. 지금 단계에서 불필요하게 perception-decision pipeline을 더 손대기보다, `main.py`에서 Control Module과 `T_map_target` 전달 계약을 완성하는 것이 우선순위가 높다.
