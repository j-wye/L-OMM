# Preprocessing Pipeline: 최종 분석 및 코드 아키텍처

> **문서 목적**: `preprocessing.py`의 설계, 구현, 실측 성능, 최적화 결정의 **단일 출처(single source of truth)**.
> **플랫폼**: Jetson AGX Orin 64GB @ MAXN mode
> **카메라**: Intel RealSense D435i (1280×720, 30 FPS)
> **최종 실측**: 2026-04-06

---

## 1. Executive Summary

### 1.1 시스템 최종 상태

| 지표 | 실측값 | 판정 |
|------|--------|------|
| **Standalone Perception FPS** | 62.4 (16.02 ms/frame) | 기준선 (round-trip latency) |
| **Profiled Total FPS** | 61.3 (16.30 ms) | per-stage exclusive profiling |
| **Live Perception FPS** | 65.0–65.7 | ✅ standalone보다 높음 → preprocessing 오버헤드 0 |
| **Camera FPS** | 29.0–30.0 | ✅ 카메라 30Hz 정상 |
| **Loop FPS** | 29.0–30.0 | ✅ Camera FPS에 동기 (input-bound) |
| **Headroom** | 2.16–2.81x | ✅ Camera 대비 2배+ 마진 |
| Drop(est) | 0 | ✅ 프레임 유실 없음 |
| Overwrite | 0 | ✅ freshness 손실 없음 |
| PairEff | 100% | ✅ RGB-Depth sync 완벽 |
| CbMean | 0.06–0.07 ms | ✅ callback 비용 무시 가능 |
| BufferAge | 0.38–0.49 ms | ✅ handoff 지연 없음 |
| SyncWait | 0.21–0.23 ms | ✅ ATS pairing 지연 없음 |
| **SensorAge** | **71.9–73.5 ms** | ⚠️ transport 지배 (latency, FPS와 무관) |
| **IngressAge** | **55.8–57.5 ms** | ⚠️ ROS2 DDS transport 지배 |
| Hint | UPSTREAM_TRANSPORT (일관) | transport가 유일한 staleness 원인 |

### 1.2 핵심 결론

1. **Preprocessing은 병목이 아니다.** Local callback + conversion + buffer handoff의 합계 비용은 < 1 ms.
2. **Live FPS가 standalone보다 높다** (65 vs 62.4): wait_for_frame 대기 중 GPU idle이 interference를 제거하기 때문.
3. **SensorAge 72 ms는 FPS와 무관한 별개의 문제**: staleness(데이터 신선도)이지 throughput이 아님.
4. **시스템은 production-ready**: 30 FPS 카메라 대비 2x+ headroom, 10 FPS 파이프라인 목표 대비 6x+ 마진.

---

## 2. 코드 아키텍처

### 2.1 전체 데이터 흐름

```
[RealSense D435i]
    │  USB3 → ROS2 realsense2_camera driver
    │  Color: 1280×720×30fps, rgb8 (2.76 MB/frame)
    │  Depth: 1280×720×30fps, 16UC1 (1.84 MB/frame, aligned)
    ▼
[ROS2 DDS Transport] ──── IngressAge ~56 ms (sensor→arrival) ────
    │
    ▼
[CameraPreprocessor Node]
    ├── _rgb_raw_callback:   카운터 + transport age 기록
    ├── _depth_raw_callback: 카운터 + transport age 기록
    ├── _info_callback:      intrinsics 저장 (1회)
    └── _sync_callback: (ApproximateTimeSynchronizer)
         ├── encoding 자동 감지 (첫 프레임)
         ├── direct numpy view (cv_bridge 우회) ← CbMean 0.06ms
         ├── conditional contiguous copy
         ├── sync wait 계산 (raw arrival → sync dispatch)
         └── latest-frame overwrite + Event.set()
    │
    │  BufferAge ~0.4 ms
    ▼
[Main Thread: wait_for_frame()]
    │
    ▼
[PerceptionModule.infer_tensor()] ──── Inference ~16 ms ────
    ├── CPU: np.copyto (HWC→CHW ± BGR→RGB)  ~1.0 ms
    ├── DMA: pinned→GPU async                ~0.2 ms
    ├── GPU Graph 1: OWL preproc + TRT       ~12.2 ms
    ├── GPU Graph 2: SAM encoder + TRT       ~8.4 ms
    └── GPU Graph 3: SAM decoder + upscale   ~2.5 ms
    │
    └── PerceptionResult(bbox, mask, score, valid, event)
```

### 2.2 핵심 설계 결정

| 결정 | 근거 |
|------|------|
| **MultiThreadedExecutor (2 threads)** | raw/sync callback 병렬 dispatch → scheduling jitter 감소 |
| **ReentrantCallbackGroup** | callback-group lock에 의한 직렬화 방지 |
| **Direct numpy conversion** | cv_bridge Python wrapper 우회 → zero-copy view (rgb8, bgr8, 16UC1 지원) |
| **cv_bridge fallback** | 미지원 encoding 시 안전하게 fallback |
| **Conditional contiguous** | 이미 contiguous면 copy 생략 (불필요한 2.76 MB memcpy 방지) |
| **Latest-frame overwrite** | queue 없이 항상 최신 프레임만 유지 → 최소 staleness |
| **Event-based wait** | polling 대신 threading.Event → main thread idle 최소화 |
| **suspend_callbacks()** | 모델 초기화/벤치마크 시 callback 간섭 완전 차단 |
| **reset_runtime_counters()** | warmup/profile 이후 live session 통계 깨끗하게 시작 |
| **first-frame baseline skip** | live loop 진입 시 첫 프레임은 baseline 설정만, 집계 제외 |

### 2.3 FrameData 인터페이스 계약

```python
class FrameData(NamedTuple):
    rgb: np.ndarray          # HWC uint8, C-contiguous (BGR or RGB)
    depth: np.ndarray        # HW uint16, C-contiguous, aligned to color, mm
    intrinsics: np.ndarray   # 3×3 float64 K matrix
    distortion: np.ndarray   # 1D float64 distortion coefficients
    distortion_model: str    # ROS distortion model name
    is_bgr: bool             # True = BGR channel order
    timestamp: float         # ROS header stamp (sensor time, seconds)
    received_timestamp: float # local wall-clock when frame was stored
```

이 계약은 PerceptionModule, Decision Module, 그리고 향후 모든 downstream 모듈의 **단일 입력 인터페이스**이다.

---

## 3. Observability 3축 체계

### 3.1 Throughput (처리량)

| 지표 | 정의 | 정상 범위 |
|------|------|-----------|
| Camera FPS | sync callback 수신율 / elapsed | ~30 |
| RGB FPS | raw RGB callback 수신율 / elapsed | ~30 |
| Depth FPS | raw Depth callback 수신율 / elapsed | ~30 |
| Perception FPS | processed / (inference time 합계) | ~62–66 |
| Loop FPS | processed / elapsed | ~30 (camera-bound) |
| Profiled FPS | startup profile 기준선 | ~61 |
| Headroom | Perception FPS / Camera FPS | ≥ 2.0x |

### 3.2 Freshness (신선도)

| 지표 | 정의 | 정상 범위 | 문제 원인 |
|------|------|-----------|-----------|
| RGBTrans | sensor stamp → raw RGB callback arrival | ~56 ms | ROS2 DDS transport |
| DepthTrans | sensor stamp → raw Depth callback arrival | ~54 ms | ROS2 DDS transport |
| SyncWait | max(rgb, depth) arrival → sync callback start | ~0.2 ms | ATS pairing |
| IngressAge | sensor stamp → received_timestamp (sync end) | ~56 ms | transport + sync |
| BufferAge | received_timestamp → inference start | ~0.4 ms | main thread dispatch |
| SensorAge | sensor stamp → inference end | ~72 ms | 전체 end-to-end |
| SensorAgeMax | window 내 최대 SensorAge | ~76 ms | worst-case latency |

### 3.3 Integrity (무결성)

| 지표 | 정의 | 정상값 | 이상 시 의미 |
|------|------|--------|-------------|
| PairEff | sync_frames / min(rgb_raw, depth_raw) | 100% | < 95% → ATS 손실 |
| CumPairEff | 누적 PairEff | 100% | 장기 추세 모니터링 |
| Drop(est) | sync_frames - processed | 0 | > 0 → inference 못 따라감 |
| Overwrite | 소비 전 덮어쓴 횟수 | 0 | > 0 → inference 느림 |
| CbMean | callback CPU 비용 평균 | < 0.1 ms | > 1 ms → conversion 문제 |
| Stamp | stamp domain 판정 (OK/APPROX) | OK | APPROX → age 해석 부정확 |

### 3.4 Pipeline Hint (자동 병목 판정)

```python
classify_pipeline_hint() → str:
    "LOCAL"              # drop/overwrite/buffer>5ms/cb>1ms
    "SYNC"               # pair_eff<95% or sync_wait>2ms
    "UPSTREAM_DEPTH"     # transport 지배 + depth FPS < rgb FPS
    "UPSTREAM_TRANSPORT" # transport 지배 + 양 stream 정상
    "RUNTIME"            # perception FPS < 0.9 × profiled FPS
    "STABLE"             # 모든 지표 정상
```

---

## 4. SensorAge 72 ms의 정확한 분해

### 4.1 수치적으로 닫히는 분해

$$
A_{\text{sensor}} = A_{\text{transport}} + A_{\text{sync-wait}} + A_{\text{buffer}} + T_{\text{infer}}
$$

$$
72.2\text{ ms} \approx 56.2\text{ ms} + 0.21\text{ ms} + 0.44\text{ ms} + 15.4\text{ ms}
$$

| Component | 실측 | Budget 비중 |
|-----------|------|------------|
| **Transport (sensor→raw arrival)** | **~56 ms** | **78%** |
| Inference (PerceptionModule) | ~15.4 ms | 21% |
| Sync wait (ATS pairing) | ~0.21 ms | 0.3% |
| Buffer age (local handoff) | ~0.44 ms | 0.6% |

### 4.2 SensorAge와 FPS는 별개의 문제

이것은 이전 분석에서 혼동되었던 핵심 구분이다:

| 문제 | 정의 | 해결 방향 |
|------|------|-----------|
| **Throughput** (FPS) | 초당 처리 가능한 프레임 수 | PerceptionModule 연산량에 의존 → **이미 최적** |
| **Latency** (SensorAge) | 센서 촬영 → 결과 사용 가능까지의 지연 | ROS2 transport에 의존 → **코드 외적 요인** |

- SensorAge를 0으로 만들어도 FPS는 변하지 않음
- Perception FPS를 2배로 올려도 SensorAge에서 inference 비중(21%)만 감소

### 4.3 물리적 영향 분석

$$\Delta x \approx v \cdot \tau$$

| 상대속도 | 위치 오차 | Grasp 판정 |
|----------|-----------|-----------|
| 0.05 m/s | 3.6 mm | ✅ gripper finger 내 → 허용 |
| 0.10 m/s | 7.2 mm | ⚠️ 경계 수준 |
| 0.20 m/s | 14.4 mm | ❌ gripper stroke 대비 무시 불가 |

**Static/quasi-static grasping (ICRA 2026 연구 범위)에서는 SensorAge 72 ms가 문제되지 않는다.**

---

## 5. 해상도와 FPS의 관계 정리

### 5.1 왜 standalone 74 FPS vs live 62.4 FPS인가

이전에 `perception_module.py` standalone이 ~74 FPS, `preprocessing.py`가 ~62 FPS로 보고되어 "preprocessing이 느리게 만든다"는 오해가 있었다.

**진짜 원인**: standalone 테스트가 **640×480**, live가 **1280×720** 이미지를 사용했다.

| 요인 | FPS 영향 | SensorAge 관련? |
|------|---------|----------------|
| **해상도 3x** (640×480 → 1280×720 = 픽셀 3배) | **지배적** (cpu_copy, F.interpolate, mask upscale) | ❌ 무관 |
| **측정 방식** (pipelined vs round-trip sync) | ~3-5 FPS | ❌ 무관 |
| **GPU cache** (동일 이미지 반복 vs 매 프레임 새 이미지) | ~1-2 FPS | ❌ 무관 |
| **Background 경합** (ROS thread) | < 1 FPS (실측 0) | ❌ 무관 |

### 5.2 해상도 의존 연산 상세

TRT engine 자체는 고정 크기(OWL 768², SAM 1024²)이므로 해상도 무관. 차이는 전/후처리에서 발생:

| Stage | 640×480 | 1280×720 | 비율 |
|-------|---------|----------|------|
| CPU np.copyto (HWC→CHW) | ~0.4 ms | ~1.0 ms | 2.5x |
| GPU F.interpolate (OWL preproc) | ~0.3 ms | ~0.7 ms | 2.3x |
| GPU F.interpolate (SAM preproc) | ~0.4 ms | ~0.9 ms | 2.3x |
| GPU mask upscale | ~0.3 ms | ~0.8 ms | 2.7x |

### 5.3 현재 1280×720에서의 실측 stage 분해

```
cpu_copy = 1.01 ms  ← 해상도 의존 (HWC→CHW transpose + optional BGR→RGB)
copy     = 0.20 ms  ← 해상도 의존 (pinned→GPU DMA)
owl      = 12.21 ms ← preproc 의존, TRT 고정
sam_enc  = 8.44 ms  ← preproc 의존, TRT 고정
sam_dec  = 2.52 ms  ← mask upscale 의존

Total    = 16.30 ms → 61.3 FPS (profiled)
Critical = 14.94 ms → 66.9 FPS (overlap 감안)
```

---

## 6. Transport Latency 분해

### 6.1 RealSense → ROS2 DDS → Consumer 경로

```
[D435i Sensor] →(USB3)→ [RealSense Node] →(DDS)→ [CameraPreprocessor]
    ↓                      ↓                          ↓
  Exposure/Capture      Align + Publish            raw callback
  (hardware)            (driver CPU)               (arrival wall)
```

| 단계 | 추정 기여 | 근거 |
|------|----------|------|
| IR exposure + depth processing | ~15–20 ms | D435i structured light pipeline |
| Align depth to color | ~5–10 ms | CPU reprojection |
| ROS2 publication + serialization | ~3–5 ms | sensor_msgs/Image DDS encoding |
| DDS intra-host transport | ~2–5 ms | shared memory or loopback |
| Subscriber scheduling latency | ~10–20 ms | rclpy executor dispatch |
| **합계** | **~35–60 ms** | 실측 ~56 ms와 정합 |

### 6.2 Transport 최적화 옵션 (적용 여부 결정)

| Level | 방법 | 예상 효과 | 판정 |
|-------|------|----------|------|
| **Level 0** | depth filter 비활성화 (temporal/spatial/decimation) | -5–15 ms | 연구 범위 외 (launch 설정) |
| **Level 1** | MultiThreadedExecutor + direct numpy | -2–5 ms | ✅ **이미 적용** |
| **Level 2** | DDS shared memory / zero-copy transport | -3–5 ms | 연구 범위 외 (middleware 설정) |
| **Level 3** | pyrealsense2 직접 사용 (ROS2 bypass) | ~56→15–25 ms | ⚠️ 검증 완료, **mainline 미채택** |

### 6.3 pyrealsense2 직접 모드의 최종 판정

ablation 실험 결과:
- **Freshness**: 대폭 개선 (SensorAge ~35 ms)
- **Throughput 안정성**: 악화 (source ingest + align이 inference 프로세스로 이동 → GPU contention)

**결론**: root-cause 확인 용도로만 사용. 기본 배포 경로는 ROS2 유지.

---

## 7. Main Loop 실행 시퀀스

### 7.1 Startup Phase

```
1. rclpy.init() + CameraPreprocessor 생성
2. MultiThreadedExecutor 시작 (background thread)
3. wait_for_ready() → Intrinsics + 첫 프레임 수신 대기 (max 30s)
4. First frame 획득 → encoding 자동 감지
5. suspend_callbacks() → 모든 callback 정지
6. PerceptionModule 초기화 (TRT engine load + CUDA Graph capture)
7. Warmup 20회 (CUDA context 안정화)
8. Standalone benchmark (50회, torch.cuda.synchronize per frame)
9. Per-stage profile (20회, CUDA Event 기반)
10. resume_callbacks() → callback 재개
11. clear_pending_frame() + reset_runtime_counters() → 통계 초기화
12. Live loop 진입
```

### 7.2 Live Loop (1-second window)

```
loop:
  wait_for_frame(0.1s timeout)
  if frame is None → continue (timeout 카운터 증가)
  if first frame after reset → baseline 설정 → continue

  # Inference (single-flight)
  torch.cuda.synchronize()          # GPU drain
  result = perception.infer_tensor()
  torch.cuda.synchronize()          # completion 대기

  # 1-second accumulation
  if elapsed ≥ 1.0:
    compute all derived metrics
    classify_pipeline_hint()
    print stats line
    reset window
```

---

## 8. 논문 서술 관점

### 8.1 보고 방법

| 지표 | 보고 용도 | 주의 |
|------|----------|------|
| **Perception FPS** | 모듈 처리 능력 (해상도 1280×720 명시) | standalone 74는 640×480 기준 → 혼동 주의 |
| **Loop FPS** | 시스템 실시간 처리율 | Camera FPS에 bounded |
| **SensorAge** | 센서→결과 end-to-end latency | FPS와 별도 보고 |
| **Headroom** | 실시간 여유 마진 | ≥ 2.0x면 충분 |

### 8.2 Reviewer 방어 논리

- **Throughput ≠ Latency**: FPS만으로 실시간성을 주장하면 안 됨. `Real-time = Throughput × Freshness`
- **Static grasping**: 연구 범위 내에서 72 ms latency는 허용 가능 (Δx ≈ 0 at v ≈ 0)
- **Limitation**: 동적 장면에서의 trade-off는 limitation으로 명시, 해결 방향(transport 최적화) 제시

---

## 9. 시스템 건강성 판정 기준

### 9.1 정상 운영

```
Camera FPS: ~30 | Perception FPS: ~65 | Loop FPS: ~30 |
Headroom: ~2.2x | Drop: 0 | Overwrite: 0 | PairEff: 100% |
BufferAge: <1ms | CbMean: <0.1ms | Hint: UPSTREAM_TRANSPORT or STABLE
```

### 9.2 이상 감지

| 증상 | Hint | 조치 |
|------|------|------|
| Drop > 0 or Overwrite > 0 | LOCAL | inference 부하 확인, GPU thermal throttling 점검 |
| PairEff < 95% | SYNC | sync_slop 확대, 카메라 stream 동기 확인 |
| Perception FPS < 0.9 × Profiled | RUNTIME | TRT 재빌드, GPU clock 확인 |
| 30 consecutive timeouts | — (WARN 출력) | 카메라 연결/ROS 노드 상태 점검 |
| Stamp: APPROX | — | ROS stamp이 system time과 불일치 → age 값 절대 해석 불가 |

---

## 10. 결론: 최적화 완료 상태

### 10.1 적용된 최적화

| 최적화 | 효과 | 상태 |
|--------|------|------|
| MultiThreadedExecutor (2 threads) | callback scheduling jitter 감소 | ✅ 적용 |
| ReentrantCallbackGroup | callback 병렬 dispatch | ✅ 적용 |
| Direct numpy conversion (cv_bridge 우회) | CbMean 0.20→0.06 ms | ✅ 적용 |
| Conditional contiguous copy | 불필요한 memcpy 방지 | ✅ 적용 |
| suspend_callbacks during init | 모델 로딩 시 간섭 차단 | ✅ 적용 |
| reset_runtime_counters | 깨끗한 live session 시작 | ✅ 적용 |
| First-frame baseline skip | 첫 프레임 통계 오염 방지 | ✅ 적용 |

### 10.2 적용하지 않은 이유가 있는 것들

| 검토 항목 | 판정 | 이유 |
|-----------|------|------|
| pyrealsense2 직접 모드 | ❌ 미채택 | freshness ↑ but throughput stability ↓ |
| DDS zero-copy transport | ⏸️ 보류 | middleware 설정 범위, 코드 변경 아님 |
| GPU alignment (CUDA kernel) | ❌ 미채택 | C++ CUDA kernel 필요, 연구 범위 초과 |
| Depth filter 비활성화 | ⏸️ 보류 | launch 파라미터 변경, 코드 외적 |

### 10.3 다음 단계

**Preprocessing은 완료(done) 상태이다. 추가 코드 최적화는 필요하지 않다.**

다음 과제:
1. **Decision Module Interface 설계** — PerceptionResult + FrameData → grasp planning
2. **전체 파이프라인 end-to-end latency 실측** — Perception + Decision + Control 통합
3. **ROS2 launch 파라미터 최적화** (Level 0) — 필요 시 depth filter 비활성화로 SensorAge 추가 감소
