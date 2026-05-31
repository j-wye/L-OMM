# Whole Body Control 확장: Mobile Manipulator Architecture 심층 분석

## 1. 문제의 재정의

### 1.1 시나리오

Mobile manipulator의 pick-and-place task:

```
[Phase 1: Approach]     Base: Start → Task Position (navigation)
[Phase 2: Manipulation] Base: Stop, Arm: Reach → Grasp (current research)
[Phase 3: Transport]    Base: Task Position → End Position (carrying object)
[Phase 4: Place]        Base: Stop, Arm: Place → Retract
```

### 1.2 핵심 질문 4가지

1. **Post-grasp configuration 미고려**: Grasping 후 object를 잡고 있는 상태에서의 arm configuration이 현재 map/planning에 반영되지 않음
2. **q[0]의 역할**: 객체를 바라보도록 회전하는 용도로만 사용한다면, map과 planning의 구조가 근본적으로 달라져야 하는가?
3. **q[0] Counter-Rotation**: q[0]를 mobile base의 yaw와 반대 방향으로 회전시키면, 기존 grasping-only 시나리오와 수학적으로 동치가 되는가? 이 경우 effective 5DOF로 봐야 하는가?
4. **Map 재설계**: 전체를 처음부터 다시 해야 하는가, 아니면 현재 구조의 확장으로 가능한가?

---

## 2. Phase 분석: 각 단계에서의 Configuration Space

### 2.1 전체 시스템의 Configuration

$$\mathbf{q}_{\text{full}} = (\underbrace{x_b, y_b, \theta_b}_{\text{mobile base} \in SE(2)}, \underbrace{q_0, q_1, \ldots, q_5}_{\text{manipulator} \in \mathbb{R}^6})$$

Total DOF = 9. 그러나 **모든 phase에서 9-DOF를 동시에 사용하지는 않는다**.

### 2.2 Phase별 DOF 분석

| Phase | Base | q[0] | q[1:6] | Active DOF | 제약 |
|---|---|---|---|---|---|
| **Approach** | $(x,y,\theta)$ 이동 | Tucked or pre-orient | Tucked | 3 (base only) | Obstacle avoidance |
| **Manipulation** | **정지** | Object 방향 orient | EE pose 달성 | 6 (arm only) | $q_0$ = f(object direction) |
| **Transport** | $(x,y,\theta)$ 이동 | **고정** (grasp 유지) | **고정** (grasp 유지) | 3 (base only) | Object collision, joint limits |
| **Place** | **정지** | Orient to target | EE pose 달성 | 6 (arm only) | Object in hand |

### 2.3 핵심 관찰

> **Phase 2(Manipulation)과 Phase 4(Place)만이 arm path planning이 필요하다.** Phase 1, 3은 순수 base navigation이다.

그러나 Phase 간의 **연결(transition)**에서 문제가 발생한다:
- Phase 2 종료 시의 arm configuration이 Phase 3의 초기 조건을 결정
- Phase 3 종료 시의 base pose가 Phase 4의 arm workspace를 결정

---

## 3. Post-Grasp Configuration 문제

### 3.1 현재 시스템의 맹점

현재 map은 `(x, y, z, roll, pitch) → μ`로, **EE가 어떤 pose에 도달하는 것을 목적**으로 설계되어 있다. 즉 "grasping을 향해 가는 경로"만 계획한다.

Grasping 성공 후:
- Arm은 object를 잡은 configuration $\mathbf{q}_{\text{grasp}}$에 있음
- 이 configuration에서 **"transport configuration"**으로 전환해야 함
- Transport configuration: base 이동 중 안전한 자세 (보통 object를 body 가까이 tucking)

### 3.2 해결 구조: Bi-directional Arm Planning

```
[Grasp Phase]
  Home → (경로 A) → Grasp Pose     ← 현재 연구의 범위
  
[Post-Grasp Phase]
  Grasp Pose → (경로 B) → Transport Pose    ← 새롭게 필요
  
[Place Phase]
  Transport Pose → (경로 C) → Place Pose    ← Grasp Phase의 mirror
```

#### 경로 B의 특수성

경로 B는 경로 A와 **근본적으로 다른 문제**이다:

| 속성 | 경로 A (Approach to Grasp) | 경로 B (Grasp to Transport) |
|---|---|---|
| Start | Home (known, fixed) | Grasp pose (task-dependent) |
| Goal | Object pose (task-dependent) | Transport pose (design choice) |
| EE Payload | 없음 | Object in hand |
| 관심사 | μ 최적화 (singularity 회피) | Collision avoidance + μ |
| Map 사용 | 동일 map 재사용 가능 | **Map 재사용 가능** |

> **핵심**: 경로 B도 task-space에서의 arm path planning이므로, **현재의 manipulability map과 A\* 알고리즘을 그대로 재사용할 수 있다**. Start/Goal만 바뀔 뿐이다.

다만, object를 잡고 있으므로:
- Collision check에 object geometry 추가 필요
- Payload에 의한 dynamic 변화 (이것은 path planning 수준에서는 무시 가능)

### 3.3 Transport Pose 설계

Transport pose는 **설계 변수(design variable)**이다:

$$\mathbf{q}_{\text{transport}} = \argmin_{\mathbf{q}} \, J(\mathbf{q})$$

$$\text{s.t.} \quad \text{no collision}, \quad \mathbf{q} \in [\mathbf{q}_{\min}, \mathbf{q}_{\max}], \quad \mu(\mathbf{q}) > \mu_{\text{th}}$$

비용 함수 $J(\mathbf{q})$의 후보:
- **Manipulability 최대화**: $J = -\mu(\mathbf{q})$ → singularity에서 먼 configuration
- **Joint 중앙 선호**: $J = \|\mathbf{q} - \mathbf{q}_{\text{mid}}\|^2$ → joint limit에서 먼 configuration
- **CoG 최적화**: 중심이 base 위에 오도록 → 주행 안정성

실용적으로는 **미리 정해진 transport pose** (예: "물건을 든 채로 팔을 body 옆에 접는 자세")를 사용하는 경우가 대부분이다. 이것은 로봇마다 한 번 설계하면 재사용 가능하다.

---

## 4. q[0]의 역할 재정의: 핵심 분석

### 4.1 현재 시스템에서의 q[0]

현재 manipulator의 kinematic 구조 (`map_final.py` 기반):

```
Base(고정) → q[0](yaw/Rz) → q[1](Rz) → q[2](Rz) → q[3](Rz) → q[4](Rz) → q[5](Rz) → EE
```

- **q[0]은 global coordinate 기준으로 순수 yaw 회전만 수행**
- ARM_BASE_Z = 0.35146m 위에 설치
- Joint limits: q[0] ∈ [-2.69, 2.69] rad (≈ ±154°)

현재 map 생성에서 q[0]은 6개 joint 중 하나로, IK solution의 일부이다. **x=0 평면(sagittal plane)에서만 map을 생성**하고 있으므로, q[0]은 사실상 0 근처에서의 해를 찾는 구조이다.

### 4.2 q[0]의 물리적 의미: Base가 움직일 때

Mobile base가 $(x_b, y_b, \theta_b)$를 제공하는 상황에서, **기존 grasping-only 시나리오에서 base가 정지**한 상태로 object를 향해 arm을 뻗을 때:

1. Base가 object를 정면으로 바라보는 위치에 정지: $\theta_b = \text{atan2}(y_{\text{obj}} - y_b, x_{\text{obj}} - x_b)$
2. q[0] ≈ 0 (base가 이미 object를 향해 정렬됨)
3. q[1:5]가 EE pose를 달성

이것이 현재 연구의 암묵적 가정이다. 그리고 map이 x=0 평면에서 계산된 이유이기도 하다.

### 4.3 Counter-Rotation 전략: 핵심 제안

**제안**: Base가 이동 중일 때, q[0]를 base yaw의 반대 방향으로 회전시키면:

$$q_0(t) = -\Delta\theta_b(t) + q_{0,\text{nominal}}$$

여기서 $\Delta\theta_b(t) = \theta_b(t) - \theta_b(t_{\text{stop}})$은 정지자세 대비 base의 yaw 변화량.

#### 수학적 동치성 증명

EE의 global frame에서의 orientation:

$$R_{\text{EE}}^{\text{global}} = R_z(\theta_b) \cdot R_z(q_0) \cdot R_{\text{rest}}(q_1, \ldots, q_5)$$

여기서 $R_{\text{rest}}$는 q[1]~q[5]에 의한 나머지 변환.

Counter-rotation 적용: $q_0 = -\Delta\theta_b + q_{0,\text{nominal}}$

$$R_z(\theta_b) \cdot R_z(-\Delta\theta_b + q_{0,\text{nom}}) = R_z(\theta_b) \cdot R_z(-\Delta\theta_b) \cdot R_z(q_{0,\text{nom}})$$

$$= R_z(\theta_b - \Delta\theta_b) \cdot R_z(q_{0,\text{nom}})$$

$$= R_z(\theta_{b,\text{stop}}) \cdot R_z(q_{0,\text{nom}})$$

> **결론**: q[0]가 base yaw의 변화를 정확히 상쇄하면, **q[1:5]가 바라보는 "arm-local frame"은 정지 상태와 정확히 동일**하다. 이것은 수학적으로 base가 정지해 있을 때의 grasping 시나리오와 **완벽한 동치(equivalence)**이다.

#### 현실적 유효성

이 전략이 물리적으로도 유효한 이유:

1. **Position 관점**: Base가 이동하면 arm base의 global position도 이동한다. 그러나 **EE target도 base와 상대적으로 고정된 관계**(object를 이미 잡은 상태에서 transport하는 경우)이면, position 문제도 소멸한다.
2. **Velocity 관점**: $\dot{q}_0 = -\dot{\theta}_b$. Mobile base의 angular velocity가 일반적으로 0.5~2.0 rad/s 이므로, q[0]의 속도 요구는 충분히 feasible하다.
3. **Joint limit 관점**: q[0] ∈ [-2.69, 2.69] rad ≈ ±154°. Base의 yaw 변화 $\Delta\theta_b$가 ±154° 이내이면 항상 feasible. 일반적인 navigation에서 이 범위를 초과하는 것은 극히 드물다.

---

## 5. Effective 5DOF 문제: 이것이 핵심 설계 질문

### 5.1 문제의 본질

Counter-rotation을 적용하면, q[0]는 더 이상 **자유 변수가 아니다**. Base의 yaw에 의해 **구속(constrained)**된다:

$$q_0 = f(\theta_b) = -\Delta\theta_b + q_{0,\text{nom}}$$

따라서 나머지 q[1:5] (5 DOF)만이 EE pose를 달성해야 한다. **6DOF EE pose를 5DOF로 달성해야 하므로, 1 DOF가 부족하다.**

### 5.2 현재 시스템에서 이미 일어나고 있는 일

그런데 잘 생각해보면, **현재 시스템에서도 사실상 같은 상황이 벌어지고 있다**:

- 현재 map 생성: x=0 평면에서 EE pose = (x, y=0, z, roll, pitch) + yaw는 0°로 고정
- q[0]의 역할: sagittal plane 내에서 IK를 풀 때, q[0]은 **y=0을 유지하도록 거의 0 근처 값**을 가짐
- 실질적으로 q[0]은 "어느 방향을 바라보는가"만 결정하고, EE pose의 세밀한 달성은 q[1:5]가 담당

즉, **현재 시스템에서도 q[0]는 이미 "방향 정렬"이라는 단일 목적에 구속되어 있고, 실절적 IK는 q[1:5]의 5DOF로 풀리고 있는 것과 마찬가지**이다.

> **핵심 통찰**: Counter-rotation은 새로운 제약을 추가하는 것이 아니라, **기존에 암묵적으로 존재하던 구속을 명시화하는 것**이다.

### 5.3 5DOF로 6DOF EE Pose를 달성하는 방법론

5 DOF (q[1:5])로 6 DOF EE pose를 달성하는 것은 일반적으로 불가능하다. 해결 전략:

#### 전략 A: EE Orientation 1-DOF Relaxation (권장)

6DOF EE pose 중 1개 orientation DOF를 relaxation한다:

$$\text{EE Task} = (x, y, z, \text{pitch}, \text{yaw/roll 중 하나})$$

**현재 시스템에서 이미 하고 있는 것**과 정확히 일치한다:
- 현재 map: (x, y, z, roll, pitch) → 5개 task DOF
- yaw는 0°로 고정 (= relaxation되어 있지 않고 고정)
- 하지만 q[0]이 yaw 정렬을 맡으므로, 실질적으로는 (x, z, pitch)의 3개 task DOF + (y=0, roll=고정)

**Counter-rotation을 적용하면**:
- q[0]가 방향 정렬에 명시적으로 구속
- q[1:5]가 (x, z, pitch) + 가능한 경우 roll 달성
- Map 구조: **현재와 동일하게 유지 가능**

#### 전략 B: Whole-Body Jacobian (확장 연구용)

$$\mathbf{J}_{\text{WB}} = \begin{bmatrix} \mathbf{J}_{\text{base}} & \mathbf{J}_{\text{arm}} \end{bmatrix} \in \mathbb{R}^{6 \times 9}$$

9 DOF (base 3 + arm 6)를 동시에 사용하여 6 DOF EE task를 달성. Redundancy = 3. 이것은 **holistic control 확장 연구**의 범위이다.

### 5.4 결론: 5DOF로 봐야 하는가?

> **아니다. 정확히 말하면, "q[0]가 방향 정렬에 구속된 6DOF"로 봐야 한다.**

이것은 5DOF 로봇과는 근본적으로 다르다:
- 5DOF 로봇: 물리적으로 관절이 5개
- 현재 상황: 물리적으로 6개 관절이 있지만, 1개가 task에 의해 구속

차이점:

| 구분 | 물리적 5DOF | q[0]-구속 6DOF |
|---|---|---|
| 관절 수 | 5 | 6 |
| 자유 변수 | q[0:5] | q[1:5] (q[0]은 $f(\theta_b)$) |
| Manipulability | $\mu = \sqrt{\det(\mathbf{J}_5 \mathbf{J}_5^T)}$ (5×5) | $\mu$ 계산은 6×6이지만, q[0] 고정 시 **reduced Jacobian으로도 표현 가능** |
| Map 호환성 | 완전히 새로운 map 필요 | **현재 map 구조와 호환** |

### 5.5 Reduced Jacobian 관점

q[0]가 구속될 때, full Jacobian을 partition:

$$\mathbf{J} = \begin{bmatrix} \mathbf{j}_0 & \mathbf{J}_{1:5} \end{bmatrix} \in \mathbb{R}^{6 \times 6}$$

q[0]가 고정(또는 prescribed)이므로:

$$\dot{\mathbf{x}} = \mathbf{j}_0 \dot{q}_0 + \mathbf{J}_{1:5} \dot{\mathbf{q}}_{1:5}$$

Counter-rotation에서 $\dot{q}_0 = -\dot{\theta}_b$ (known)이므로:

$$\dot{\mathbf{x}} - \mathbf{j}_0 (-\dot{\theta}_b) = \mathbf{J}_{1:5} \dot{\mathbf{q}}_{1:5}$$

$$\dot{\mathbf{x}}_{\text{eff}} = \mathbf{J}_{1:5} \dot{\mathbf{q}}_{1:5}$$

이것은 **5×6이 아니라 6×5 Jacobian** ($\mathbf{J}_{1:5} \in \mathbb{R}^{6\times5}$). 따라서 $\mathbf{J}_{1:5}\mathbf{J}_{1:5}^T \in \mathbb{R}^{6\times6}$이고, **rank deficiency가 1** (최소 1개 singular value가 0이 되어야 함).

이것은 arm이 **특정 방향으로의 task-space velocity를 독립적으로 제어할 수 없음**을 의미한다. 그 방향이 바로 q[0]의 회전축(z축)에 해당하는 **arm base 주변의 azimuthal 방향**이다.

> **그러나 이 "잃어버린 방향"은 바로 q[0]의 counter-rotation이 담당하는 방향이므로, 전체 시스템(base + arm)에서는 DOF가 보존된다.** 이것이 counter-rotation의 수학적 정당성이다.

### 5.6 Manipulability Map에 대한 함의

**핵심 질문**: counter-rotation을 적용하면 기존 manipulability map을 다시 계산해야 하는가?

**답: 아니다.** 그 이유:

1. 현재 map은 `(x, y=0, z, roll, pitch) → μ_best`로 계산됨
2. μ 계산에서 q[0]의 기여는 **arm이 가리키는 azimuthal 방향과 무관**
   - $\mu = \sqrt{\det(\mathbf{J}\mathbf{J}^T)}$에서, q[0]가 0°이든 45°이든 $\mu$ 값은 동일 (rotationally symmetric)
3. q[0]가 0°일 때의 map = q[0]가 $-\Delta\theta_b$일 때의 map (회전 대칭)

> **Map은 arm-base 기준 좌표계에서 계산되므로, global frame에서의 q[0] 값과 무관하다.** Counter-rotation은 global frame에서의 arm-base 좌표계 방향만 바꿀 뿐, 그 좌표계 내에서의 workspace geometry는 변하지 않는다.

---

## 6. 전체 아키텍처 제안: Counter-Rotation 통합

### 6.1 Hierarchical Planning Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Level 3: Task Planner                                      │
│  [Pick object at T_obj] → [Place at T_place]                │
│  Output: task sequence + constraints                        │
├─────────────────────────────────────────────────────────────┤
│  Level 2: Base Pose Selector (IRM-based)                    │
│  Input: T_obj, 5D Tensor Map                                │
│  Output: (x_b, y_b, θ_b)_grasp, (x_b, y_b, θ_b)_place     │
│  Method: IRM lookup + μ optimization                        │
│  ★ NEW: θ_b 결정 → q_0,nom 결정                             │
├─────────────────────────────────────────────────────────────┤
│  Level 1a: Base Path Planner (Nav2)                         │
│  Start → Grasp Pose → Place Pose → End                     │
│  Method: Standard navigation (obstacle avoidance)           │
├─────────────────────────────────────────────────────────────┤
│  Level 1b: Arm Path Planner (현재 연구)                      │
│  Home → EE Grasp Pose  (Phase 2)                            │
│  Grasp Config → Transport Config  (Phase 2→3 transition)    │
│  Transport Config → EE Place Pose  (Phase 4)                │
│  Method: Manipulability-Aware A* on 5D Tensor Map           │
├─────────────────────────────────────────────────────────────┤
│  Level 0: Whole-Body Controller                             │
│  ★ q_0(t) = -Δθ_b(t) + q_0,nom (counter-rotation)         │
│  ★ q[1:5]: resolved motion control under reduced Jacobian  │
│  Method: QP-based whole-body control or prioritized IK      │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 Data Flow with Counter-Rotation

```
5D Tensor Map ──┬──→ IRM (Level 2): Base pose selection → θ_b → q_0,nom
                │
                └──→ A* Planner (Level 1b): Arm path planning (q[1:5] space)
                      ├─ Phase 2: Home → Grasp (base 정지, q[0]≈q_0,nom)
                      ├─ Transition: Grasp → Transport (base 정지)
                      └─ Phase 4: Transport → Place (base 정지, q[0]≈q_0,nom)

                Nav2 (Level 1a): Base navigation
                      ├─ Phase 1: Start → Grasp Position
                      ├─ Phase 3: Grasp Position → Place Position
                      │   └─ ★ q_0(t) = -Δθ_b(t) + q_0,nom (실시간)
                      └─ Phase 5: Place Position → End
```

### 6.3 핵심 포인트

> **5D Tensor Map은 한 번만 계산하면 Level 1b (arm planning)과 Level 2 (base pose selection) 모두에 재사용된다. Counter-rotation은 Level 0에서 처리되므로, map이나 planner의 수정이 전혀 필요 없다.**

---

## 7. Smooth Base Trajectory Planning

### 7.1 3-Waypoint Navigation with Orientation Constraints

```
Start (x₀,y₀,θ₀) ──→ Grasp Pose (x₁,y₁,θ₁) ──→ Place Pose (x₂,y₂,θ₂)
```

각 waypoint에서의 **정지 자세($\theta$)가 constrained**:
- $\theta_1$: object 방향을 향해야 함 (IRM에서 결정)
- $\theta_2$: place target 방향을 향해야 함

### 7.2 Smooth Trajectory 생성

Waypoint 간 smooth trajectory는 **Bézier curve** 또는 **Dubins path**로 생성:

$$\mathbf{p}(t) = \text{Bézier}(\mathbf{p}_0, \mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3)$$

Nav2의 `Smac Planner`나 `Regulated Pure Pursuit`이 이 문제를 이미 잘 처리한다.

### 7.3 Counter-Rotation during Transport (Phase 3)

Phase 3에서 base가 이동하면서 q[0]가 counter-rotate:

$$q_0(t) = -(\theta_b(t) - \theta_{b,\text{grasp}}) + q_{0,\text{grasp}}$$

이것의 의미:
- **Object를 잡은 arm의 전체 gesture가 global frame에서 일정하게 유지**
- Base가 어떤 궤적으로 이동하든, arm이 잡은 object는 같은 방향을 가리킴
- **주행 중 object가 흔들리거나 방향이 바뀌는 것을 방지**

Joint limit check:

$$q_0(t) \in [-2.69, 2.69] \quad \forall t$$

$$\Leftrightarrow \quad |\Delta\theta_b(t)| < 2.69 - |q_{0,\text{grasp}}| \quad \text{rad}$$

만약 $q_{0,\text{grasp}} \approx 0$이면 (base가 object를 정면으로 바라본 채 grasp), base는 최대 ±154° 회전 가능. 충분한 여유이다.

---

## 8. Map 재설계가 필요한가?

### 8.1 결론: 전면 재설계는 불필요. 확장(Extension)으로 충분.

현재 map의 구조:

$$\mathcal{M}: (x, y, z, \text{roll}, \text{pitch}) \to \mu_{\text{best}}$$

이것은 **base가 고정된 상태에서의 arm workspace manipulability**를 인코딩한다.

### 8.2 Whole body로 확장할 때 변하는 것과 변하지 않는 것

| 요소 | 변화? | 이유 |
|---|---|---|
| **Map 데이터 자체** | ✗ | Base-relative frame에서 계산 → rotationally symmetric |
| **Map의 좌표계** | ✗ → base-relative | Map은 base frame 기준이므로 counter-rotation과 무관 |
| **EE orientation** | 가능한 변경 | 현재: fixed → 확장: relaxation 가능 |
| **q[0] 처리** | 불변 | Counter-rotation은 Level 0에서 처리, map에 영향 없음 |
| **A\* 알고리즘** | ✗ | Start/Goal만 변경하면 됨 |
| **Cost function** | ✗ | Multiplicative cost 구조 그대로 |

### 8.3 유일하게 새로 필요한 것: Base Pose Selection

**"어디에서 정지해서 grasping을 할 것인가?"** — 이것이 whole body planning의 핵심 추가 요소.

이것은 **Inverse Reachability Map (IRM)** 문제이다 (Zacharias et al., ICRA 2007):

$$\text{IRM}: T_{\text{object}} \to \{(x_b, y_b, \theta_b) : \text{reachable with } \mu > \mu_{\text{th}}\}$$

Object pose가 주어졌을 때, base가 취할 수 있는 pose들의 집합과 각 pose에서의 manipulability.

#### IRM 계산: 기존 5D Map으로부터 직접 파생

현재 5D tensor map으로부터 IRM을 **직접 파생**할 수 있다:

1. Object pose $(x_o, y_o, z_o)$ 주어짐
2. Map에서 해당 EE pose에 도달 가능한 영역 확인 → base-relative 좌표에서의 위치
3. Base pose = "arm base에서 object까지의 상대 위치를 만족시키는 global base pose"
4. 각 base pose 후보에서의 $\mu$ 값 = map에서의 직접 lookup
5. $\mu$가 높은 base pose 선택 + $\theta_b = \text{atan2}(\Delta y, \Delta x)$ → q[0,nom] ≈ 0

> **현재의 5D map이 IRM의 building block이 된다.** 새로운 map을 처음부터 만드는 것이 아니다.

---

## 9. Post-Grasp 경로 문제의 구체적 해결

### 9.1 경로 B (Grasp → Transport) 계획

현재 A\* 알고리즘으로 **즉시 재사용 가능**:

```python
# Phase 2: Approach to grasp
path_A, _ = planner.search(w=5.0, kappa=3.0)  # Home → Grasp EE pose

# Object grasped — arm is now at q_grasp

# Phase 2→3 transition: Grasp → Transport
planner.start = grasp_grid_pos     # Grasp pose의 EE position (grid 좌표)
planner.goal = transport_grid_pos  # Transport pose의 EE position (grid 좌표)
path_B, _ = planner.search(w=5.0, kappa=3.0)  # Grasp → Transport

# Phase 4: Transport → Place  
planner.start = transport_grid_pos
planner.goal = place_grid_pos
path_C, _ = planner.search(w=5.0, kappa=3.0)  # Transport → Place
```

### 9.2 Map의 유효성

경로 B에서 object를 잡고 있으므로:
- **Object가 manipulator에 부착** → collision geometry 변경
- **μ 값 자체는 변하지 않음** (μ는 joint configuration에만 의존, payload 무관)
- Collision은 별도 check 필요 (map 밖에서 처리)

따라서 **manipulability map은 그대로 사용 가능**하지만, collision avoidance layer는 별도로 업데이트해야 한다.

---

## 10. 직접적인 답변

### Q1: "Grasping 후의 위치가 고려되지 않는데 어떻게 해야 하는가?"

**현재 map과 A\* 알고리즘을 Start/Goal만 바꿔서 재사용한다.** Grasp pose → Transport pose 경로를 현재 시스템으로 바로 계획 가능하다. 새로운 map이나 algorithm은 불필요하다. Collision check만 object geometry를 추가하면 된다.

### Q2: "q[0]이 객체를 바라보는 용도라면 map/planning을 다시 해야 하는가?"

**다시 할 필요 없다.** q[0]를 base yaw의 counter-rotation으로 사용하면, arm-local frame은 base 정지 상태와 수학적으로 동치이다. 현재 map은 arm-base 기준 좌표계에서 계산되었으므로, q[0]의 global 방향과 무관하게 유효하다.

### Q3: "q[0] counter-rotation을 적용하면 effective 5DOF로 다시 진행해야 하는가?"

**아니다. 현재 시스템을 수정할 필요가 없다.** 그 이유:
1. 현재 map 생성에서도 q[0]은 사실상 "방향 정렬"에 구속되어 있었음 (x=0 평면에서의 IK)
2. Counter-rotation은 이 암묵적 구속을 Level 0 controller에서 명시화하는 것일 뿐
3. Map 안에서의 μ는 q[0] 값과 무관 (rotational symmetry)
4. q[1:5]의 workspace와 manipulability는 변하지 않음

**Counter-rotation은 "새로운 제약"이 아니라 "기존 가정의 명시화"**이다.

### Q4: "전면 재설계가 최선인가?"

**아니다. 확장(extension)이 최선이다.** 현재 5D tensor map이 그대로 IRM의 building block이 되고, A\* algorithm도 Start/Goal 변경만으로 모든 phase에 재사용된다. 새로 추가해야 하는 것은 **Level 2: Base Pose Selector (IRM)** 하나뿐이고, counter-rotation은 **Level 0: Controller**에서 $q_0(t) = -\Delta\theta_b(t) + q_{0,\text{nom}}$ 한 줄이면 된다.

---

## 11. 논문 스토리와의 연결

### 현재 논문 (ICRA/IROS)

```
Contribution: 5D Tensor Map + Manipulability-Aware A* + Inflation Layer
Scope: Fixed base, single manipulation phase
Message: "Precomputed map으로 실시간 manipulability-aware planning 가능"
```

### 확장 논문 (RA-L / T-RO) — Whole Body Control

```
Contribution: 5D Tensor Map → IRM 파생 → Counter-Rotation → Whole Body Planning
Scope: Mobile manipulator, multi-phase task
Key Insight: "q[0] counter-rotation이 mobile manipulation을 
             fixed-base manipulation과 수학적으로 동치로 만든다"
Message: "단일 precomputed map이 base pose selection부터 arm planning까지
          전체 mobile manipulation pipeline의 foundation으로 기능.
          Counter-rotation에 의해 이동 중에도 arm workspace가 보존된다."
```

### Holistic Control 확장 (IJRR / T-RO Full Paper)

```
Contribution: Whole-Body Jacobian + Task Priority + Dynamic Consistency
Scope: Simultaneous base+arm control, dynamic environments
Key Insight: "Counter-rotation의 자연스러운 일반화가 whole-body QP control"
Message: "Counter-rotation은 task-priority framework의 가장 비용이 다른 (degenerate) 특수 case.
          Whole-body Jacobian으로 확장하면 counter-rotation이
          자연스럽게 QP의 한 해로 복원된다."
```

> **현재 논문의 5D Tensor Map → 확장 논문의 IRM + Counter-Rotation → Holistic control의 Whole-Body QP**로 이어지는 자연스러운 연구 progression이 구축된다. 각 단계가 이전 단계의 자연스러운 일반화이며, **한 번 계산한 5D map이 모든 단계에서 재사용**된다는 것이 일관된 연구 스토리의 핵이다.

---

## 12. Critical Discussion: Counter-Rotation의 한계와 방어

### 12.1 예상 공격 포인트

| 공격 | 방어 |
|---|---|
| "Counter-rotation은 trivial하다" | **수학적 동치성 증명**이 핵심. Trivial해 보이지만 기존 연구에서 이러한 형식적 증명 + precomputed map 통합은 제시된 적 없음 |
| "Joint limit으로 인한 feasibility" | q[0] range = ±154°. 일반적 navigation의 yaw 변화량 << 이 범위. Infeasible case는 base re-orientation으로 해결 |
| "Velocity coupling 무시" | Counter-rotation은 kinematic level. Dynamic coupling ($\mathbf{M}_{qb}\ddot{\theta}_b$ 항)은 low-speed regime에서 무시 가능. High-speed는 whole-body dynamics 필요 → holistic control 확장으로 자연스럽게 연결 |
| "왜 whole-body Jacobian을 바로 안 쓰는가?" | Whole-body QP는 online 계산 → latency. Counter-rotation은 **precomputed map 재활용 + O(1) controller** → edge device에서 실시간 가능. Computational hierarchy의 적절한 위치에서의 설계 |

### 12.2 Counter-Rotation이 Holistic Control의 Special Case임을 보이는 방법

Whole-body QP formulation:

$$\min_{\dot{\mathbf{q}}_{\text{full}}} \|\dot{\mathbf{q}}_{\text{full}}\|^2 \quad \text{s.t.} \quad \mathbf{J}_{\text{WB}} \dot{\mathbf{q}}_{\text{full}} = \dot{\mathbf{x}}_{\text{des}}$$

여기서 $\dot{\mathbf{q}}_{\text{full}} = (\dot{x}_b, \dot{y}_b, \dot{\theta}_b, \dot{q}_0, \ldots, \dot{q}_5)^T$

**Counter-rotation constraint**: $\dot{q}_0 = -\dot{\theta}_b$

이것을 QP에 equality constraint로 추가하면:

$$\min_{\dot{\mathbf{q}}_{\text{full}}} \|\dot{\mathbf{q}}_{\text{full}}\|^2 \quad \text{s.t.} \quad \mathbf{J}_{\text{WB}} \dot{\mathbf{q}}_{\text{full}} = \dot{\mathbf{x}}_{\text{des}}, \quad \dot{q}_0 + \dot{\theta}_b = 0$$

> Holistic control에서 counter-rotation constraint를 제거하면, QP solver가 자동으로 더 나은 (minimum-norm) 해를 찾는다. **Counter-rotation은 holistic control의 suboptimal special case이며, precomputed map과의 호환성을 대가로 최적성을 일부 희생하는 설계 선택**이다. 이것이 연구의 progression을 정당화한다.
