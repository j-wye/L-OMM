# L-OMM: Language-guided Observability-constrained Manifold Manipulation

이 문서는 L-OMM 연구의 기준 연구 주제를 정의한다. 목적은 개별 구현 아이디어를 나열하는 것이 아니라, Perception, Decision, Active-Map Planning, Reference Generation, DLS Control이 하나의 edge-deployable mobile manipulation pipeline으로 어떻게 수학적으로 닫히는지를 명확히 고정하는 것이다.

본 연구의 핵심은 다음 한 문장으로 요약된다.

> 자연어로 지정된 물체를 edge device에서 실시간으로 인지하고, side-contact task가 요구하는 관측 가능성, target-contact geometry, 기구학적 실행 가능성을 reduced active manifold 위로 축소한 뒤, precomputed reduced manipulability map과 runtime final active mask를 이용해 execution-aware skeleton을 deterministic하게 선택하고 DLS로 검증하는 mobile manipulation architecture.

중요한 점은 본 연구가 새로운 거대 모델, 새로운 CUDA kernel, 새로운 end-to-end learned controller를 만드는 연구가 아니라는 것이다. 기여는 이미 존재하는 perception backbone, geometric decision logic, deterministic graph search, DLS control을 L-OMM task structure에 맞게 수학적으로 재구성하고, 그 전체를 Jetson-class edge device에서 실행 가능한 pipeline으로 닫는 데 있다.

---

## 1. System-Level Problem Statement

시간 \(t\)에서 시스템 입력은 다음으로 구성된다.

$$
\mathcal{I}_t =
\left(
I_t,\;D_t,\;\ell,\;T^m_o(t),\;q_t
\right),
$$

where \(I_t\) is RGB image, \(D_t\) is aligned depth, \(\ell\) is a natural-language command, \(T^m_o(t)\in SE(3)\) is the transform from optical frame to map frame, and \(q_t\) is the manipulator state.

시스템은 다음 출력을 순차적으로 생성한다.

1. target object mask and bounding box,
2. side-contact target geometry and grasp feasibility flags,
3. an active-manifold path skeleton,
4. a continuous reference and reduced active joint command.

전체 문제는 다음 constrained real-time decision problem으로 볼 수 있다.

$$
\begin{aligned}
\text{find}\quad & \gamma,\;\Gamma,\;q_{\mathrm{act}}(t),\;u(t) \\
\text{s.t.}\quad
& \text{target object selected by } \ell,\\
& \text{side-contact geometry is feasible},\\
& \gamma \subset \mathcal{F}_{\mathrm{final}}(h,t),\\
& \Gamma \text{ is a feasible arclength reference generated from } \gamma,\\
& q_{\mathrm{act}}(t) \in [q_{\min},q_{\max}],\\
& \text{gaze, target-offset, and capsule safety constraints are respected},\\
& T_{\mathrm{loop}} \le T_{\mathrm{budget}}.
\end{aligned}
$$

여기서 \(\gamma\)는 online planner가 선택한 active-manifold skeleton이고, \(\Gamma\)는 이 skeleton에서 생성된 continuous reference다. \(\mathcal{F}_{\mathrm{final}}(h,t)\)는 height layer \(h\)에서 runtime obstacle, target, unknown, occluded, inflation, capsule-pruning semantics가 반영된 final active mask다.

이 연구의 구조적 관점은 다음이다.

$$
\text{Perception}
\rightarrow
\text{Decision}
\rightarrow
\text{Map Update}
\rightarrow
\text{Active-Manifold Planning}
\rightarrow
\text{Reference Generation}
\rightarrow
\text{DLS Execution}.
$$

각 module은 서로 다른 수학적 object를 다룬다. Perception은 semantic evidence, Decision은 side-contact target geometry, Map Update는 final active mask, Planning은 active graph skeleton, Reference Generation은 continuous task reference, Control은 reduced active joint dynamics를 다룬다. 이 책임 분리가 전체 pipeline의 실시간성, 디버그 가능성, 실험 해석 가능성을 동시에 만든다.

---

## 2. Main Contributions

본 연구의 contribution은 다음 네 축으로 정리한다.

### Contribution 1: Edge-Optimized Language-Conditioned Perception

자연어 명령을 이용해 target object를 찾고 segmentation mask를 생성하는 perception pipeline을 TensorRT, CUDA Graph, GPU-resident preprocessing 중심으로 최적화한다. 핵심은 단순 FPS 경쟁이 아니라, downstream geometric decision과 active-map update가 사용할 수 있는 mask, bounding box, depth-aligned tensor를 낮은 latency와 높은 freshness로 제공하는 것이다.

### Contribution 2: Gravity-Constrained Side-Contact Decision Module

Decision Module은 learned 6-DoF grasp regressor 대신, optical-frame raw geometry와 map-frame state를 분리하는 frame contract 위에서 side-contact target geometry를 추정한다. Side-contact task는 target rotation을 \(SO(3)\) 전체가 아니라 gravity-constrained \(S^1\) manifold 위로 축소한다.

이로써 다음 failure mode를 구조적으로 줄인다.

- frame identity fallback,
- camera-frame gravity misuse,
- PCA axis sign flip,
- unobservable yaw jitter,
- gripper stroke infeasibility,
- target-contact point와 EE-safe goal의 혼동.

### Contribution 3: Reduced Active-Manifold Map

Side-contact / gaze-constrained phase에서 필요한 active submanifold를 명시적으로 선택한다. Active joint vector는 다음과 같다.

$$
q_{\mathrm{act}}
=
\begin{bmatrix}
q_2\\
q_3\\
q_5
\end{bmatrix}.
$$

고정 posture는 다음으로 둔다.

$$
q_1=0,\qquad
q_4=\frac{\pi}{2},\qquad
q_6=-\frac{\pi}{2}.
$$

Active task coordinate는 다음이다.

$$
r=
\begin{bmatrix}
x\\
z\\
\theta_{\mathrm{gaze}}
\end{bmatrix}.
$$

Active plane은

$$
y_{\mathrm{plane}}=0.047\ \mathrm{m}
$$

로 고정한다. 이 값은 runtime tuning knob가 아니라, 현재 fixed posture와 active joint subset이 만드는 reduced task plane이다.

Offline 단계에서는 target height \(h\)마다 reduced active map을 생성한다. 각 cell은 EE reference point의 \((x,z)\) 후보이며, map은 다음 정보를 저장한다.

$$
\mu_{\mathrm{act}}(x,z;h),
\qquad
\theta_{\mathrm{gaze}}(x,z;h),
\qquad
\sigma_{\min}(x,z;h),
\qquad
q_{\mathrm{act}}^\star(x,z;h).
$$

Reduced Jacobian은

$$
J_{\mathrm{act}}
=
\frac{\partial(x,z,\theta_{\mathrm{gaze}})}
{\partial(q_2,q_3,q_5)}
\in\mathbb{R}^{3\times3}
$$

이고, reduced manipulability는

$$
\mu_{\mathrm{act}}
=
\left|\det J_{\mathrm{act}}\right|
$$

로 정의한다.

이 map은 generic reachability map이 아니다. 각 cell은 fixed active plane, target height, gaze pitch, joint limits, capsule-consistent body safety, reduced manipulability를 반영한 task-conditioned cell이다.

### Contribution 4: Manipulability-Cost Active-Manifold A* and DLS-Validated Reference

현재 main planning branch는 manipulability-cost active-manifold A*다. Distance-only A*는 삭제하지 않고 ablation baseline으로 유지한다.

핵심은 두 branch가 같은 reduced active map, 같은 final active mask, 같은 start/goal, 같은 reference generator, 같은 DLS validator, 같은 capsule validation을 공유한다는 점이다. 차이는 A* edge priority뿐이다.

Manipulability-cost branch는 같은 feasible graph 안에서도 low-\(\mu_{\mathrm{act}}\) edge를 부드럽게 불리하게 만들어, path length를 크게 늘리지 않으면서 downstream DLS execution burden을 줄이는 것을 목표로 한다.

인접 node \(n_i,n_j\)의 edge representative는 harmonic mean으로 둔다.

$$
\mu_e(n_i,n_j)
=
\frac{2\mu_i\mu_j}{\mu_i+\mu_j}.
$$

Safe level은 feasible map 내부 percentile로 둔다.

$$
\mu_{\mathrm{safe}}
=
\operatorname{percentile}
\left(
\{\mu_{\mathrm{act}}(n)\mid n\in\mathcal{F}_{\mathrm{base}}\},
25\%
\right).
$$

Penalty deficit은

$$
\delta_\mu
=
\max
\left(
0,\;
\frac{\mu_{\mathrm{safe}}-\mu_e}
{\mu_{\mathrm{safe}}-\mu_{\min}}
\right)
$$

이고, bounded penalty는

$$
\phi_\kappa
=
\tanh(\kappa\delta_\mu)
$$

다. Candidate edge cost는 다음과 같다.

$$
c_{\mathrm{manip}}(n_i,n_j)
=
c_{\mathrm{dist}}(n_i,n_j)
\left(
1+w\phi_\kappa(n_i,n_j)
\right).
$$

현재 global fixed operating point는

$$
w=0.25,\qquad \kappa=3.0
$$

이다. 이 값은 obstacle-free clear case 전체 height sweep에서 paired downstream metric을 기준으로 선택한 값이다. Obstacle layout은 runtime에서 변하므로 tuning target에 직접 넣지 않고 stress validation으로 둔다.

A* output은 trajectory가 아니라 skeleton이다. Skeleton은 local \(C^2\) reference generation, deterministic gaze pitch reconstruction, gap-aware DLS tracking, capsule validation을 거쳐 execution evidence로 평가된다.

---

## 3. Perception Module

Perception Module은 language query \(\ell\)와 RGB image \(I_t\)를 입력받아 target object의 detection과 segmentation을 수행한다.

입력:

$$
(I_t,\ell)
$$

출력:

$$
\mathcal{Y}_t =
\left(
B_t,\;M_t,\;s_t
\right),
$$

where \(B_t\) is bounding box, \(M_t\) is segmentation mask, and \(s_t\) is semantic confidence.

본 연구에서 Perception Module의 설계 원칙은 다음이다.

1. Text encoder는 language command가 바뀔 때만 실행하고, text embedding은 cache한다.
2. Image encoder와 segmentation image encoder는 가능한 병렬화한다.
3. Downstream Decision과 Map Update가 사용할 tensor를 GPU memory resident 상태로 유지한다.
4. CPU-GPU transfer는 result extraction에 필요한 최소 부분으로 제한한다.

Perception Module의 novelty는 새로운 vision backbone 자체가 아니라, edge device에서 natural-language segmentation을 real-time manipulation loop의 앞단으로 넣을 수 있도록 pipeline을 재구성한 데 있다.

---

## 4. Decision Module

Decision Module은 Perception output과 depth를 이용해 side-contact target geometry를 계산한다.

입력:

$$
\left(
M_t,\;B_t,\;D_t,\;T^m_o(t)
\right).
$$

출력:

$$
T^m_{\mathrm{target}}
=
\begin{bmatrix}
R^m_T & p^m_T\\
0 & 1
\end{bmatrix},
\qquad
\text{feasibility flags}.
$$

### 4.1 Frame Contract

Raw geometry는 optical frame에서 생성하고, persistent state와 final output은 map frame에서 정의한다.

$$
p^m = R^m_o p^o + t^m_o.
$$

이 contract는 단순한 구현 규칙이 아니라 failure prevention mechanism이다. Optical frame에서 gravity를 상수로 취급하면 camera pitch 변화에 의해 vertical axis가 잘못 추정된다. 따라서 gravity는 map frame에서 정의된 상수 방향이고, optical frame에서는

$$
\hat g_o = (R^m_o)^T \hat g_m
$$

로 해석되어야 한다.

### 4.2 Side-Contact Manifold

Side-contact에서는 approach axis와 closing axis가 horizontal plane과 정합되어야 하며, gripper upright condition이 유지되어야 한다. 이때 target rotation은 \(SO(3)\) 전체가 아니라 gravity-constrained 1D manifold 위로 축소된다.

$$
R^m_T
=
\left[
\hat x^m_{\mathrm{approach}}\;\middle|\;
\hat y^m_{\mathrm{closing}}\;\middle|\;
\hat z^m_{\mathrm{grav}}
\right],
\qquad
\hat z^m_{\mathrm{grav}}=(0,0,1)^T.
$$

수평면 위 yaw는 \(S^1\) element로 다루며, orientation smoothing이 필요할 경우 Euclidean averaging이 아니라 circular 또는 Lie-group-consistent update를 사용한다.

### 4.3 Target Contact and EE-Safe Goal

Target object는 robot body가 들어가면 안 되는 occupied volume으로 취급한다. 그러나 tool contact point는 target surface를 바라보고 접근해야 한다. 따라서 target/contact/gaze point와 EE-safe goal은 분리한다.

Canonical geometry는 다음이다.

$$
x_{\mathrm{target}}=0.53\ \mathrm{m},
\qquad
x_{\mathrm{EE}}=0.40\ \mathrm{m},
\qquad
x_{\mathrm{target}}-x_{\mathrm{EE}}=0.13\ \mathrm{m}.
$$

Path planning goal은 target point가 아니라 EE-safe point다.

$$
p_{\mathrm{goal}}
=
\begin{bmatrix}
x_{\mathrm{EE}}\\
h
\end{bmatrix}.
$$

반면 gaze pitch는 target/contact point를 기준으로 재구성한다.

$$
\theta_{\mathrm{gaze}}(x,z;h)
=
\operatorname{atan2}
\left(
x_{\mathrm{target}}-x,\;
h-z
\right).
$$

이 분리는 “target을 obstacle로 취급하면서도 tool contact task를 유지하는” 핵심 조건이다.

---

## 5. Reduced Active Map Generation

### 5.1 Map Definition

Target height \(h\)에 대해 active map은 \((x,z)\) grid로 정의된다. 각 node는

$$
n=(i,j),
\qquad
p(n)=(x_i,z_j)
$$

이고, reduced task reference는

$$
r(n;h)
=
\begin{bmatrix}
x_i\\
z_j\\
\theta_{\mathrm{gaze}}(x_i,z_j;h)
\end{bmatrix}.
$$

각 cell에서 reduced active IK를 풀어

$$
q_{\mathrm{act}}^\star(n)
=
\operatorname{IK}_{\mathrm{red}}(r(n;h))
$$

를 얻는다. 이 solution이 joint limits, reduced Jacobian conditioning, capsule-consistent safety condition을 만족하면 valid cell로 저장한다.

### 5.2 Map Channels

Map은 최소한 다음 정보를 갖는다.

$$
\mathcal{M}(x,z,0)=\mu_{\mathrm{act}}(x,z;h),
$$

$$
\mathcal{M}(x,z,1)=\theta_{\mathrm{gaze}}(x,z;h),
$$

$$
\mathcal{M}(x,z,2)=\sigma_{\min}(x,z;h).
$$

Sidecar q-map은

$$
q_{\mathrm{act}}^\star(x,z;h)
$$

를 저장한다. 이 q-map은 planning 전 capsule pruning과 DLS validation consistency를 위해 필요하다.

### 5.3 Base Feasible Layer

Base feasible layer는 다음과 같다.

$$
\mathcal{F}_{\mathrm{base}}(h)
=
\left\{
n\in\mathcal{G}_h
\mid
\mu_{\mathrm{act}}(n)>\mu_{\min}
\right\}.
$$

\(\mu_{\min}\)은 cost parameter가 아니라 graph에서 제거할 kinematic risk boundary다. 너무 높이면 graph connectivity가 깨질 수 있고, 너무 낮으면 DLS damping exposure가 커질 수 있다. 따라서 \(\mu_{\min}\)은 connectivity와 execution validation 사이의 trade-off parameter다.

---

## 6. Runtime Map Update Layer

Runtime map update는 Control Module 내부가 아니라 별도 layer의 책임이다. 이 layer는 camera/perception/decision 결과를 받아 final active mask와 blocked mask를 만든다.

공간 상태는 최소한 다음으로 나눈다.

- free region,
- occupied region,
- target region,
- unknown region,
- occluded region,
- inflated safety region.

초기 연구 구현에서는 unknown과 occluded region을 conservative하게 blocked 처리한다. 이유는 NeRF나 dense reconstruction 없이 보이는 표면만 free로 간주하면, object 뒤쪽의 불확실한 영역으로 arm이 들어갈 수 있기 때문이다.

Runtime final mask는 다음 의미를 갖는다.

$$
\mathcal{F}_{\mathrm{final}}(h,t)
=
\mathcal{F}_{\mathrm{base}}(h)
\setminus
\left(
\mathcal{O}_{\mathrm{occ}}
\cup
\mathcal{O}_{\mathrm{target}}
\cup
\mathcal{O}_{\mathrm{unk}}
\cup
\mathcal{O}_{\mathrm{occled}}
\cup
\mathcal{O}_{\mathrm{infl}}
\cup
\mathcal{O}_{\mathrm{capsule}}
\right).
$$

여기서 \(\mathcal{O}_{\mathrm{capsule}}\)은 q-map posture에서 reduced capsule proxy가 blocked mask와 충돌하는 cell을 의미한다.

Control Module은 cell이 왜 막혔는지 알 필요가 없다. It only needs:

- final active mask for A* planning,
- blocked mask for capsule validation and current-reference blockage detection.

이 분리는 중요하다. Camera update, target-as-obstacle policy, occlusion policy, inflation policy를 바꿔도 planner와 DLS controller의 내부 구조를 바꾸지 않아도 되기 때문이다.

---

## 7. Active-Manifold A* Planning

### 7.1 Shared Graph

두 planning branch는 같은 graph 위에서 동작한다.

$$
\mathcal{G}_{\mathrm{active}}(h,t)
=
\left(
\mathcal{F}_{\mathrm{final}}(h,t),\;
\mathcal{E}_8
\right),
$$

where \(\mathcal{E}_8\) is the 8-connected grid edge set.

A* output은 continuous trajectory가 아니라 skeleton이다.

$$
\gamma^\star
=
\{(x_k,z_k)\}_{k=0}^{N-1}.
$$

### 7.2 Distance-Only Baseline

Distance-only branch는 geometric edge length만 사용한다.

$$
c_{\mathrm{dist}}(n_i,n_j)
=
\|p(n_j)-p(n_i)\|_2.
$$

이 branch는 fallback이 아니라 canonical ablation baseline이다. Manipulability-cost branch의 효과를 보려면 반드시 동일한 downstream validator를 공유하는 distance-only branch가 필요하다.

### 7.3 Manipulability-Cost Main Branch

Manipulability-cost branch는 reduced active map의 \(\mu_{\mathrm{act}}\) 정보를 edge priority에 반영한다.

$$
c_{\mathrm{manip}}(n_i,n_j)
=
c_{\mathrm{dist}}(n_i,n_j)
\left(
1+w\phi_\kappa(n_i,n_j)
\right).
$$

현재 fixed parameter는

$$
w=0.25,
\qquad
\kappa=3.0.
$$

이 branch는 현재 main branch다. 목표는 shortest path를 크게 포기하지 않으면서, low-\(\mu_{\mathrm{act}}\) edge를 부드럽게 피하고, 그 결과 DLS execution burden을 줄이는 것이다.

### 7.4 Invalid Planning Semantics

Requested EE-safe goal이 final active mask에서 resolution tolerance 이상으로 projection되어야만 도달 가능하다면, 그것은 성공이 아니다. 그 경우 현재 base pose와 current active mask에서 requested side-contact grasp가 불가능한 것으로 본다.

이 invalid output은 code failure가 아니라 system-level decision signal이다. 이후 mobile base reposition, alternate approach, top-down grasp 같은 행동은 상위 decision layer에서 결정한다.

---

## 8. Reference Generation

A* skeleton은 grid artifact와 corner discontinuity를 가진다. 따라서 raw waypoint를 DLS servo에 직접 넣지 않는다.

Reference generation flow는 다음과 같다.

$$
\text{A* skeleton}
\rightarrow
\text{keyframe extraction}
\rightarrow
\text{local }C^2\text{ xz blending}
\rightarrow
\text{feasibility validation}
\rightarrow
\text{dense arclength table}
\rightarrow
\text{gaze pitch reconstruction}.
$$

### 8.1 Keyframe Extraction

Start와 goal은 항상 유지한다. 내부 keyframe은 \((x,z)\) grid direction change를 기준으로 추출한다.

$$
d_k
=
\operatorname{sign}(c_{k+1}-c_k).
$$

만약

$$
d_k\ne d_{k-1},
$$

이면 \(k\)는 direction-change keyframe candidate다.

Bounded line-of-sight shortcut은 shortcut segment가 같은 final active mask 내부에 있을 때만 허용한다.

### 8.2 Local \(C^2\) Corner Blending

세 keyframe \(K_{i-1},K_i,K_{i+1}\)에 대해 incoming/outgoing tangent를 둔다.

$$
t_i^-
=
\frac{K_i-K_{i-1}}{\|K_i-K_{i-1}\|_2},
\qquad
t_i^+
=
\frac{K_{i+1}-K_i}{\|K_{i+1}-K_i\|_2}.
$$

Blend endpoints는

$$
A_i=K_i-r_i t_i^-,
\qquad
B_i=K_i+r_i t_i^+
$$

로 둔다. Local blend는 \((x,z)\) quintic Hermite curve로 만들며, position, tangent, zero acceleration boundary condition을 맞춘다.

Global spline은 사용하지 않는다. 전체 waypoint를 하나의 spline으로 통과시키면 obstacle boundary나 narrow corridor에서 final mask 밖으로 overshoot할 수 있다.

### 8.3 Feasibility Validation and Fallback

Dense reference sample은 반드시 final active mask 내부에 남아야 한다.

$$
(x(s_m),z(s_m))\in\mathcal{F}_{\mathrm{final}}
\qquad
\forall m.
$$

Blend가 mask를 벗어나면 radius를 줄여 재시도한다. 그래도 실패하면 해당 corner는 \(C^0\) polyline fallback으로 남긴다. 이 fallback은 failure가 아니라 safety-first decision이며, 반드시 count로 기록한다.

### 8.4 Deterministic Gaze Pitch Reconstruction

Pitch는 독립 smoothing state가 아니다. Dense \(x,z\) sample 위에서 gaze geometry로 재계산한다.

$$
\theta_m
=
\operatorname{atan2}
\left(
x_{\mathrm{target}}-x_m,\;
h-z_m
\right).
$$

Dense reference table은 다음을 포함한다.

$$
\mathcal{R}
=
\left\{
s_m,\;
x_m,\;
z_m,\;
\theta_m,\;
\frac{dx}{ds}(s_m),\;
\frac{dz}{ds}(s_m),\;
\frac{d\theta}{ds}(s_m)
\right\}_{m=0}^{M-1}.
$$

---

## 9. Gap-Aware DLS Tracking

DLS servo는 raw time schedule을 맹목적으로 따라가지 않는다. 현재 EE 위치를 dense reference에 projection하고, projection보다 약간 앞의 arclength point를 sub-goal로 둔다.

현재 EE projection은 다음이다.

$$
s_{\mathrm{proj}}
=
\arg\min_s
\left\|
\begin{bmatrix}
x(s)\\
z(s)
\end{bmatrix}
-
\begin{bmatrix}
x_{\mathrm{EE}}\\
z_{\mathrm{EE}}
\end{bmatrix}
\right\|_2.
$$

Sub-goal은 다음이다.

$$
s_{\mathrm{target}}
=
\operatorname{clip}
\left(
s_{\mathrm{proj}}+L_{\mathrm{look}},
0,L
\right).
$$

Canonical lookahead는 약 \(10\ \mathrm{mm}\)다.

$$
L_{\mathrm{look}}\approx0.01\ \mathrm{m}.
$$

이 구조의 목적은 reference와 EE 사이의 gap을 작게 유지하면서 forward progress를 만드는 것이다. Reference가 너무 멀리 앞서가면 DLS가 path를 따라가기보다 먼 target을 잡으려는 shortcut-like behavior를 만들 수 있다. Gap-aware scheduling은 이를 제한한다.

DLS command는 다음과 같다.

$$
\dot{q}_{\mathrm{act}}
=
J_{\mathrm{act}}^T
\left(
J_{\mathrm{act}}J_{\mathrm{act}}^T+\lambda^2I
\right)^{-1}
\left(
\dot{r}_{\mathrm{ref}}+K_p e
\right).
$$

여기서

$$
e=r_{\mathrm{ref}}-r(q_{\mathrm{act}}).
$$

Damping은 \(\sigma_{\min}\)에 따라 adaptive하게 변한다.

$$
\lambda^2
=
\lambda_{\min}^2
+
\left(
\lambda_{\max}^2-\lambda_{\min}^2
\right)
\left[
\max
\left(
0,\;
1-\frac{\sigma_{\min}}{\sigma_{\mathrm{th}}}
\right)
\right]^2.
$$

Terminal hold는 path-following phase가 끝난 뒤 final residual settling을 위한 기간이다. Reference progress가 끝나지 않았는데 terminal hold로 들어가면 안 된다.

---

## 10. Capsule-Consistent Validation

Runtime arm-volume safety는 exact mesh collision으로 처리하지 않는다. 현재 phase는 fixed active plane 위의 reduced serial chain으로 닫혀 있으므로, link body는 planar capsule proxy로 근사한다.

Reduced capsule chain은 세 segment로 둔다.

1. \(q_2\) pivot에서 \(q_3\) pivot까지,
2. \(q_3\) pivot에서 \(q_5\) pivot까지,
3. \(q_5\) pivot에서 EE reference point까지.

Capsule radius는 다음으로 고정한다.

$$
r_{q_2\rightarrow q_3}=0.040\ \mathrm{m},
$$

$$
r_{q_3\rightarrow q_5}=0.041\ \mathrm{m},
$$

$$
r_{q_5\rightarrow EE}=0.033\ \mathrm{m}.
$$

Collision check는 clearance metric이 아니라 binary gate다. 기록할 값은 다음 정도면 충분하다.

- capsule collision-free 여부,
- violation sample count,
- checked sample count,
- colliding tick count.

Minimum clearance나 signed distance margin은 현재 main metric으로 쓰지 않는다. Rasterized mask, dilation, segment sampling density에 강하게 의존하기 때문에 physical clearance처럼 해석하기 어렵다.

Capsule primitive는 두 위치에서 같은 의미로 쓰인다.

1. planning 전에 q-map cell posture에 대해 capsule이 blocked mask와 충돌하는 cell을 graph에서 제거한다.
2. DLS rollout 후 active joint trajectory에 대해 같은 capsule primitive로 다시 검사한다.

이 두 단계가 같은 geometry를 공유해야 planning-control semantic mismatch가 생기지 않는다.

---

## 11. Runtime Architecture

Runtime architecture는 세 주파수 계층으로 분리한다.

- DLS servo: at least 100 Hz
- camera / map update: approximately 10-30 Hz
- A* replanning: event-triggered

Map update layer는 camera/perception 결과에서 final active mask와 blocked mask를 만든다. Control Module은 새 mask snapshot이 들어오면 그 위에서 replan할 수 있다.

Replanning trigger는 다음 경우에 발생한다.

- current reference의 남은 구간이 새 blocked mask에 걸린 경우
- goal-side corridor가 바뀐 경우
- final active mask가 크게 바뀐 경우
- 이전 candidate reference가 invalid로 판정된 경우

Replanning이 실패하면 세부 실패 원인을 외부 policy로 과도하게 나누지 않는다. 현재 active pose와 current active mask에서 side-contact grasp가 불가능하다는 의미로 반환하고, 이후 mobile base reposition, alternate approach, top-down grasp 같은 행동은 상위 decision layer의 책임으로 둔다.

현재 dynamic obstacle claim은 controlled slow cube or box-like obstacle로 제한한다. 빠르게 튀어나오는 사람 손 같은 impulsive obstacle은 별도 emergency stop, prediction, hardware safety layer가 필요하며 현재 claim에 포함하지 않는다.

---

## 12. Parameter Selection and Branch Comparison

### 12.1 Fixed Parameter Selection

Manipulability-cost branch의 fixed parameter는 다음이다.

$$
w=0.25,
\qquad
\kappa=3.0.
$$

이 값은 obstacle-free clear case 전체 height sweep에서 선택한 global fixed pair다. Obstacle 위치는 runtime에서 달라지므로 parameter tuning에 직접 넣지 않는다. Obstacle case는 stress validation으로 둔다.

Global fixed를 선택한 이유는 다음이다.

1. height table보다 해석이 단순하다.
2. target height가 변해도 하나의 operating point를 유지할 수 있다.
3. path length detour를 제한하면서 execution metric 개선을 얻을 수 있다.
4. QP weight처럼 고정된 planning parameter로 설명하기 쉽다.

### 12.2 Paired Metrics

Distance-only baseline \(b\)와 manipulability-cost candidate \(c\)가 모두 valid일 때만 ratio를 계산한다.

Path length:

$$
L_x
=
\sum_k
\|p_{k+1}-p_k\|_2.
$$

Reduced active joint motion:

$$
L_q
=
\sum_k
\|q_{\mathrm{act},k+1}-q_{\mathrm{act},k}\|_2.
$$

Joint-wise travel:

$$
L_{q_2},\qquad L_{q_3},\qquad L_{q_5}.
$$

Inefficiency:

$$
\rho
=
\frac{L_q}{L_x}.
$$

Main ratios:

$$
J_a
=
\frac{L_q^{c}}{L_q^{b}},
$$

$$
J_b
=
\frac{(L_{q,j}^{\max})^{c}}
{(L_{q,j}^{\max})^{b}},
$$

$$
J_c
=
\frac{\rho^{c}}{\rho^{b}}.
$$

Joint-wise ratios:

$$
J_{q_2}
=
\frac{L_{q_2}^{c}}{L_{q_2}^{b}},
\qquad
J_{q_3}
=
\frac{L_{q_3}^{c}}{L_{q_3}^{b}},
\qquad
J_{q_5}
=
\frac{L_{q_5}^{c}}{L_{q_5}^{b}}.
$$

Path length ratio:

$$
R_x
=
\frac{L_x^c}{L_x^b}.
$$

### 12.3 Current Evidence

전체 height clear-case comparison에서 51개 height 중 30개 height에서 distance-only와 manipulability-cost branch가 서로 다른 path를 선택했다.

Path-changing 30개 case 기준 평균 경향은 다음이다.

$$
\overline{J_a}\approx0.956,
\qquad
\overline{J_b}\approx0.952,
\qquad
\overline{J_c}\approx0.952,
$$

$$
\overline{J_{q_2}}\approx1.000,
\qquad
\overline{J_{q_3}}\approx0.889,
\qquad
\overline{J_{q_5}}\approx0.952,
$$

$$
\overline{R_x}\approx1.005.
$$

따라서 현재 evidence는 다음처럼 해석한다.

> Manipulability-cost branch는 path length를 거의 늘리지 않으면서 active joint motion과 joint-to-task inefficiency를 줄인다. 개선은 특히 \(q_3\)와 \(q_5\)에서 더 분명하고, \(q_2\)는 거의 동일하게 유지된다.

---

## 13. Metric Hierarchy and Acceptance

Planner metrics:

- path found,
- expanded node count,
- planning time,
- geometric path length \(L_x\),
- final mask collision-free flag,
- feasible coverage ratio,
- path \(\mu_{\mathrm{act}}\) statistics.

Reference metrics:

- original waypoint count,
- keyframe count,
- corner count,
- accepted blend count,
- radius shrink count,
- \(C^0\) fallback corner count,
- dense table sample count,
- reference sample feasibility,
- pitch reconstruction error,
- reference length ratio.

DLS metrics:

$$
L_q,\qquad
L_{q_2},\qquad
L_{q_3},\qquad
L_{q_5},\qquad
\rho,\qquad
e_{\mathrm{final}}.
$$

Additional diagnostics:

- tracking RMS and max,
- gap RMS and max,
- peak active joint velocity,
- \(\sigma_{\min}\) minimum,
- damping activation ratio,
- non-finite failure flag,
- joint-limit saturation flag,
- terminal settling status.

Basic acceptance requires:

1. path exists,
2. requested goal projection is within tolerance,
3. path and dense reference samples are feasible,
4. capsule proxy validation passes,
5. DLS rollout metrics are finite,
6. final residual tolerance passes,
7. hard numerical failure does not occur.

Invalid case is not a code failure. It means the requested side-contact grasp is not feasible under the current active mask and current base pose.

---

## 14. Experimental Strategy

### 14.1 Perception Experiments

Measure:

- image encoder latency,
- text cache behavior,
- segmentation latency,
- total perception freshness,
- throughput under live camera input.

The goal is not merely standalone FPS, but stable input quality for Decision and Map Update.

### 14.2 Decision Experiments

Measure:

- target pose stability,
- yaw degeneracy handling,
- width/depth consistency,
- frame-contract robustness,
- side-contact feasibility,
- target-contact and EE-safe goal separation.

### 14.3 Map Generation Experiments

Measure:

- reduced active map feasible coverage,
- \(\mu_{\mathrm{act}}\) distribution,
- \(\sigma_{\min}\) distribution,
- q-map continuity,
- capsule-pruned cell ratio,
- map generation time.

### 14.4 Planning and Control Experiments

Compare:

1. distance-only baseline,
2. manipulability-cost main branch,
3. linear reference versus local \(C^2\) reference,
4. clear case across all height layers,
5. controlled obstacle / changed-map stress cases,
6. invalid case handling when side-contact corridor is blocked.

Main result table should report:

$$
L_x,\quad
L_q,\quad
L_{q_2},\quad
L_{q_3},\quad
L_{q_5},\quad
\rho,\quad
J_a,\quad
J_b,\quad
J_c,\quad
J_{q_2},\quad
J_{q_3},\quad
J_{q_5},\quad
T_{\mathrm{plan}}.
$$

### 14.5 Integrated Validation

Integrated validation should test whether the full pipeline remains closed on the same reduced active manifold.

Measure:

- final residual,
- tracking error,
- gap statistics,
- DLS damping activation,
- peak joint velocity,
- completion status,
- gaze consistency,
- capsule validation result,
- invalid return correctness.

---

## 15. Claim Boundary

The current method does not claim:

- exact runtime mesh collision guarantee,
- physical clearance guarantee,
- energy optimality,
- torque optimality,
- time optimality,
- dynamic optimality,
- guaranteed closed-loop stability of the full hardware system,
- fast human-hand obstacle safety,
- success for every obstacle configuration.

The current method claims:

> L-OMM constructs a reduced active manifold for the side-contact / gaze-constrained phase, updates a final active mask from perception-derived obstacle semantics, selects an execution-aware skeleton with manipulability-cost A*, converts the skeleton into a feasibility-validated local \(C^2\) reference with deterministic gaze pitch, and validates execution through gap-aware reduced DLS tracking and capsule-consistent collision checking. The distance-only branch is retained as a paired ablation baseline.

---

## 16. Current Canonical Method Statement

The current canonical path-control method is defined by the following invariant.

$$
q_{\mathrm{act}}=(q_2,q_3,q_5),
\qquad
y_{\mathrm{plane}}=0.047\ \mathrm{m},
\qquad
x_{\mathrm{EE}}=0.40\ \mathrm{m},
\qquad
x_{\mathrm{target}}=0.53\ \mathrm{m}.
$$

Final data flow:

$$
\text{perception / decision}
\rightarrow
\text{map update layer}
\rightarrow
\text{final active mask}
\rightarrow
\text{manipulability-cost A* skeleton}
\rightarrow
\text{local }C^2\text{ reference}
\rightarrow
\text{deterministic gaze pitch}
\rightarrow
\text{gap-aware reduced DLS}
\rightarrow
\text{capsule validation}
\rightarrow
\text{accepted or ungraspable}.
$$

Distance-only A* is retained as the same pipeline with only the edge cost changed to geometric distance.

This separation is essential. It prevents the paper from looking like a hand-tuned demonstration and makes the method reproducible: a different robot, map resolution, or workspace can regenerate its reduced active map, define its target-contact offset, select or tune its manipulability-cost parameters, and validate its own reference generation and DLS execution behavior.
