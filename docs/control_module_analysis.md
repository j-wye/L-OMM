# Control Module Reference Spec — Reduced Active-Manifold Planning and DLS Tracking

## 1. Purpose

이 문서는 현재 Control Module의 기준 설계를 정의한다. 목표는 임의의 전신 로봇 운동을 모두 해결하는 것이 아니라, side-contact / gaze-constrained manipulation phase에서 필요한 active manifold를 명확히 정의하고, 그 위에서 path planning, reference generation, DLS tracking, capsule-consistent validation이 같은 기하적 의미를 공유하도록 만드는 것이다.

Control Module의 책임은 다음과 같다.

- precomputed reduced active map과 runtime final active mask를 입력으로 받아 feasible path skeleton을 생성한다.
- 생성된 skeleton을 raw waypoint trajectory로 쓰지 않고, control-friendly arclength reference로 변환한다.
- dense reference 위에서 gaze pitch를 deterministic하게 재구성한다.
- reduced active joint space에서 DLS servo simulation 또는 runtime servo command를 수행한다.
- generated reference가 tracking 가능한지, 그리고 capsule proxy 기준으로 실행 가능한지를 판정한다.

Control Module의 책임이 아닌 것은 다음과 같다.

- camera image에서 object mask를 직접 생성하는 것
- target pose를 perception/decision 없이 직접 추정하는 것
- occupied / unknown / occluded / inflation layer를 직접 해석하는 것
- physical torque, contact dynamics, full-body dynamic optimality를 보장하는 것
- every tick마다 global planning을 수행하는 것

따라서 Control Module은 map을 “만드는” 모듈이 아니라, 이미 만들어진 active planning mask 위에서 path, reference, reduced DLS execution을 생성하고 검증하는 모듈이다.

---

## 2. Task Phase Definition

현재 phase는 target-side side-contact와 gaze constraint를 동시에 만족해야 하는 접근 구간이다. 이 phase에서 사용하는 active joint vector는 다음과 같다.

$$
q_{\mathrm{act}}
=
\begin{bmatrix}
q_2\\
q_3\\
q_5
\end{bmatrix}.
$$

고정 posture는 다음과 같다.

$$
q_1=0,\qquad
q_4=\frac{\pi}{2},\qquad
q_6=-\frac{\pi}{2}.
$$

여기서 \(q_1\)은 active map 내부의 planning variable이 아니라, mobile base 또는 상위 yaw alignment layer에서 다룰 값이다. \(q_4\)와 \(q_6\)은 현재 phase에서 wrist/tool frame convention을 고정하기 위한 posture parameter다.

Reduced task vector는 다음과 같다.

$$
r
=
\begin{bmatrix}
x\\
z\\
\theta_{\mathrm{gaze}}
\end{bmatrix}.
$$

Active plane은 다음으로 고정한다.

$$
y_{\mathrm{plane}}=0.047\ \mathrm{m}.
$$

이 값은 runtime tuning knob가 아니라, 현재 reduced active posture가 만드는 task plane이다. 따라서 map generation, path planning, reference generation, DLS tracking, capsule validation은 모두 같은 \(y_{\mathrm{plane}}\) 위에서 정의되어야 한다.

---

## 3. Target Contact and EE-Safe Goal

Target object는 runtime obstacle로 취급한다. 이것은 target에 접근하지 않는다는 뜻이 아니라, robot body가 target volume 안으로 들어가면 안 된다는 뜻이다.

Tool contact point와 EE-safe goal은 분리한다.

$$
x_{\mathrm{target}}=0.53\ \mathrm{m},
$$

$$
x_{\mathrm{EE}}=0.40\ \mathrm{m},
$$

$$
x_{\mathrm{target}}-x_{\mathrm{EE}}=0.13\ \mathrm{m}.
$$

Path planning goal은 target point가 아니라 EE-side safe pose다.

$$
p_{\mathrm{goal}}
=
\begin{bmatrix}
x_{\mathrm{EE}}\\
h
\end{bmatrix}.
$$

반면 gaze pitch는 target contact point를 향하도록 계산한다.

$$
\theta_{\mathrm{gaze}}(x,z;h)
=
\operatorname{atan2}
\left(
x_{\mathrm{target}}-x,\;
h-z
\right).
$$

이 분리는 현재 task의 핵심이다. EE, wrist, link body는 target volume 밖에 있어야 하지만, tool contact geometry는 target surface를 향해야 한다.

---

## 4. Reduced Active Map Contract

Reduced active map은 target height layer \(h\)와 fixed active plane \(y_{\mathrm{plane}}\)에 대해 정의된 \((x,z)\) grid다. 각 cell은 EE reference point의 후보 위치를 의미한다.

각 node는 다음과 같이 쓴다.

$$
n=(i,j),
\qquad
p(n)=(x_i,z_j).
$$

각 node의 reduced task reference는 다음과 같다.

$$
r(n;h)
=
\begin{bmatrix}
x_i\\
z_j\\
\theta_{\mathrm{gaze}}(x_i,z_j;h)
\end{bmatrix}.
$$

Map channel은 다음 의미를 가진다.

$$
\mathcal{M}(x,z,0)=\mu_{\mathrm{act}}(x,z;h),
$$

$$
\mathcal{M}(x,z,1)=\theta_{\mathrm{gaze}}(x,z;h),
$$

$$
\mathcal{M}(x,z,2)=\sigma_{\min}(x,z;h).
$$

Sidecar q-map은 각 valid cell에 대한 reduced active joint solution을 저장한다.

$$
q_{\mathrm{act}}^\star(x,z;h)
=
\begin{bmatrix}
q_2^\star\\
q_3^\star\\
q_5^\star
\end{bmatrix}.
$$

Reduced Jacobian은 다음과 같다.

$$
J_{\mathrm{act}}(q_{\mathrm{act}})
=
\frac{\partial(x,z,\theta_{\mathrm{gaze}})}
{\partial(q_2,q_3,q_5)}
\in\mathbb{R}^{3\times3}.
$$

Manipulability는 다음으로 정의한다.

$$
\mu_{\mathrm{act}}
=
\left|\det J_{\mathrm{act}}\right|.
$$

이 값은 두 가지 방식으로 사용된다.

첫째, \(\mu_{\mathrm{act}}\)는 base feasibility를 만드는 hard pruning 기준이다. 매우 낮은 manipulability cell은 risk region으로 보고 graph에서 제거할 수 있다.

둘째, 현재 main branch에서는 \(\mu_{\mathrm{act}}\)를 A* edge priority에 반영한다. 즉, reduced active map은 단순 reachability map이 아니라, low-manipulability corridor를 부드럽게 회피할 수 있는 execution-aware graph signal도 제공한다.

---

## 5. Runtime Map Input Contract

Control Module이 runtime에서 직접 받아야 하는 것은 semantic obstacle의 원본 layer들이 아니다. Camera/perception/decision 쪽에서 다음 해석을 먼저 끝내야 한다.

- 화면에 들어온 object를 occupied로 둘 것
- target object도 robot body 기준 obstacle로 둘 것
- occluded/unknown region을 conservative blocked region으로 둘 것
- safety inflation을 적용할 것
- 필요하면 obstacle motion에 따라 mask를 camera rate에서 갱신할 것

이 결과 Control Module에 전달되어야 하는 것은 다음 두 binary mask다.

1. **Final active mask**

   A*가 실제로 사용할 planning mask다.

   $$
   \mathcal{F}_{\mathrm{final}}
   \subseteq
   \mathcal{G}_h.
   $$

2. **Blocked mask**

   Capsule validation과 current-reference blockage detection에 사용하는 binary forbidden mask다.

Control Module은 cell이 왜 막혔는지 알 필요가 없다. 즉, occupied / target / unknown / occluded / inflation은 Control Module 외부의 Map Update Layer 책임이다.

이 분리가 중요한 이유는 다음과 같다.

- Control Module의 API가 단순해진다.
- perception semantics가 planner 내부로 침투하지 않는다.
- 나중에 camera update, occlusion policy, target-as-obstacle policy를 바꿔도 DLS/controller 코드를 고치지 않아도 된다.
- runtime replanning은 “새 final mask가 들어왔다”는 event로 표현할 수 있다.

---

## 6. Active-Manifold A* Branches

현재 path planning은 두 branch를 유지한다.

1. **Manipulability-cost branch**
2. **Distance-only branch**

Main branch는 manipulability-cost branch다. Distance-only branch는 삭제하지 않고 ablation baseline으로 유지한다.

두 branch는 반드시 같은 reduced active map, 같은 final mask, 같은 start/goal, 같은 reference generator, 같은 DLS validator, 같은 capsule validation을 공유해야 한다. 차이는 A* edge cost뿐이어야 한다.

### 6.1 Distance-Only Branch

Distance-only branch의 edge cost는 geometric distance다.

$$
c_{\mathrm{dist}}(n_i,n_j)
=
\|p(n_j)-p(n_i)\|_2.
$$

이 branch의 의미는 다음과 같다.

> 같은 feasible active graph 위에서 가장 짧은 geometric skeleton을 선택하는 canonical baseline.

Distance-only branch는 fallback이 아니라 denominator다. Manipulability-cost branch가 실제로 execution metric을 개선하는지 판단하려면 반드시 유지되어야 한다.

### 6.2 Manipulability-Cost Branch

Manipulability-cost branch는 같은 feasible graph 위에서 low-\(\mu_{\mathrm{act}}\) edge를 부드럽게 불리하게 만든다.

인접 node \(n_i,n_j\)의 edge manipulability representative는 harmonic mean으로 둔다.

$$
\mu_e(n_i,n_j)
=
\frac{2\mu_i\mu_j}{\mu_i+\mu_j}.
$$

Feasible edge에서 \(\mu_i+\mu_j>0\)이므로 불필요한 epsilon은 넣지 않는다.

Safe manipulability level은 feasible map 내부 percentile로 둔다.

$$
\mu_{\mathrm{safe}}
=
\operatorname{percentile}
\left(
\{\mu_{\mathrm{act}}(n)\mid n\in\mathcal{F}_{\mathrm{base}}\},
25\%
\right).
$$

Manipulability deficit은 다음과 같다.

$$
\delta_\mu
=
\max
\left(
0,\;
\frac{\mu_{\mathrm{safe}}-\mu_e}
{\mu_{\mathrm{safe}}-\mu_{\min}}
\right).
$$

Bounded penalty는 다음과 같다.

$$
\phi_\kappa
=
\tanh(\kappa\delta_\mu).
$$

Candidate edge cost는 다음과 같다.

$$
c_{\mathrm{manip}}(n_i,n_j)
=
c_{\mathrm{dist}}(n_i,n_j)
\left(
1+w\phi_\kappa(n_i,n_j)
\right).
$$

현재 fixed operating point는 다음과 같다.

$$
w=0.25,
\qquad
\kappa=3.0.
$$

이 값은 obstacle-free clear case 전체 height sweep에서 선택한 global fixed pair다. Obstacle layout은 runtime에서 변하므로 tuning target에 직접 넣지 않는다. Obstacle case는 parameter tuning이 아니라 stress validation과 invalid policy 검증에 사용한다.

---

## 7. A* Output Is a Skeleton

A* output은 trajectory가 아니라 feasible corridor skeleton이다.

$$
\gamma^\star
=
\{(x_k,z_k)\}_{k=0}^{N-1}.
$$

Raw A* waypoint는 grid artifact와 corner discontinuity를 가진다. 따라서 DLS servo가 이를 직접 따라가면 feedforward velocity가 불연속이 되고, joint velocity command도 불필요하게 튈 수 있다.

Control Module은 skeleton을 다음 절차로 reference화한다.

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

Global spline은 사용하지 않는다. 전체 waypoint를 하나의 spline으로 통과시키면 obstacle boundary나 narrow corridor에서 active mask 밖으로 overshoot할 수 있다. Smoothing은 local corner에서만 수행하고, dense sample이 final active mask 안에 남는 경우에만 accept한다.

---

## 8. \(C^2\) Reference Generation

Reference generation은 \(x,z\) geometry에 대해서만 smoothing을 수행한다. Pitch는 독립 smoothing state가 아니다.

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

Bounded line-of-sight shortcut은 shortcut segment가 같은 final active mask 내부에 남을 때만 허용한다.

### 8.2 Local Corner Blending

연속 keyframe \(K_{i-1},K_i,K_{i+1}\)에 대해 incoming/outgoing tangent를 둔다.

$$
t_i^-
=
\frac{K_i-K_{i-1}}{\|K_i-K_{i-1}\|_2},
\qquad
t_i^+
=
\frac{K_{i+1}-K_i}{\|K_{i+1}-K_i\|_2}.
$$

Blend endpoints는 다음과 같다.

$$
A_i=K_i-r_i t_i^-,
\qquad
B_i=K_i+r_i t_i^+.
$$

Local blend는 \((x,z)\) quintic Hermite curve로 만든다. Boundary 조건은 position, tangent, zero acceleration을 맞춘다. Accept된 blend corner는 local \(C^2\) continuity를 가진다.

### 8.3 Feasibility Validation and Fallback

Dense reference sample은 반드시 final active mask 내부에 남아야 한다.

$$
(x(s_m),z(s_m))\in\mathcal{F}_{\mathrm{final}}
\qquad
\forall m.
$$

Blend가 mask를 벗어나면 radius를 줄여 재시도한다. 그래도 실패하면 해당 corner는 \(C^0\) polyline fallback으로 남긴다. 이 fallback은 실패가 아니라 safety-first decision이며, 반드시 fallback count로 기록한다.

### 8.4 Pitch Reconstruction

Dense \(x,z\) sample이 만들어진 뒤 pitch를 다시 계산한다.

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

Runtime evaluation은 \(s\) 기준 interpolation으로 수행한다.

---

## 9. Gap-Aware Reference Scheduling

Reference scheduling은 DLS tick마다 어떤 arclength point를 sub-goal로 줄지 결정한다.

현재 EE position을 reference path 위에 projection한다.

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

Sub-goal은 projection보다 약간 앞에 둔다.

$$
s_{\mathrm{target}}
=
\operatorname{clip}
\left(
s_{\mathrm{proj}}+L_{\mathrm{look}},
\;0,\;L
\right).
$$

Canonical lookahead는 약 \(10\ \mathrm{mm}\)다.

$$
L_{\mathrm{look}}\approx0.01\ \mathrm{m}.
$$

Scheduler는 reference를 순간적으로 멀리 보내지 않는다. Arclength progress는 rate-limited되고, EE가 damping region이나 corner에서 느려지면 reference도 실제 EE projection에 묶여 같이 느려진다.

Terminal hold는 path following이 끝난 뒤 final residual settling을 위한 구간이다. Reference progress가 끝나지 않았는데 terminal hold로 들어가면 안 된다.

---

## 10. Reduced DLS Servo Contract

DLS servo의 입력은 다음과 같다.

$$
q_{\mathrm{act}},
\qquad
r_{\mathrm{ref}}
=
\begin{bmatrix}
x_{\mathrm{ref}}\\
z_{\mathrm{ref}}\\
\theta_{\mathrm{ref}}
\end{bmatrix},
\qquad
\dot{r}_{\mathrm{ref}},
\qquad
\Delta t.
$$

Task error는 다음과 같다.

$$
e
=
r_{\mathrm{ref}}-r(q_{\mathrm{act}}).
$$

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

\(\sigma_{\min}\)은 \(J_{\mathrm{act}}\)의 smallest singular value다. Damping schedule은 다음과 같다.

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

Servo는 joint velocity limit, joint position limit, non-finite guard를 적용한다. Singularity exposure는 failure로 숨기지 않고 \(\sigma_{\min}\), damping activation ratio, hard-breach count로 기록한다.

---

## 11. Capsule-Consistent Collision Semantics

Runtime collision validation은 exact STL mesh collision이 아니라 reduced active-plane capsule proxy를 사용한다.

Capsule은 line segment와 disk의 Minkowski sum이다. \((x,z)\) plane에서 보면 달리기 트랙처럼 생긴 swept segment다.

Reduced capsule chain은 세 segment로 둔다.

1. \(q_2\) pivot에서 \(q_3\) pivot까지
2. \(q_3\) pivot에서 \(q_5\) pivot까지
3. \(q_5\) pivot에서 EE reference point까지

Final capsule radii는 다음과 같다.

$$
r_{q_2\rightarrow q_3}=0.040\ \mathrm{m},
$$

$$
r_{q_3\rightarrow q_5}=0.041\ \mathrm{m},
$$

$$
r_{q_5\rightarrow EE}=0.033\ \mathrm{m}.
$$

Collision check는 clearance metric이 아니라 binary gate다.

기록할 값은 다음 정도면 충분하다.

- capsule collision-free 여부
- violation sample count
- checked sample count
- colliding tick count

Minimum clearance나 signed distance margin은 현재 metric으로 쓰지 않는다. Rasterized mask, dilation, segment sampling density에 강하게 의존하기 때문에 physical clearance처럼 해석하기 어렵다.

Capsule primitive는 두 위치에서 같은 의미로 쓰여야 한다.

첫째, planning 전에 q-map cell posture에 대해 capsule이 blocked mask와 충돌하는 cell을 graph에서 제거한다.

둘째, DLS rollout 후 실제 active joint trajectory에 대해 같은 capsule primitive로 validation한다.

이 두 단계가 같은 geometry를 공유해야 planning-control semantic mismatch가 생기지 않는다.

---

## 12. Runtime Obstacle Update and Replanning

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

Replanning이 실패하면 Control Module은 세부 원인을 외부 policy로 과도하게 나누지 않는다. 현재 active pose와 현재 active mask에서 side-contact grasp가 불가능하다는 의미로 `UNGRASPABLE`에 해당하는 결과를 반환하면 된다. 이후 mobile base reposition, 다른 approach mode, top-down grasp 같은 결정은 상위 module의 책임이다.

현재 dynamic obstacle claim은 controlled slow cube or box-like obstacle로 제한한다. 빠르게 튀어나오는 사람 손 같은 impulsive obstacle은 별도 emergency stop과 hardware safety layer가 필요하며 현재 claim에 넣지 않는다.

---

## 13. Metrics and Acceptance

Planner metrics는 다음을 포함한다.

- path found
- expanded node count
- planning time
- geometric path length \(L_x\)
- final mask collision-free flag
- feasible coverage ratio
- path \(\mu_{\mathrm{act}}\) statistics

Reference metrics는 다음을 포함한다.

- original waypoint count
- keyframe count
- corner count
- accepted blend count
- radius shrink count
- \(C^0\) fallback corner count
- dense table sample count
- reference sample feasibility
- pitch reconstruction error
- reference length ratio

DLS metrics는 다음을 포함한다.

$$
L_q
=
\sum_k
\|q_{\mathrm{act},k+1}-q_{\mathrm{act},k}\|_2,
$$

$$
L_{q_j}
=
\sum_k
\left|
q_{j,k+1}-q_{j,k}
\right|,
\qquad
j\in\{2,3,5\},
$$

$$
\rho
=
\frac{L_q}{L_x}.
$$

Joint-wise travel은 반드시 따로 기록한다.

$$
L_{q_2},\qquad L_{q_3},\qquad L_{q_5}.
$$

Final residual은 reduced task 기준으로 판단한다.

$$
e_{\mathrm{final}}
=
\left\|
r_{\mathrm{ref}}(L)-r(q_{\mathrm{act},f})
\right\|_2.
$$

Basic acceptance는 다음 조건을 요구한다.

- path exists
- requested goal projection이 tolerance 안에 있음
- path and dense reference samples are feasible
- capsule proxy validation 통과
- DLS rollout metric finite
- final residual tolerance 통과
- hard numerical failure 없음

Invalid case는 code failure가 아니다. 현재 active mask와 current base pose에서 requested side-contact grasp가 불가능하다는 결과다.

---

## 14. Distance vs Manipulability Comparison Protocol

Distance-only와 manipulability-cost branch는 paired comparison으로만 비교한다.

Baseline은 distance-only branch다. Candidate는 manipulability-cost branch다.

두 branch가 모두 valid일 때만 ratio를 계산한다.

$$
J_a
=
\frac{L_q^{\mathrm{cand}}}{L_q^{\mathrm{dist}}},
$$

$$
J_b
=
\frac{(L_{q,j}^{\max})^{\mathrm{cand}}}
{(L_{q,j}^{\max})^{\mathrm{dist}}},
$$

$$
J_c
=
\frac{\rho^{\mathrm{cand}}}{\rho^{\mathrm{dist}}}.
$$

Joint-wise ratios도 함께 본다.

$$
J_{q_2}
=
\frac{L_{q_2}^{\mathrm{cand}}}{L_{q_2}^{\mathrm{dist}}},
\qquad
J_{q_3}
=
\frac{L_{q_3}^{\mathrm{cand}}}{L_{q_3}^{\mathrm{dist}}},
\qquad
J_{q_5}
=
\frac{L_{q_5}^{\mathrm{cand}}}{L_{q_5}^{\mathrm{dist}}}.
$$

Path length ratio도 guard로 본다.

$$
R_x
=
\frac{L_x^{\mathrm{cand}}}{L_x^{\mathrm{dist}}}.
$$

Improvement라고 말하려면 다음 구조가 필요하다.

- \(R_x\)는 거의 1에 가까워야 한다.
- \(J_a<1\)이면 total active joint motion이 줄었다.
- \(J_b<1\)이면 worst joint burden이 줄었다.
- \(J_c<1\)이면 joint-to-task inefficiency가 줄었다.
- \(J_{q_2},J_{q_3},J_{q_5}\) 중 특정 joint가 과도하게 악화되지 않아야 한다.

경로가 완전히 같은 height는 branch 선택 효과가 없으므로, main ratio 분석에서는 path-changing case만 따로 보는 것이 맞다.

---

## 15. Current Evidence Summary

현재 fixed parameter는 다음과 같다.

$$
w=0.25,
\qquad
\kappa=3.0.
$$

이 값으로 전체 height clear-case comparison을 수행했을 때, 전체 51개 height 중 30개 height에서 distance-only와 manipulability-cost branch가 서로 다른 path를 선택했다.

Path-changing 30개 case 기준으로 관찰된 평균 경향은 다음과 같다.

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

해석은 명확하다. Manipulability-cost branch는 path length를 거의 늘리지 않으면서 active joint motion과 joint-to-task inefficiency를 줄인다. 특히 q3와 q5에서 개선이 더 분명하고, q2는 거의 동일하게 유지된다.

따라서 현재 branch 구성은 다음처럼 정리한다.

- Manipulability-cost branch: current main branch
- Distance-only branch: canonical ablation baseline

---

## 16. Final Data Flow

최종 data flow는 다음과 같다.

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
\text{gap-aware scheduling}
\rightarrow
\text{reduced DLS servo}
\rightarrow
\text{capsule validation}
\rightarrow
\text{accepted or ungraspable}.
$$

Distance-only branch는 같은 flow에서 A* edge cost만 geometric distance로 바꾼 ablation branch다.

이 pipeline의 핵심 invariant는 다음과 같다.

$$
q_{\mathrm{act}}=(q_2,q_3,q_5),
\qquad
y_{\mathrm{plane}}=0.047\ \mathrm{m},
\qquad
x_{\mathrm{EE}}=0.40\ \mathrm{m},
\qquad
x_{\mathrm{target}}=0.53\ \mathrm{m}.
$$

이 invariant가 깨지면 map, planner, reference, DLS, capsule validation이 서로 다른 문제를 풀게 된다. 따라서 모든 코드와 실험은 이 invariant를 유지해야 한다.

---

## 17. Explicit Non-Claims

현재 구조가 주장하지 않는 것은 다음과 같다.

- exact runtime mesh collision guarantee
- physical clearance guarantee
- torque optimality
- dynamic optimality
- time optimality
- fast human-hand obstacle safety
- 모든 obstacle configuration에서 side-contact success
- hardware closed-loop stability guarantee

현재 구조가 주장하는 것은 다음이다.

> Reduced active map 위에서 final active mask를 구성하고, manipulability-cost A*로 execution-aware skeleton을 선택한 뒤, local \(C^2\) reference, deterministic gaze pitch, gap-aware DLS tracking, capsule-consistent validation을 통해 current side-contact phase의 실행 가능성을 판정한다. Distance-only branch는 동일 pipeline에서 edge cost만 제거한 ablation baseline으로 유지한다.

