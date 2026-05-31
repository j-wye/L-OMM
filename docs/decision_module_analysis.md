# Decision Module Analysis

## Executive Summary

현재 `decision_module.py`는 learned grasp regressor가 아니라, Perception Module이 제공한 ROI-ready GPU tensor를 입력으로 받아 **중력 정렬 side-grasp pose**를 결정론적으로 추정하는 geometric state observer이다. 핵심 설계는 다음 세 가지로 요약된다.

1. **Raw geometry는 optical frame에서 계산하고, 시간 상태와 최종 target pose는 map frame에서 유지한다.**  
   RGB-D crop, mask, ray base는 camera optical frame의 metric point set으로 lifting되고, 관측된 중심과 축은 `T_map_optical`을 통해 map frame으로 이동한다. 이후 EMA, yaw filtering, stale/hold policy는 모두 map frame 상태를 기준으로 수행된다.

2. **문제의 자유도를 full SE(3)에서 중력 정렬 \(SE(2)\)-like grasp manifold로 축소한다.**  
   target pose는
   \[
   T^{m}_{t} =
   \begin{bmatrix}
   R^{m}_{t}(\theta) & p^{m}_{t} \\
   0 & 1
   \end{bmatrix},
   \qquad
   R^{m}_{t}(\theta)=
   \begin{bmatrix}
   x(\theta) & y(\theta) & g_m
   \end{bmatrix},
   \]
   로 제한된다. 여기서 \(g_m=[0,0,1]^\top\)이고, yaw \(\theta\in S^1\)만 orientation의 동적 상태로 남는다. 이 제한은 mobile manipulator/edge setting에서 가장 불안정한 roll-pitch 추정을 제거하고, side-grasp에 필요한 접근 방향만 안정적으로 추정하게 만든다.

3. **단일 프레임의 noisy geometry를 직접 publish하지 않고, gate가 포함된 상태 추정기로 변환한다.**  
   매 프레임에서 object center, horizontal axes, physical extent를 다시 측정하되, center jump, extent jump, depth range, gripper feasibility, axis quality, degeneracy hysteresis를 통과한 정보만 상태에 반영한다. 실패한 프레임에서는 이전 유효 pose를 held/stale output으로 유지하여 downstream controller가 measurement validity와 output continuity를 분리해서 판단할 수 있게 한다.

이 구조의 학술적 의미는 단순한 post-processing이 아니라, segmentation 기반 perception의 불확실성을 **저차원 geometric observer**로 투영한다는 점에 있다. Perception이 dense mask를 제공하고 Decision이 grasp pose를 제공하는 전형적 pipeline에서 가장 흔한 failure는 mask boundary noise, depth hole, object symmetry, one-frame jump가 직접 robot command로 전달되는 것이다. 현재 Decision Module은 이 문제를 \(S^1\) yaw observer, physical gate, stale-aware output contract로 차단한다.

---

## 1. Module Contract

Decision Module은 full-frame perception을 다시 수행하지 않는다. 입력은 이미 upstream pipeline에서 crop/slice된 ROI-ready tensor이며, 모든 핵심 연산은 동일 GPU device 상에서 수행되는 것을 전제로 한다.

### 1.1 Inputs

입력 contract는 다음과 같다.

| 입력 | 의미 | 수학적 역할 |
|---|---|---|
| `roi_rgb` | ROI RGB crop | guided depth filtering의 guide image |
| `roi_depth` | ROI depth crop, meter 단위 | optical-frame point lifting의 depth scalar \(z(u,v)\) |
| `roi_mask` | ROI object mask | point selection indicator \(M(u,v)\) |
| `roi_x_base`, `roi_y_base` | ROI ray base | \(x=(u-c_x)/f_x\), \(y=(v-c_y)/f_y\) 형태의 precomputed ray slope |
| `T_map_optical` | camera optical frame에서 map frame으로 가는 transform | raw optical geometry를 map-frame state로 lifting |

Decision Module이 기대하는 point lifting은
\[
p_o(u,v)=
\begin{bmatrix}
x_{\text{base}}(u,v)z(u,v)\\
y_{\text{base}}(u,v)z(u,v)\\
z(u,v)
\end{bmatrix},
\qquad
M(u,v)>0.5.
\]
여기서 \(o\)는 camera optical frame, \(m\)은 map frame이다.

`T_map_optical`은 단순 부가 정보가 아니라, 이 모듈의 핵심 contract이다. raw point cloud의 PCA와 depth statistics는 optical frame에서 계산되지만, 상태 추정기의 중심 \(p_m\), yaw \(\theta_m\), 최종 target transform \(T^m_t\)는 map frame에서 유지된다.

### 1.2 Outputs

출력은 하나의 dataclass가 아니라 dictionary contract이다. 핵심 필드는 다음과 같이 해석해야 한다.

| 출력 | 의미 |
|---|---|
| `T_map_target` | downstream motion planner/controller가 사용해야 하는 최종 target pose |
| `T_optical_target_raw` | 현재 프레임 raw geometry 진단용 optical-frame pose |
| `p_map`, `approach_map` | map-frame target center 및 approach axis |
| `feasible`, `hard_feasible`, `safe_feasible` | gripper stroke 및 margin 기준 grasp feasibility |
| `measurement_valid` | 현재 프레임 측정이 state update에 충분했는지 |
| `output_valid` | publish 가능한 target output이 존재하는지 |
| `stale` | 현재 출력이 fresh measurement가 아니라 cached state인지 |
| `status`, `reason` | fresh/held/rejected/reset 및 rejection 원인 |

Downstream에서는 `output_valid`와 `stale`을 분리해서 해석해야 한다. 예를 들어 `output_valid=True`, `stale=True`는 controller가 즉시 폐기해야 하는 NaN output이 아니라, “현재 프레임 측정은 거부되었지만 이전 valid state가 유지되고 있다”는 뜻이다. 정밀 조작에서는 이를 hold command, velocity attenuation, replanning trigger로 분기할 수 있다.

---

## 2. Problem Formulation

Decision Module의 목표는 RGB-D segmentation 결과
\[
\mathcal{I}_t=\{I_t^c, D_t, M_t, T^m_{o,t}\}
\]
로부터 side-grasp target pose
\[
\hat{T}^{m}_{t} \in SE(3)
\]
를 추정하는 것이다. 그러나 이 문제를 full \(SE(3)\) pose estimation으로 두면, mask boundary noise와 object symmetry 때문에 orientation이 불필요하게 ill-posed해진다. 현재 구현은 grasp task에 필요한 자유도만 남기기 위해 다음과 같은 constrained manifold를 사용한다.

\[
\mathcal{M}_{\text{grasp}}
=
\left\{
(p_m,\theta)
\mid
p_m\in\mathbb{R}^3,\;
\theta\in S^1,\;
R^m_t(\theta)=
\begin{bmatrix}
\cos\theta & -\sin\theta & 0\\
\sin\theta & \cos\theta & 0\\
0 & 0 & 1
\end{bmatrix}
\right\}.
\]

즉 Decision Module은 “물체의 완전한 6D pose”를 추정하지 않는다. 대신 manipulation에 필요한 **approach direction, closing direction, target point, gripper feasibility**를 안정적으로 추정한다. 이 점은 연구적으로 중요하다. segmentation mask로부터 얻는 partial point cloud는 보통 object canonical frame을 식별할 만큼 충분하지 않지만, tabletop side-grasp의 approach axis와 physical extent를 추정하기에는 충분하다. 따라서 추정 문제를 task-relevant variable로 축소하는 것이 noise robustness와 real-time latency 양쪽에서 더 합리적이다.

---

## 3. Geometry Pipeline

### 3.1 Guided Depth Regularization

Depth crop은 RGB crop을 guide로 하는 local guided filtering을 거친다. 설계 목적은 depth hole과 작은 discontinuity가 곧바로 point cloud outlier로 증폭되는 것을 줄이는 것이다.

이 단계는 object geometry를 새로 생성하는 semantic completion이 아니라, 동일 ROI 안에서 RGB edge와 depth consistency를 이용해 local smoothing을 수행하는 low-level regularization이다. 중요한 점은 depth가 없는 영역을 무조건 확장하지 않고, valid depth와 guide statistics의 local relationship을 통해 보정한다는 것이다.

Edge device 관점에서 이 연산은 convolutional average pooling 기반이므로 복잡도는 ROI pixel 수 \(P=H_{\text{roi}}W_{\text{roi}}\)에 대해
\[
\mathcal{O}(P k^2)
\]
로 볼 수 있다. 실제 구현에서는 pooling operator가 CUDA kernel로 수행되므로 Python loop 기반 filtering보다 훨씬 안정적인 latency를 가진다.

### 3.2 Mask-Guided Backprojection

Guided depth \(D_g\)와 mask \(M\)으로부터 optical-frame point set을 만든다.

\[
\mathcal{P}_o
=
\left\{
\begin{bmatrix}
x_b(u,v)D_g(u,v)\\
y_b(u,v)D_g(u,v)\\
D_g(u,v)
\end{bmatrix}
\;\middle|\;
M(u,v)>0.5,\;D_g(u,v)>0
\right\}.
\]

이후 depth median \(\tilde{z}\)를 기준으로
\[
|z_i-\tilde{z}|\le \delta_z
\]
인 점만 유지한다. 이 depth-band gate는 segmentation mask가 주변 물체나 배경으로 새는 상황에서 가장 단순하면서 효과적인 3D consistency test이다.

다만 이 gate는 동일 depth surface로 mask가 새는 경우에는 충분하지 않다. 예를 들어 target object와 같은 plane에 놓인 neighboring object가 mask에 포함되면, median depth band만으로는 분리되지 않을 수 있다. 이 한계는 connected component, object-level temporal association, tactile/force feedback과 같은 별도 cue로 보완해야 하는 부분이다.

### 3.3 Robust Center Estimation

초기 center는 arithmetic mean이 아니라 geometric median에 가깝게 추정된다.

\[
p^\star
=
\arg\min_{p\in\mathbb{R}^3}
\sum_{i=1}^{N}\|p_i-p\|_2.
\]

구현은 제한된 반복 횟수의 Weiszfeld update를 사용한다. mean은 mask leakage나 depth spike에 선형적으로 끌려가지만, geometric median은 outlier에 대한 breakdown 특성이 더 좋다. ROI point 수가 충분한 상황에서 center initialization의 안정성을 높이는 선택이다.

Tracking 단계에서는 이전 map-frame state를 optical frame으로 다시 투영한 center를 anchor로 사용한다. 이로 인해 매 프레임 center가 완전히 새로 정의되는 것이 아니라, temporal state 주변에서 re-measurement가 수행된다.

---

## 4. Gravity-Aligned Axis Estimation

### 4.1 Optical Frame에서의 중력 평면 구성

map frame의 gravity axis를
\[
g_m=[0,0,1]^\top
\]
라고 하면, optical frame에서의 gravity direction은
\[
g_o=(R^m_o)^\top g_m
\]
이다. Decision Module은 이 \(g_o\)에 수직인 2D plane을 구성하고, point cloud를 이 plane으로 투영한다. 즉 물체의 horizontal footprint를 추정하는 구조이다.

이 선택은 Euler angle 기반 object pose estimation보다 훨씬 잘 조건화되어 있다. Side-grasp에서 중요한 것은 gravity에 수직인 approach/closing axes이지, mask-derived point cloud의 roll-pitch 회전이 아니다. 따라서 gravity-aligned PCA는 불필요한 orientation degree of freedom을 제거한다.

### 4.2 Closed-Form 2D PCA

center \(a_o\) 주변의 centered point를
\[
\bar{p}_i=p_i-a_o
\]
라고 하자. 중력 평면 basis \(\{e_1,e_2\}\)에 대한 2D coordinate는
\[
q_i=
\begin{bmatrix}
e_1^\top \bar{p}_i\\
e_2^\top \bar{p}_i
\end{bmatrix}.
\]
이때 covariance
\[
\Sigma
=
\frac{1}{N}\sum_i q_iq_i^\top
=
\begin{bmatrix}
a & b\\
b & c
\end{bmatrix}
\]
의 principal direction을 closed form으로 계산한다.

Principal axis의 angle은
\[
\phi=\frac{1}{2}\operatorname{atan2}(2b,a-c)
\]
이며, eigenvalue는
\[
\lambda_{\max,\min}
=
\frac{a+c}{2}
\pm
\sqrt{\left(\frac{a-c}{2}\right)^2+b^2}.
\]

axis confidence는
\[
q_\lambda
=
\frac{\lambda_{\max}-\lambda_{\min}}
{\lambda_{\max}+\lambda_{\min}+\epsilon}
\]
로 정의된다. 원형에 가까운 footprint일수록 \(\lambda_{\max}\approx\lambda_{\min}\)이 되어 yaw direction이 ill-posed해지고, 길쭉한 물체일수록 \(q_\lambda\)가 커진다.

### 4.3 Degeneracy Hysteresis

Decision Module은 axis quality가 순간적으로 낮아졌다고 즉시 orientation을 뒤집지 않는다. 대신 두 가지 조건을 사용해 degeneracy를 판단한다.

1. PCA eigenvalue separation \(q_\lambda\)
2. 후보 grasp frame에서의 extent anisotropy

enter threshold와 recover threshold를 분리한 hysteresis를 사용하기 때문에, 임계값 근처에서 orientation update가 on/off로 빠르게 진동하는 현상을 줄인다.

\[
\text{degenerate enter}: q_\lambda < \tau_{\lambda}^{\text{enter}}
\;\text{or}\;
q_e < \tau_{e}^{\text{enter}},
\]
\[
\text{degenerate recover}: q_\lambda > \tau_{\lambda}^{\text{recover}}
\;\text{and}\;
q_e > \tau_{e}^{\text{recover}}.
\]

Degenerate 상태에서는 현재 프레임이 center update에는 사용될 수 있지만, yaw update는 보수적으로 제한된다. 이는 symmetric object에서 매 프레임 PCA 축이 \(90^\circ\) 또는 \(180^\circ\)로 jump하는 현상을 막기 위한 핵심 장치이다.

---

## 5. Candidate Grasp Frame Selection

Horizontal PCA는 두 개의 직교 후보축을 제공한다. Decision Module은 이 중 어떤 축을 approach axis로 사용할지 선택해야 한다.

후보 frame은 다음과 같이 볼 수 있다.

\[
r_1=h_1,\quad c_1=h_2,
\qquad
r_2=h_2,\quad c_2=h_1,
\]

여기서 \(r\)은 approach axis, \(c\)는 gripper closing axis이다. 각 후보에 대해 quantile extent를 계산한다.

\[
w = Q_{0.97}(c^\top p)-Q_{0.03}(c^\top p),
\]
\[
\ell = Q_{0.97}(r^\top p)-Q_{0.03}(r^\top p).
\]

기본적인 feasibility는 closing width \(w\)가 gripper stroke limit 안에 들어오는지로 판단된다. 즉,
\[
w \le w_{\max}
\]
인 후보가 우선된다. 양쪽이 모두 feasible하거나 모두 infeasible할 때는 approach prior와 width/length relation이 tie-breaker로 작동한다.

이 설계의 물리적 의미는 명확하다. Side grasp에서 gripper가 실제로 물체를 닫을 수 있는지는 closing direction의 width가 결정한다. Approach direction이 semantic하게 좋아 보여도 closing width가 gripper limit를 넘으면 실행 가능한 grasp가 아니다. 따라서 candidate selection의 첫 번째 기준은 learned score가 아니라 physical feasibility가 되어야 한다.

### 5.1 Approach Prior

Approach prior는 optical camera의 viewing direction을 gravity plane으로 투영하여 얻는다. 직관적으로 말하면, 카메라가 바라보는 방향과 일관된 side approach를 선호한다.

\[
a_o
=
\frac{(I-g_og_o^\top)e_z^o}
{\|(I-g_og_o^\top)e_z^o\|}.
\]

만약 camera optical z축이 gravity와 거의 평행하면 horizontal projection이 퇴화하므로, approach prior는 unreliable하다고 표시된다. 이때 초기화는 보수적인 fallback을 사용하고, tracking에서는 기존 state의 orientation을 더 신뢰한다.

---

## 6. State Observer Design

Decision Module의 본질은 single-frame estimator가 아니라 state observer이다. 내부 상태는 크게 네 가지로 볼 수 있다.

| 상태 | 의미 |
|---|---|
| \(p_m\) | map-frame target center |
| \(\theta_m\) | gravity-aligned yaw |
| extent cache | approach/closing/gravity 축 방향 half-extent |
| front-shell offset | 물체 front surface로부터 grasp target을 정의하기 위한 offset |

### 6.1 Initialization

초기화는 다음 순서로 이루어진다.

1. ROI mask와 depth에서 optical-frame point set을 생성한다.
2. Weiszfeld median으로 robust center seed를 구한다.
3. gravity plane에서 closed-form 2D PCA를 수행한다.
4. 두 후보 grasp frame 중 gripper feasibility와 approach prior에 맞는 frame을 선택한다.
5. quantile extent midpoint로 center를 보정한다.
6. optical-frame center와 approach axis를 map frame으로 lifting한다.
7. depth range, physical extent, approach prior, horizontal lift validity를 통과하면 state를 commit한다.

초기화가 실패하면 rejected status와 reason이 기록된다. 그러나 실패한 초기화가 곧바로 잘못된 pose publish로 이어지지 않는다. 유효 state가 없으면 `output_valid=False`가 되며, 유효 state가 이미 있으면 held/stale policy가 작동한다.

### 6.2 Tracking

Tracking 단계에서는 이전 map-frame state를 다시 optical frame으로 투영한다.

\[
p_{o,t-1}
=
(R^m_o)^\top(p_{m,t-1}-t^m_o).
\]

그 후 현재 point cloud에 대해 center, orientation, extent를 다시 측정한다. 핵심은 모든 측정값을 무조건 반영하지 않는다는 점이다.

Center update는 physical gate와 jump gate를 통과해야 한다.

\[
\|p^{\text{raw}}_{m,t}-p_{m,t-1}\| \le \Delta p_{\max},
\]
\[
\|\text{extent}^{\text{raw}}_t-\text{extent}_{t-1}\|_\infty
\le \Delta e_{\max}.
\]

통과한 경우에도 center는 rate-limited EMA로 갱신된다.

\[
p_{m,t}
=
p_{m,t-1}
\alpha_p\,
\operatorname{clip}_{\Delta p_{\text{step}}}
(p^{\text{raw}}_{m,t}-p_{m,t-1}).
\]

여기서 clipping은 단일 프레임 jump가 너무 큰 경우에도 state가 순간적으로 멀리 이동하지 않도록 하는 장치이다.

### 6.3 Orientation Update on \(S^1\)

Yaw는 Euclidean average가 아니라 circular difference를 사용해 갱신된다.

\[
\Delta\theta
=
\operatorname{wrap}(\theta^{\text{raw}}_t-\theta_{t-1})
\in[-\pi,\pi].
\]

axis gate가 통과하면
\[
\theta_t
=
\theta_{t-1}
\alpha_\theta\Delta\theta.
\]

axis gate는 다음을 요구한다.

1. PCA energy가 finite해야 한다.
2. axis quality가 threshold 이상이어야 한다.
3. raw yaw jump가 \(d\theta_{\max}\) 이하여야 한다.
4. center measurement가 유효해야 한다.
5. degeneracy 상태가 아니어야 한다.

이 구조는 yaw를 \(\mathbb{R}\)의 일반 스칼라처럼 smoothing할 때 발생하는 wrap-around 문제를 피한다. 또한 symmetric object에서 raw PCA axis가 흔들리더라도 이전 valid orientation이 유지된다.

---

## 7. Target Point Construction

Decision Module은 object center를 그대로 grasp target으로 사용하지 않는다. Side grasp에서는 gripper가 물체 중심을 향해 들어가는 것이 아니라, visible front shell에서 일정 penetration만큼 들어간 target을 잡아야 한다.

선택된 approach axis를 \(a_m\), front-shell offset을 \(s_f\), object approach length를 \(\ell\)이라고 하자. 그러면 front shell point는
\[
p_{\text{front}}^m
=
p_m+s_f a_m
\]
로 정의된다.

Penetration depth는
\[
\rho
=
\min(\eta \ell,\; \ell_g - m_f)
\]
로 제한된다. 여기서 \(\eta\)는 inward fraction, \(\ell_g\)는 gripper length limit, \(m_f\)는 finger margin이다.

최종 target은
\[
p_{\text{target}}^m
=
p_{\text{front}}^m+\rho a_m.
\]

이 식의 의미는 명확하다. Decision Module은 물체 전체 center를 찾는 것이 아니라, gripper가 실제로 접근해서 닫을 수 있는 **contact-relevant target point**를 생성한다. 따라서 `p_map_raw` 같은 진단값과 `T_map_target`의 translation은 같을 필요가 없다. `T_map_target`은 manipulation target이고, raw pose는 현재 프레임 geometry의 관측값이다.

---

## 8. Failure Semantics

현재 구현의 failure policy는 downstream safety 관점에서 중요하다. 단순히 실패 시 null을 반환하는 구조가 아니라, state availability와 measurement freshness를 분리한다.

| Status | 의미 | Downstream 해석 |
|---|---|---|
| reset | 아직 의미 있는 관측이 없음 | command 생성 금지 |
| rejected | 현재 관측이 state 생성에 실패 | valid state가 없으면 command 금지 |
| held | 현재 관측은 실패했지만 이전 state가 존재 | hold, slow-down, timeout policy 적용 |
| fresh | 현재 관측이 state update에 사용됨 | normal command 가능 |

Reason code는 rejection 원인을 더 세분화한다.

| Reason | 의미 |
|---|---|
| insufficient points | mask/depth 이후 point 수가 최소치 미만 |
| approach prior degenerate | optical viewing direction의 horizontal projection이 불충분 |
| initial physical gate | 초기 depth/extent가 물리 조건을 만족하지 않음 |
| initial horizontal component missing | map-frame horizontal yaw를 정의할 수 없음 |
| tracking physical gate | tracking 중 depth, extent, jump gate를 통과하지 못함 |

이 설계는 실험 분석에 유리하다. 전체 실패율을 하나로 보고하는 대신, perception failure, geometric degeneracy, physical infeasibility, temporal instability를 분리해서 ablation할 수 있기 때문이다.

---

## 9. Computational Characteristics

Decision Module의 주요 연산량은 ROI pixel 수 \(P\)와 selected point 수 \(N\)으로 표현할 수 있다.

| 단계 | 복잡도 | 특징 |
|---|---:|---|
| guided depth filtering | \(\mathcal{O}(P k^2)\) | pooling 기반 dense ROI 연산 |
| mask backprojection | \(\mathcal{O}(P)\) | valid pixel selection 및 point lifting |
| depth median/band | \(\mathcal{O}(N)\) to \(\mathcal{O}(N\log N)\) | backend 구현에 따라 다르며 point 수에 민감 |
| Weiszfeld median | \(\mathcal{O}(IN)\) | \(I\)는 작은 고정 반복 수 |
| 2D PCA | \(\mathcal{O}(N)\) | 2x2 covariance와 closed-form eigen 계산 |
| quantile extent | \(\mathcal{O}(N)\) average | kth-value 기반 robust extent |
| EMA/gates | \(\mathcal{O}(1)\) | state update |

이 구조는 dense learned model을 다시 호출하지 않으며, 대부분의 비용이 ROI-local tensor operation에 집중된다. 따라서 전체 latency는 ROI 크기, mask leakage로 인한 point 수, guided filter kernel 크기, quantile/median 연산의 backend 성능에 민감하다.

Perception-Decision Pipeline 관점에서 Decision latency를 해석할 때 주의할 점은 TF lookup과 host synchronization을 분리해야 한다는 것이다. Decision Module 자체는 numeric `T_map_optical`을 입력으로 받기 때문에, transform lookup의 blocking latency는 pipeline wrapper의 책임이다. 따라서 실험 표에서는 다음 항목을 분리해 보고하는 것이 가장 설득력 있다.

1. ROI preparation latency
2. TF lookup and transform preparation latency
3. Decision core latency
4. End-to-end Perception-to-Decision latency

이 분리는 reviewer가 "geometric decision이 느린 것인가, ROS2/TF boundary가 느린 것인가"를 구분할 수 있게 해준다.

---

## 10. Why This Design Is Scientifically Reasonable

### 10.1 Task-Relevant State Reduction

Full object pose estimation은 일반적으로 다음과 같은 hidden assumption을 요구한다.

1. object CAD 또는 category-level canonical frame이 존재한다.
2. observed partial point cloud가 canonical orientation을 식별할 만큼 충분하다.
3. segmentation mask와 depth가 enough coverage를 제공한다.

현재 L-OMM pipeline의 목적은 category-level object recognition이나 full 6D pose recovery가 아니라, edge device에서 real-time manipulation target을 생성하는 것이다. 따라서 Decision Module은 task variable인 side-grasp pose만 추정한다. 이는 문제를 약하게 푸는 것이 아니라, manipulation objective에 대해 더 잘 정식화한 것이다.

### 10.2 Deterministic Geometry as a Safety Layer

Perception Module은 neural segmentation을 담당한다. Neural output을 곧바로 robot action으로 연결하면 rare mask failure가 action jump로 증폭될 수 있다. Decision Module은 다음과 같은 deterministic safety layer를 제공한다.

1. depth band로 mask leakage 일부 제거
2. physical gripper width로 infeasible grasp 제거
3. center/extent jump gate로 temporal discontinuity 차단
4. degeneracy hysteresis로 symmetric footprint yaw jump 억제
5. stale-aware output으로 transient perception failure와 command discontinuity 분리

이 구조는 black-box learned policy보다 해석 가능성이 높고, edge deployment에서 failure diagnosis가 쉽다.

### 10.3 Geometry-Consistent Frame Handling

가장 중요한 구현적 장점은 optical frame과 map frame의 역할이 분리되어 있다는 점이다.

Raw point cloud의 metric geometry는 optical frame에서 가장 자연스럽다. 반면 robot control target은 map/world frame에서 정의되어야 한다. 현재 구현은
\[
p_m = R^m_o p_o+t^m_o,
\qquad
x_m = R^m_o x_o
\]
로 raw center와 axis를 lifting한 뒤, time state를 map frame에서 유지한다. 이로 인해 camera motion, TF 변화, robot base motion이 있을 때도 state의 의미가 world-consistent하게 유지된다.

---

## 11. Limitations and Reviewer-Critical Points

### 11.1 Same-Depth Mask Leakage

Depth-band filtering은 target과 leakage region의 depth가 다를 때 효과적이다. 그러나 같은 tabletop plane 또는 같은 거리의 neighboring object가 mask에 포함되면, 현재 gate만으로는 이를 완전히 제거할 수 없다. 이 경우 PCA footprint와 extent가 왜곡되어 approach axis 또는 width feasibility가 잘못 계산될 수 있다.

Validation에서는 다음 ablation이 필요하다.

1. single isolated object
2. adjacent object with different depth
3. adjacent object with similar depth
4. severe mask over-segmentation

이 네 조건에서 center error, yaw error, stale ratio, physical gate rejection ratio를 분리해 보고해야 한다.

### 11.2 Degenerate Object Classes

원형 컵, 정사각형 박스, 구형 물체처럼 horizontal footprint가 isotropic한 경우 yaw는 본질적으로 식별 불가능하다. 이때 yaw error를 절대 orientation 기준으로 평가하면 불공정하다. 대신 다음 지표를 사용해야 한다.

1. grasp success rate
2. closing width feasibility
3. target point stability
4. yaw update variance under degeneracy

즉 symmetric object에서는 "정답 yaw를 맞히는가"보다 "불필요하게 yaw가 흔들리지 않는가"가 더 중요한 지표이다.

### 11.3 TF Boundary Latency

Decision Module은 `T_map_optical`이 이미 준비되어 있다고 가정한다. 실제 ROS2 pipeline에서는 TF lookup, message timestamp alignment, GPU tensor preparation이 latency와 jitter를 만들 수 있다. 논문에서는 Decision Module의 algorithmic latency와 system integration latency를 반드시 분리해야 한다.

### 11.4 Single-Object ROI Assumption

현재 구조는 Perception Module이 제공한 ROI가 하나의 target object를 중심으로 한다는 가정을 갖는다. Multi-object mask가 하나의 ROI에 섞이면 geometric observer는 이를 하나의 rigid-ish footprint로 해석한다. 따라서 multi-object clutter에서는 upstream instance selection 또는 ROI association이 contribution의 중요한 전제조건이다.

---

## 12. Validation Strategy

Decision Module의 실험은 단순 average latency만으로는 부족하다. 다음 실험 구성이 필요하다.

### 12.1 Geometric Accuracy

Motion capture 또는 calibrated tabletop setup에서 다음을 측정한다.

\[
e_p = \|p^{m}_{\text{target}}-\bar{p}^{m}_{\text{target}}\|_2,
\]
\[
e_\theta =
\left|
\operatorname{wrap}(\theta-\bar{\theta})
\right|.
\]

단, symmetric object는 yaw ground truth가 ill-defined하므로 별도 group으로 분리한다.

### 12.2 Temporal Stability

정지 물체에 대해 camera 또는 robot이 미세하게 움직일 때 다음을 측정한다.

1. target point standard deviation
2. yaw standard deviation
3. fresh/held transition count
4. stale duration distribution
5. one-frame jump magnitude before/after gates

이 실험은 observer 설계의 가치를 가장 직접적으로 보여준다.

### 12.3 Physical Feasibility

Gripper limit와 object width의 관계를 sweep한다.

\[
w_{\text{object}}\in
\{0.5w_{\max},0.8w_{\max},1.0w_{\max},1.2w_{\max}\}.
\]

기대되는 결과는 \(w_{\max}\) 이하에서는 feasible ratio가 높고, 초과 영역에서는 physical gate가 안정적으로 infeasible을 반환하는 것이다.

### 12.4 Mask Noise Ablation

Synthetic mask perturbation을 추가한다.

1. boundary erosion/dilation
2. random holes
3. side leakage
4. adjacent-object merge

각 조건에서 point count, depth-band count, center error, yaw stability, rejection reason histogram을 보고하면, Decision Module이 어떤 failure를 흡수하고 어떤 failure에는 취약한지 명확히 보일 수 있다.

### 12.5 End-to-End Manipulation

최종 평가는 grasp success만이 아니라 failure taxonomy와 함께 보고해야 한다.

1. perception miss
2. insufficient points
3. geometric degeneracy
4. infeasible width
5. stale timeout
6. controller execution failure

이렇게 분리해야 Decision Module의 contribution이 단순 성공률 향상인지, failure mode를 더 잘 구조화한 것인지 드러난다.

---

## 13. Parameter Interpretation

주요 parameter는 단순 tuning knob가 아니라 각각 물리적 의미를 가진다.

| Parameter | 의미 | 너무 작을 때 | 너무 클 때 |
|---|---|---|---|
| `min_valid_points` | geometry 추정 최소 point 수 | noisy small mask도 통과 | sparse failure 증가 |
| `depth_band_m` | median depth 주변 허용 폭 | valid object surface 손실 | background leakage 허용 |
| `quality_thr` | PCA axis 신뢰 threshold | yaw jitter 증가 | yaw update 과도 억제 |
| `dtheta_max` | 한 프레임 yaw jump 허용량 | real rotation 추종 저하 | PCA flip 허용 |
| `max_center_jump` | raw center gate | target loss 증가 | mask jump가 state로 유입 |
| `max_center_step` | EMA step limit | response lag 증가 | abrupt command 가능 |
| `gripper_width_limit` | closing feasibility | reachable object reject | infeasible grasp accept |
| `inward_fraction` | front shell penetration | shallow grasp | over-penetration |

논문에서는 이 parameter들을 empirical tuning으로만 설명하지 말고, sensor noise scale, gripper geometry, object size distribution, controller bandwidth와 연결해 설명하는 것이 좋다.

---

## 14. Recommended Reporting in the Paper

Decision Module을 논문에 쓸 때는 “PCA로 grasp pose를 구했다” 수준으로 쓰면 contribution이 약해 보인다. 다음과 같이 positioning하는 것이 더 적절하다.

1. **Gravity-constrained side-grasp manifold**  
   Full \(SE(3)\) pose 대신 task-relevant \(p_m,\theta\) state를 추정한다.

2. **ROI-local GPU geometric observer**  
   Segmentation output을 deterministic geometric state로 변환하며, dense learned model을 추가 호출하지 않는다.

3. **Stale-aware safety contract**  
   measurement failure와 output availability를 분리해 transient perception failure가 바로 robot command discontinuity로 이어지지 않게 한다.

4. **Physical feasibility embedded in pose selection**  
   Gripper width/length constraint가 후보 frame 선택과 target point construction에 직접 들어간다.

이 네 가지를 contribution 또는 system design principle로 정리하면, 단순 engineering glue가 아니라 embodied perception-to-action interface로서 설득력이 생긴다.

---

## 15. Final Assessment

현재 `decision_module.py`의 설계는 Perception Module의 neural output을 robot-control-ready target pose로 변환하는 중간 계층으로서 상당히 잘 정식화되어 있다. 특히 optical-frame raw geometry와 map-frame temporal state를 분리한 점, yaw를 \(S^1\) 상태로 다룬 점, degeneracy hysteresis와 stale-aware output을 둔 점은 실제 robot deployment에서 중요한 engineering detail을 넘어 학술적으로도 방어 가능한 구조이다.

다만 이 모듈의 성능을 논문 수준으로 주장하려면, 단순 평균 latency와 grasp success만으로는 부족하다. Mask leakage, symmetric footprint, TF latency, stale duration을 분리해서 분석해야 한다. 이 검증이 포함되면 Decision Module은 “segmentation 결과를 PCA로 후처리한 코드”가 아니라, **edge-deployable geometric decision observer for robust side-grasping**으로 충분히 positioning할 수 있다.
