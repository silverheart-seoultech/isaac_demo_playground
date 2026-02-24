# H1 Humanoid Velocity Tracking — Manager-Based Locomotion
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

Unitree H1 이족보행 로봇의 velocity tracking 환경. 공통 베이스(`LocomotionVelocityRoughEnvCfg`)를 상속하되, 이족보행의 특성에 맞게 reward 구조를 대폭 수정합니다. ANYmal-C가 베이스를 그대로 사용하는 것과 달리, H1은 별도의 `H1Rewards` 클래스를 정의하여 termination penalty, biped-specific gait reward, 관절 편차 페널티를 추가합니다.

이족보행은 사족보행보다 본질적으로 불안정하여 학습이 2배 더 오래 걸리고(3000 iterations vs 1500), reward shaping이 더 정교해야 합니다. 이 환경은 그 차이를 구체적으로 보여주는 좋은 사례입니다.

## Architecture

### 상속 구조

```
LocomotionVelocityRoughEnvCfg
    │
    └── H1RoughEnvCfg (rough_env_cfg.py)
            │
            ├── rewards = H1Rewards()          ← 커스텀 reward 클래스
            │       ├── 기존 reward 수정 (6개)
            │       ├── 새 reward 추가 (6개)
            │       └── 기존 reward 비활성 (2개)
            │
            ├── scene.robot = H1_MINIMAL_CFG
            ├── height_scanner → torso_link    ← base → torso 변경
            ├── events 대폭 수정               ← push/mass randomization 비활성
            ├── commands 범위 제한             ← 전진만, 횡이동 없음
            └── terminations → torso_link      ← base → torso 접촉 감지
```

### H1 vs ANYmal-C: 구조적 차이

H1의 config 변경이 많은 이유는 이족보행 로봇의 물리적 특성 때문입니다:

1. **불안정성**: 2족은 지지 다각형(support polygon)이 좁아 외란에 취약 → push_robot 비활성, 정지 시작(velocity=0)
2. **관절 구조**: 팔, 허리 등 보행에 불필요한 관절이 있음 → joint_deviation 페널티로 기본 자세 유지
3. **발 접촉 패턴**: trot 대신 biped walking gait → `feet_air_time_positive_biped` 사용
4. **이동 방향**: 전진 위주 (lin_vel_y = 0) → 횡이동은 이족에서 비효율적

## Source Files

| 파일 | 역할 |
|---|---|
| `velocity_env_cfg.py` | 공통 베이스 config |
| `config/h1/rough_env_cfg.py` | H1 rough terrain (reward 재정의, event/command 수정) |
| `config/h1/flat_env_cfg.py` | H1 flat terrain |
| `config/h1/__init__.py` | Gymnasium 등록 (4개 env ID) |
| `config/h1/agents/rsl_rl_ppo_cfg.py` | RSL-RL PPO 하이퍼파라미터 |
| `config/h1/agents/skrl_*_ppo_cfg.yaml` | SKRL PPO config |
| `mdp/` | 공유 MDP term 함수들 |

## 핵심 코드 분석

### H1Rewards 클래스 — Reward 재설계

H1은 베이스의 `RewardsCfg`를 상속한 `H1Rewards`에서 reward를 전면 재구성합니다:

```python
@configclass
class H1Rewards(RewardsCfg):
    # 신규 추가
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(func=mdp.feet_air_time_positive_biped, weight=0.25,
        params={"sensor_cfg": ..., "body_names": ".*ankle_link", "threshold": 0.4})
    feet_slide = RewTerm(func=mdp.feet_slide, weight=-0.25,
        params={"sensor_cfg": ..., "body_names": ".*ankle_link"})
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.0,
        params={"joint_names": ".*_ankle"})
    joint_deviation_hip = RewTerm(func=mdp.joint_deviation_l1, weight=-0.2,
        params={"joint_names": [".*_hip_yaw", ".*_hip_roll"]})
    joint_deviation_arms = RewTerm(func=mdp.joint_deviation_l1, weight=-0.2,
        params={"joint_names": [".*_shoulder_.*", ".*_elbow"]})
    joint_deviation_torso = RewTerm(func=mdp.joint_deviation_l1, weight=-0.1,
        params={"joint_names": "torso"})

    # 기존 reward 수정
    track_lin_vel_xy_exp: yaw-frame 추종으로 변경 (std=0.5)
    track_ang_vel_z_exp: world-frame 추종, weight 1.0→1.0 (std=0.5)
    lin_vel_z_l2 = None                    # 비활성
    flat_orientation_l2.weight = -1.0      # 0.0 → -1.0 활성화
    action_rate_l2.weight = -0.005         # -0.01 → -0.005 완화
    dof_acc_l2.weight = -1.25e-7           # -2.5e-7 → -1.25e-7 완화
    dof_torques_l2.weight = 0.0            # -1e-5 → 0.0 비활성
    undesired_contacts = None              # 비활성
```

### Reward 가중치 분석

| Reward Term | Weight | ANYmal-C | 변경 이유 |
|---|---|---|---|
| `termination_penalty` | **-200.0** | 없음 | 넘어짐의 비용이 극히 크므로 강한 생존 동기 부여 |
| `track_lin_vel_xy_exp` | +1.0 | +1.0 (동일) | yaw-frame 추종으로 함수만 변경 |
| `track_ang_vel_z_exp` | +1.0 | +0.5 | 이족은 회전 제어가 더 중요 |
| `flat_orientation_l2` | **-1.0** | 0.0 | 이족은 torso 기울기 유지가 필수 |
| `feet_air_time_positive_biped` | +0.25 | +0.125 | biped gait 유도에 더 강한 신호 필요 |
| `feet_slide` | **-0.25** | 없음 | 발 미끄러짐 방지 (이족은 접지력이 중요) |
| `joint_deviation_hip` | -0.2 | 없음 | hip yaw/roll이 벌어지면 보행 불안정 |
| `joint_deviation_arms` | -0.2 | 없음 | 팔이 흔들리면 무게중심 이동으로 불안정 |
| `joint_deviation_torso` | -0.1 | 없음 | 허리 비틀림 억제 |
| `dof_pos_limits` | -1.0 | 0.0 | 발목 관절 한계 도달 방지 |
| `lin_vel_z_l2` | **없음** | -2.0 | 이족 보행에서 수직 진동은 자연스러움 |
| `undesired_contacts` | **없음** | -1.0 | 허벅지 접촉 페널티 불필요 (이족 구조) |

**핵심 설계 철학**: 
- `termination_penalty(-200.0)`이 다른 모든 reward를 압도 → 정책은 무엇보다 넘어지지 않는 것을 최우선으로 학습
- `joint_deviation` 3종은 보행에 직접 기여하지 않는 관절(팔, 허리)을 기본 자세로 유지시킴
- `feet_slide(-0.25)`는 이족만의 고유 문제: 발이 미끄러지면 자세 제어가 급격히 어려워짐

### Yaw-Frame Velocity Tracking

```python
track_lin_vel_xy_exp = RewTerm(
    func=mdp.track_lin_vel_xy_yaw_frame_exp,  # ← ANYmal은 track_lin_vel_xy_exp
    weight=1.0,
    params={"command_name": "base_velocity", "std": 0.5},
)
```

ANYmal-C는 body frame에서 속도를 추종하지만, H1은 **yaw-frame**에서 추종합니다. Yaw-frame은 로봇의 yaw 회전만 반영하고 roll/pitch는 무시합니다. 이족보행은 상체가 자연스럽게 기울어지므로, body frame 추종을 사용하면 기울기에 의한 속도 방향 변화가 reward를 교란합니다.

### Event 수정

```python
def __post_init__(self):
    # 비활성
    self.events.push_robot = None           # 외란 밀기 비활성 (넘어짐 위험)
    self.events.add_base_mass = None         # 질량 랜덤화 비활성
    self.events.base_com = None              # CoM 랜덤화 비활성

    # 초기 상태 수정
    self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)  # default 자세로만
    self.events.reset_base.params["velocity_range"] = all zeros           # 정지 상태 시작
```

이족보행의 불안정성 때문에 domain randomization을 대폭 줄였습니다. 질량/CoM 랜덤화와 push를 비활성화하고, 초기 자세를 default 그대로(1.0배), 초기 속도를 0으로 설정합니다. 이는 학습 초기에 안정적인 standing부터 학습할 수 있게 합니다.

### Command 범위 제한

```python
self.commands.base_velocity.ranges.lin_vel_x = (0.0, 1.0)   # 전진만 (후진 없음)
self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)   # 횡이동 없음
self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)  # 회전은 유지
```

이족보행에서 후진과 횡이동은 학습이 매우 어렵고 실용성이 낮습니다. 전진과 회전에만 집중하여 학습 효율을 높입니다.

### 물리 설정

베이스와 동일한 물리 파라미터를 사용합니다:

| 파라미터 | 값 |
|---|---|
| `dt` | 0.005 s (200Hz) |
| `decimation` | 4 |
| 제어 주파수 | 50 Hz |
| `episode_length_s` | 20.0 s (학습), 40.0 s (평가) |
| `action_scale` | 0.5 |
| `num_envs` | 4096 |

## 실행 방법

```bash
cd ~/workspace/IsaacLab

# Rough terrain 학습 (RSL-RL)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Rough-H1-v0 \
    --num_envs 4096

# Flat terrain 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-H1-v0 \
    --num_envs 4096

# 사전학습 체크포인트 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Rough-H1-Play-v0 \
    --use_pretrained_checkpoint

# Headless 학습 (서버)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Rough-H1-v0 \
    --num_envs 4096 --headless
```

### 등록된 Environment IDs

| ID | Terrain | 용도 |
|---|---|---|
| `Isaac-Velocity-Rough-H1-v0` | Rough | 학습 |
| `Isaac-Velocity-Rough-H1-Play-v0` | Rough | 평가 (50 envs, episode 40s) |
| `Isaac-Velocity-Flat-H1-v0` | Flat | 학습 |
| `Isaac-Velocity-Flat-H1-Play-v0` | Flat | 평가 |

### 학습 하이퍼파라미터 (RSL-RL PPO)

| 파라미터 | Rough | Flat |
|---|---|---|
| Network | [512, 256, 128] | [128, 128, 128] |
| Activation | ELU | ELU |
| Max Iterations | **3000** | 1000 |
| Num Steps/Env | 24 | 24 |
| Learning Rate | 1e-3 (adaptive) | 1e-3 (adaptive) |
| Entropy Coef | **0.01** | 0.01 |
| Discount (γ) | 0.99 | 0.99 |
| GAE (λ) | 0.95 | 0.95 |

ANYmal-C 대비 주요 차이:
- **Max iterations 3000** (vs 1500): 이족보행은 수렴이 느림
- **Entropy coef 0.01** (vs 0.005): 더 많은 탐색이 필요 (복잡한 gait 발견)
- Flat도 1000 iterations (ANYmal-C는 300)

## Comparison: H1 Reward Shaping 전략

H1의 reward 설계는 "구조적 제약 → reward로 표현"이라는 패턴을 보여줍니다:

```
이족 구조적 특성              → Reward 설계 결정
─────────────────────────────────────────────
좁은 지지 다각형             → termination_penalty = -200.0
자연스러운 상체 기울임       → yaw-frame tracking (body-frame 대신)
보행 무관 관절 (팔, 허리)   → joint_deviation penalty (3종)
biped walking gait          → feet_air_time_positive_biped
발 미끄러짐 취약성          → feet_slide penalty
수직 진동의 자연스러움       → lin_vel_z_l2 비활성
```

이 패턴은 새로운 로봇에 대한 reward shaping 시 참고할 수 있는 가이드라인이다: 로봇의 물리적 특성을 분석하고, 각 특성에 대응하는 reward term을 설계합니다.

## Further Reading

- **H1 config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/h1/rough_env_cfg.py`
- **H1 에셋**: `isaaclab_assets.H1_MINIMAL_CFG`
- **biped MDP terms**: `mdp.feet_air_time_positive_biped`, `mdp.feet_slide`, `mdp.track_lin_vel_xy_yaw_frame_exp`
- **공통 베이스 config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/velocity_env_cfg.py`
- **RSL-RL PPO config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/h1/agents/rsl_rl_ppo_cfg.py`
