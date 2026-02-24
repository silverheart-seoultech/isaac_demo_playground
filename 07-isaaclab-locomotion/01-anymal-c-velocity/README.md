# ANYmal-C Velocity Tracking — Manager-Based Locomotion
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

ANYmal-C 사족보행 로봇의 velocity tracking 환경. Isaac Lab의 locomotion velocity 공통 베이스(`LocomotionVelocityRoughEnvCfg`)를 상속하며, 로봇 에셋만 ANYmal-C로 교체한 최소 구성입니다. 13종의 로봇이 동일한 베이스 config를 공유하는 구조에서 ANYmal-C는 가장 기본적인 구성으로, velocity locomotion 파이프라인의 레퍼런스 구현이라 할 수 있습니다.

Rough terrain과 Flat terrain 두 가지 변형이 있으며, RSL-RL/RL Games/SKRL 세 가지 학습 프레임워크를 모두 지원합니다. Symmetry augmentation을 활용한 변형도 제공됩니다.

## Architecture

### 상속 구조

```
ManagerBasedRLEnvCfg
    │
    └── LocomotionVelocityRoughEnvCfg (velocity_env_cfg.py)
            │   ├── MySceneCfg          ← terrain + robot(MISSING) + sensors
            │   ├── ObservationsCfg     ← 8개 observation term
            │   ├── RewardsCfg          ← 11개 reward term (2 optional)
            │   ├── EventCfg            ← domain randomization
            │   ├── TerminationsCfg     ← time_out + base_contact
            │   └── CurriculumCfg       ← terrain difficulty
            │
            └── AnymalCRoughEnvCfg (rough_env_cfg.py)
                    └── scene.robot = ANYMAL_C_CFG  ← 로봇만 교체
```

ANYmal-C config는 베이스를 있는 그대로 사용하며, `__post_init__`에서 `self.scene.robot = ANYMAL_C_CFG`만 설정합니다. 이는 베이스 config가 ANYmal-C를 기준으로 설계되었기 때문입니다. 다른 로봇(H1, Go2 등)은 이 베이스에서 reward, event, command 등을 추가로 커스터마이징합니다.

### Velocity Command 시스템

```
UniformVelocityCommandCfg
    │
    ├── lin_vel_x: (-1.0, 1.0) m/s    ← 전후 속도
    ├── lin_vel_y: (-1.0, 1.0) m/s    ← 좌우 속도
    ├── ang_vel_z: (-1.0, 1.0) rad/s  ← 회전 속도
    ├── heading: (-π, π)              ← 목표 방향 (heading_command=True)
    │
    ├── resampling_time: 10.0s         ← 10초마다 새 명령
    ├── rel_standing_envs: 0.02        ← 2% 환경은 정지 명령
    └── heading_control_stiffness: 0.5 ← heading → ang_vel 변환 게인
```

`heading_command=True`이면 목표 heading을 설정하고, heading error에 stiffness를 곱하여 `ang_vel_z`를 자동 생성합니다. 이는 방향 추종을 더 자연스럽게 만듭니다.

## Source Files

| 파일 | 역할 |
|---|---|
| `velocity_env_cfg.py` | 공통 베이스 config (Scene, Obs, Reward, Event, Termination, Curriculum) |
| `config/anymal_c/rough_env_cfg.py` | ANYmal-C rough terrain 환경 (로봇 교체만) |
| `config/anymal_c/flat_env_cfg.py` | ANYmal-C flat terrain 환경 |
| `config/anymal_c/__init__.py` | Gymnasium 등록 (4개 env ID) |
| `config/anymal_c/agents/rsl_rl_ppo_cfg.py` | RSL-RL PPO 하이퍼파라미터 |
| `config/anymal_c/agents/skrl_*_ppo_cfg.yaml` | SKRL PPO config |
| `config/anymal_c/agents/rl_games_*_ppo_cfg.yaml` | RL Games PPO config |
| `mdp/` | 공유 MDP term 함수들 (observation, reward, event, command) |

## 핵심 코드 분석

### Observation Space (8 terms)

```python
class PolicyCfg(ObsGroup):
    base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
    base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
    projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
    velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
    joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
    joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5))
    actions = ObsTerm(func=mdp.last_action)
    height_scan = ObsTerm(func=mdp.height_scan, noise=Unoise(n_min=-0.1, n_max=0.1), clip=(-1.0, 1.0))
```

| Term | 차원 | Noise 범위 | 역할 |
|---|---|---|---|
| `base_lin_vel` | 3 | ±0.1 | 베이스 선속도 (body frame) |
| `base_ang_vel` | 3 | ±0.2 | 베이스 각속도 |
| `projected_gravity` | 3 | ±0.05 | body frame으로 투영된 중력 벡터 → 기울기 정보 |
| `velocity_commands` | 3 | 없음 | 목표 선속도/각속도 명령 |
| `joint_pos` | 12 | ±0.01 | 관절 위치 (default 기준 상대값) |
| `joint_vel` | 12 | ±1.5 | 관절 속도 |
| `actions` | 12 | 없음 | 이전 스텝 action |
| `height_scan` | 187 | ±0.1 | 지형 높이맵 (1.6m × 1.0m 그리드, 0.1m 해상도) |

**총 observation 차원**: 3+3+3+3+12+12+12+187 = **235D** (rough terrain), height_scan 제외 시 48D (flat terrain)

**Noise 설계 근거**: 각 term에 uniform noise를 주입하여 sim-to-real transfer 시 센서 노이즈에 대한 robustness를 확보합니다. `joint_vel`의 noise가 ±1.5로 가장 큰 이유는 실제 로봇의 관절 속도 추정이 가장 부정확하기 때문입니다. `enable_corruption = True`는 학습 시 활성화, 평가 시 비활성화합니다.

### Reward Decomposition

```python
class RewardsCfg:
    # Task rewards
    track_lin_vel_xy_exp  weight=1.0    std=√0.25
    track_ang_vel_z_exp   weight=0.5    std=√0.25

    # Penalties
    lin_vel_z_l2          weight=-2.0
    ang_vel_xy_l2         weight=-0.05
    dof_torques_l2        weight=-1e-5
    dof_acc_l2            weight=-2.5e-7
    action_rate_l2        weight=-0.01

    # Gait rewards
    feet_air_time         weight=0.125  threshold=0.5s

    # Safety penalties
    undesired_contacts    weight=-1.0   body=".*THIGH"

    # Optional (disabled by default)
    flat_orientation_l2   weight=0.0
    dof_pos_limits        weight=0.0
```

| Reward Term | Weight | 수식 | 설계 의도 |
|---|---|---|---|
| `track_lin_vel_xy_exp` | +1.0 | exp(-\|v_cmd - v_actual\|² / 0.25) | 목표 선속도 추종. Exponential로 목표 근처에서 sharply rewarding |
| `track_ang_vel_z_exp` | +0.5 | exp(-\|ω_cmd - ω_actual\|² / 0.25) | 목표 각속도 추종. 선속도보다 낮은 가중치 |
| `lin_vel_z_l2` | -2.0 | v_z² | 수직 방향 진동 억제 (bouncing 방지) |
| `ang_vel_xy_l2` | -0.05 | ω_x² + ω_y² | roll/pitch 각속도 억제 (안정성) |
| `dof_torques_l2` | -1e-5 | Σ τ_i² | 에너지 효율 (매우 약한 regularizer) |
| `dof_acc_l2` | -2.5e-7 | Σ ä_i² | 관절 가속도 억제 (부드러운 움직임) |
| `action_rate_l2` | -0.01 | Σ (a_t - a_{t-1})² | action 변화율 억제 (jitter 방지) |
| `feet_air_time` | +0.125 | Σ max(0, t_air - 0.5) | 발이 0.5초 이상 공중에 있으면 보상 → trotting gait 유도 |
| `undesired_contacts` | -1.0 | 허벅지 접촉 감지 시 | 허벅지가 지면에 닿는 비정상 자세 페널티 |

**가중치 계층 분석**: 
1. `lin_vel_z_l2(-2.0)` > `track_lin_vel_xy_exp(+1.0)` — 수직 진동 억제가 수평 추종보다 중요
2. `undesired_contacts(-1.0)` — 비정상 접촉은 즉각적인 큰 페널티
3. `feet_air_time(+0.125)` — gait 패턴은 약하게 유도 (정책이 자연스럽게 발견)
4. `dof_torques(-1e-5)`, `dof_acc(-2.5e-7)` — 매우 약한 regularizer로 학습 초기에는 무시 가능

### Domain Randomization (EventCfg)

```python
# Startup (1회)
physics_material:  static_friction=0.8, dynamic_friction=0.6  (64 buckets)
add_base_mass:     ±5.0 kg 추가
base_com:          CoM 이동 x±0.05, y±0.05, z±0.01 m

# Reset (에피소드 시작마다)
reset_base:        위치 ±0.5m, yaw ±π, 속도 ±0.5
reset_robot_joints: default의 0.5~1.5배

# Interval (주기적)
push_robot:        10~15초마다 ±0.5 m/s velocity push
```

**설계 근거**: Startup randomization은 물리 파라미터(마찰, 질량, CoM)를 변경하여 다양한 로봇 개체를 시뮬레이션합니다. 64개 마찰 bucket으로 물리 엔진의 효율을 유지하면서 다양성을 확보합니다. Interval push는 외란(disturbance) 복구 능력을 학습시킵니다.

### 물리 설정

| 파라미터 | 값 | 의미 |
|---|---|---|
| `dt` | 0.005 s | PhysX 시뮬레이션 타임스텝 (200Hz) |
| `decimation` | 4 | 물리 4스텝당 1 제어 스텝 |
| 제어 주파수 | 50 Hz | 0.005 × 4 = 0.02s |
| `episode_length_s` | 20.0 s | 최대 1000 제어 스텝 |
| `action_scale` | 0.5 | action [-1,1] → joint offset [-0.5, 0.5] rad |
| `num_envs` | 4096 | 병렬 환경 수 |
| `env_spacing` | 2.5 m | 환경 간 간격 |

### Terrain Curriculum

```python
class CurriculumCfg:
    terrain_levels = CurrTerm(func=mdp.terrain_levels_vel)
```

`terrain_levels_vel`은 로봇의 velocity tracking 성능에 따라 terrain 난이도를 자동 조절합니다. 추적 오차가 작으면 더 어려운 terrain으로 이동하고, 크면 쉬운 terrain으로 되돌립니다. 이 curriculum은 `ROUGH_TERRAINS_CFG`의 terrain generator와 연동되어, boxes/stairs/random_rough 등 다양한 sub-terrain에서 점진적으로 학습합니다.

### Height Scanner 설정

```python
height_scanner = RayCasterCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base",
    offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
    ray_alignment="yaw",
    pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
)
```

로봇 base에서 하방으로 ray를 쏘아 지형 높이를 측정합니다. 1.6m × 1.0m 영역을 0.1m 해상도로 스캔하여 (16+1) × (10+1) = 187개 높이 포인트를 생성합니다. `ray_alignment="yaw"`는 로봇의 yaw 방향으로만 정렬하여 roll/pitch 변화에 독립적인 지형 정보를 제공합니다.

## 실행 방법

```bash
cd ~/workspace/IsaacLab

# Rough terrain 학습 (RSL-RL)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Rough-Anymal-C-v0 \
    --num_envs 4096

# Flat terrain 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0 \
    --num_envs 4096

# Symmetry augmentation 포함 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Rough-Anymal-C-v0 \
    --num_envs 4096 \
    --agent_cfg rsl_rl_with_symmetry_cfg_entry_point

# 사전학습 체크포인트 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Rough-Anymal-C-Play-v0 \
    --use_pretrained_checkpoint

# SKRL로 학습
./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py \
    --task Isaac-Velocity-Rough-Anymal-C-v0

# Headless 학습 (서버)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Rough-Anymal-C-v0 \
    --num_envs 4096 --headless
```

### 등록된 Environment IDs

| ID | Terrain | 용도 |
|---|---|---|
| `Isaac-Velocity-Rough-Anymal-C-v0` | Rough | 학습 |
| `Isaac-Velocity-Rough-Anymal-C-Play-v0` | Rough | 평가 (50 envs, no corruption) |
| `Isaac-Velocity-Flat-Anymal-C-v0` | Flat | 학습 |
| `Isaac-Velocity-Flat-Anymal-C-Play-v0` | Flat | 평가 |

### 학습 하이퍼파라미터 (RSL-RL PPO)

| 파라미터 | Rough | Flat |
|---|---|---|
| Network | [512, 256, 128] | [128, 128, 128] |
| Activation | ELU | ELU |
| Max Iterations | 1500 | 300 |
| Num Steps/Env | 24 | 24 |
| Learning Rate | 1e-3 (adaptive) | 1e-3 (adaptive) |
| Entropy Coef | 0.005 | 0.005 |
| Discount (γ) | 0.99 | 0.99 |
| GAE (λ) | 0.95 | 0.95 |
| Desired KL | 0.01 | 0.01 |

Rough terrain은 더 큰 네트워크([512,256,128])와 더 긴 학습(1500 iter)을 사용합니다. Height scan observation(187D)이 추가되므로 입력 차원이 크고 terrain 다양성에 대응해야 하기 때문입니다. Flat terrain은 height scan이 없어 observation이 48D로 줄어들므로 [128,128,128]으로 충분합니다.

### Symmetry Augmentation

ANYmal-C는 좌우 대칭 구조를 가지므로, 학습 데이터를 좌우 반전하여 augmentation할 수 있습니다. `compute_symmetric_states` 함수가 observation과 action의 좌우 대칭 변환을 수행하며, 이를 통해 학습 효율을 약 2배 향상시킬 수 있습니다.

## Comparison: ANYmal-C vs H1 vs Go2

| 항목 | ANYmal-C | H1 | Go2 |
|---|---|---|---|
| 형태 | 사족 | 이족(humanoid) | 사족(소형) |
| 베이스 변경량 | 없음 (레퍼런스) | Reward 대폭 수정 | Terrain/reward 조정 |
| 추가 reward | — | termination_penalty, joint_deviation (3종) | — |
| 비활성 reward | — | lin_vel_z_l2, undesired_contacts | undesired_contacts |
| Max iterations | 1500 | 3000 | 1500 |
| Action scale | 0.5 | 0.5 | 0.25 |
| Push robot | 비활성 | 비활성 | 비활성 |
| 특수 처리 | Symmetry augmentation | Yaw-frame tracking, biped air time | 소형 terrain scale |

## Further Reading

- **공통 베이스 config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/velocity_env_cfg.py`
- **ANYmal-C config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/anymal_c/`
- **ANYmal-C 에셋**: `~/workspace/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/anymal.py`
- **MDP terms**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/mdp/`
- **Rough terrain config**: `~/workspace/IsaacLab/source/isaaclab/isaaclab/terrains/config/rough.py`
- **RSL-RL PPO config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/anymal_c/agents/rsl_rl_ppo_cfg.py`
