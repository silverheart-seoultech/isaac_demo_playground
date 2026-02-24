# Unitree Go2 Velocity Tracking — Manager-Based Locomotion
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

Unitree Go2 소형 사족보행 로봇의 velocity tracking 환경. 공통 베이스(`LocomotionVelocityRoughEnvCfg`)를 상속하되, Go2의 작은 체격에 맞게 terrain 스케일, action scale, reward 가중치를 조정합니다. ANYmal-C(레퍼런스)와 같은 사족보행이지만, 로봇 크기 차이가 환경 설계에 미치는 영향을 구체적으로 보여주는 사례입니다.

## Architecture

### 상속 구조

```
LocomotionVelocityRoughEnvCfg
    │
    └── UnitreeGo2RoughEnvCfg (rough_env_cfg.py)
            │
            ├── scene.robot = UNITREE_GO2_CFG
            ├── scene.height_scanner → base (유지)
            │
            ├── terrain 스케일 축소
            │   ├── boxes: grid_height_range (0.025, 0.1)  ← 낮은 장애물
            │   └── random_rough: noise (0.01, 0.06)       ← 작은 노이즈
            │
            ├── actions.joint_pos.scale = 0.25             ← 0.5 → 0.25
            │
            ├── events 수정
            │   ├── push_robot = None
            │   ├── base_com = None
            │   ├── add_base_mass: (-1.0, 3.0) kg         ← (-5.0, 5.0) 축소
            │   └── reset: default 자세, 정지 시작
            │
            └── rewards 수정
                ├── track_lin_vel_xy_exp.weight = 1.5      ← 1.0 → 1.5
                ├── track_ang_vel_z_exp.weight = 0.75      ← 0.5 → 0.75
                ├── feet_air_time.weight = 0.01            ← 0.125 → 0.01
                ├── dof_torques_l2.weight = -0.0002        ← -1e-5 → 큰 폭 증가
                └── undesired_contacts = None
```

### ANYmal-C와의 핵심 차이

| 항목 | ANYmal-C (~50kg) | Go2 (~15kg) | 변경 근거 |
|---|---|---|---|
| Action scale | 0.5 | **0.25** | 작은 관절 → 같은 각도 변위에도 상대적으로 큰 토크 필요 |
| Terrain height | 기본값 | **축소** | 몸체 높이 대비 장애물 크기를 비례적으로 맞춤 |
| Mass randomization | ±5.0 kg | **-1.0~+3.0 kg** | 15kg 로봇에 5kg 추가는 33% 변화로 과도 |
| Torque penalty | -1e-5 | **-0.0002** | 20배 증가 — 소형 모터의 토크 효율 중요 |
| Velocity tracking | 1.0 / 0.5 | **1.5 / 0.75** | 추종 reward 강화로 빠른 수렴 유도 |
| Feet air time | 0.125 | **0.01** | gait 유도 약화 — Go2는 자연스럽게 trot 발견 |

## Source Files

| 파일 | 역할 |
|---|---|
| `velocity_env_cfg.py` | 공통 베이스 config |
| `config/go2/rough_env_cfg.py` | Go2 rough terrain (terrain/reward/event 조정) |
| `config/go2/flat_env_cfg.py` | Go2 flat terrain |
| `config/go2/__init__.py` | Gymnasium 등록 (4개 env ID) |
| `config/go2/agents/rsl_rl_ppo_cfg.py` | RSL-RL PPO 하이퍼파라미터 |
| `config/go2/agents/skrl_*_ppo_cfg.yaml` | SKRL PPO config |

## 핵심 코드 분석

### Terrain Scale 조정

```python
# 장애물 높이 축소
self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)
# 불규칙 지형 노이즈 축소
self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01
```

Go2의 다리 길이는 ANYmal-C의 약 절반입니다. 같은 높이의 장애물이라도 Go2에게는 상대적으로 2배 어려운 셈입니다. boxes의 최대 높이를 0.1m으로 제한하고, random_rough의 노이즈를 0.01–0.06m으로 줄여 체격에 비례한 난이도를 유지합니다.

### Action Scale 축소

```python
self.actions.joint_pos.scale = 0.25  # 기본 0.5에서 절반
```

Action [-1, 1]이 관절 offset [-0.25, +0.25] rad으로 매핑됩니다. Go2의 관절 가동 범위가 ANYmal-C보다 작고, 작은 로봇에서 같은 각도 변화가 상대적으로 더 큰 동작을 만들기 때문에 스케일을 절반으로 줄입니다. 이는 정책의 action 출력이 더 세밀한 제어를 할 수 있게 합니다.

### Reward 가중치 재조정

```python
# 추종 reward 강화
self.rewards.track_lin_vel_xy_exp.weight = 1.5    # 1.0 → 1.5
self.rewards.track_ang_vel_z_exp.weight = 0.75    # 0.5 → 0.75

# 토크 페널티 대폭 증가
self.rewards.dof_torques_l2.weight = -0.0002      # -1e-5 → -0.0002 (20배)

# gait 유도 약화
self.rewards.feet_air_time.weight = 0.01           # 0.125 → 0.01
self.rewards.feet_air_time.params["sensor_cfg"].body_names = ".*_foot"

# 비활성
self.rewards.undesired_contacts = None
```

| Reward Term | ANYmal-C | Go2 | 변경 근거 |
|---|---|---|---|
| `track_lin_vel_xy_exp` | 1.0 | **1.5** | 추종 정확도 중시 — Go2의 주요 용도 |
| `track_ang_vel_z_exp` | 0.5 | **0.75** | 비례적으로 상향 |
| `dof_torques_l2` | -1e-5 | **-2e-4** | 소형 모터의 에너지 효율이 배터리 수명에 직결 |
| `feet_air_time` | 0.125 | **0.01** | Go2는 자연스럽게 trotting gait을 학습하므로 약한 유도로 충분 |
| `dof_acc_l2` | -2.5e-7 | -2.5e-7 | 동일 유지 |

**핵심 인사이트**: Go2의 `dof_torques_l2` 가중치가 20배로 증가한 것은 단순한 크기 비례가 아니다. 소형 로봇의 모터는 효율 곡선이 다르며, 배터리 용량이 제한적이므로 에너지 절약이 실환경에서 더 중요합니다.

### Domain Randomization 조정

```python
# 질량 범위 축소
self.events.add_base_mass.params["mass_distribution_params"] = (-1.0, 3.0)  # (-5.0, 5.0)에서

# 비활성
self.events.push_robot = None       # 소형 로봇은 push에 취약
self.events.base_com = None         # CoM 변화 비활성

# 초기 상태
self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)  # default 자세
self.events.reset_base.params["velocity_range"] = all zeros            # 정지 시작
```

Go2의 체중이 ~15kg이므로 ±5kg 질량 변화는 33%~67%에 해당합니다 (ANYmal-C의 ~50kg에서는 10%). 비대칭 범위 (-1.0, +3.0)은 배낭 등 적재 시나리오를 반영합니다(무게 추가가 일반적).

### Termination 조건

```python
self.terminations.base_contact.params["sensor_cfg"].body_names = "base"
```

ANYmal-C의 `"base"` body와 동일하게 base 접촉을 감지합니다. H1이 `"torso_link"`로 변경하는 것과 달리 Go2는 사족보행이므로 base 접촉이 곧 전도를 의미합니다.

## 실행 방법

```bash
cd ~/workspace/IsaacLab

# Rough terrain 학습 (RSL-RL)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Rough-Unitree-Go2-v0 \
    --num_envs 4096

# Flat terrain 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 \
    --num_envs 4096

# 사전학습 체크포인트 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Rough-Unitree-Go2-Play-v0 \
    --use_pretrained_checkpoint

# Headless 학습 (서버)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Rough-Unitree-Go2-v0 \
    --num_envs 4096 --headless
```

### 등록된 Environment IDs

| ID | Terrain | 용도 |
|---|---|---|
| `Isaac-Velocity-Rough-Unitree-Go2-v0` | Rough | 학습 |
| `Isaac-Velocity-Rough-Unitree-Go2-Play-v0` | Rough | 평가 (50 envs) |
| `Isaac-Velocity-Flat-Unitree-Go2-v0` | Flat | 학습 |
| `Isaac-Velocity-Flat-Unitree-Go2-Play-v0` | Flat | 평가 |

### 학습 하이퍼파라미터 (RSL-RL PPO)

| 파라미터 | Rough | Flat |
|---|---|---|
| Network | [512, 256, 128] | [128, 128, 128] |
| Activation | ELU | ELU |
| Max Iterations | 1500 | 300 |
| Num Steps/Env | 24 | 24 |
| Learning Rate | 1e-3 (adaptive) | 1e-3 (adaptive) |
| Entropy Coef | 0.01 | 0.01 |
| Discount (γ) | 0.99 | 0.99 |
| GAE (λ) | 0.95 | 0.95 |

ANYmal-C와 동일한 네트워크 구조/학습 스케줄을 사용합니다. 같은 사족보행 구조이므로 정책의 복잡도가 유사합니다. Entropy coef만 0.01로 ANYmal-C(0.005)보다 높은데, 이는 Go2의 다른 reward 가중치 세팅에서 더 넓은 탐색이 필요하기 때문입니다.

## Comparison: 로봇 크기가 환경 설계에 미치는 영향

Go2 config는 "로봇 크기 스케일링" 패턴을 명확히 보여줍니다:

```
로봇 특성                     → Config 조정
─────────────────────────────────────────────
작은 체격 (다리 짧음)        → terrain height/noise 축소
작은 관절                     → action_scale 0.5 → 0.25
낮은 체중 (~15kg)            → mass randomization 범위 축소, 비대칭
소형 모터                     → dof_torques_l2 가중치 20배 증가
빠른 자연 gait 발견          → feet_air_time 가중치 대폭 감소
```

이 패턴은 새로운 소형/대형 로봇을 추가할 때 참고할 수 있습니다. 핵심 원칙은 "환경 파라미터를 로봇 체격에 비례하여 스케일링"하되, 단순 비례가 아닌 물리적 특성(모터 효율, gait 특성)을 고려하는 것입니다.

## Further Reading

- **Go2 config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/rough_env_cfg.py`
- **Go2 에셋**: `isaaclab_assets.robots.unitree.UNITREE_GO2_CFG`
- **공통 베이스 config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/velocity_env_cfg.py`
- **RSL-RL PPO config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/agents/rsl_rl_ppo_cfg.py`
- **ANYmal-C 비교**: `../01-anymal-c-velocity/README.md`
- **H1 비교**: `../02-h1-locomotion/README.md`
