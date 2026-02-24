# Shadow Hand In-Hand Manipulation — Direct RL

## Overview

Shadow Dexterous Hand로 큐브를 손 안에서 목표 orientation으로 회전시키는 in-hand manipulation 환경. Isaac Lab 전체에서 가장 복잡한 Direct RL 환경 중 하나로, 157D observation, 20D action, 광범위한 domain randomization을 특징으로 합니다.

Direct RL 패러다임으로 구현되어 있으며, OpenAI 스타일의 asymmetric observation variant도 제공합니다. Domain randomization의 범위와 세밀도에서 sim-to-real transfer를 위한 RL 환경 설계의 최전선을 보여줍니다.

## Architecture

### 환경 구조

```
DirectRLEnv
    │
    └── ShadowHandEnv (shadow_hand_env.py)
            │
            ├── _setup_scene()
            │   ├── robot = SHADOW_HAND_CFG (24 DOF, 20 actuated)
            │   ├── object = DexCube
            │   └── goal_object (visualization only)
            │
            ├── _pre_physics_step()  ← action → joint target
            ├── _get_observations()  ← 157D full / 42D openai
            ├── _get_rewards()       ← rotation tracking + penalties
            ├── _get_dones()         ← fall detection + consecutive success
            └── _reset_idx()         ← hand/object 재초기화
```

### Two Observation Modes

| Mode | Obs Dim | State Dim | 특징 |
|---|---|---|---|
| `full` | 157 | 0 | 모든 정보 단일 observation |
| `openai` | 42 | 187 | Asymmetric: 정책은 42D, critic은 187D |

OpenAI mode는 실제 로봇에서 얻을 수 있는 정보만 policy에 제공하고, 시뮬레이션에서만 얻을 수 있는 정보(물체의 정확한 상태 등)는 critic에게만 제공하는 asymmetric actor-critic 구조입니다.

## Source Files

| 파일 | 역할 |
|---|---|
| `shadow_hand_env.py` | 환경 전체 구현 (Direct RL) |
| `shadow_hand_env_cfg.py` | 환경 설정 (물리, reward, randomization) |
| `__init__.py` | Gymnasium 등록 |
| `agents/rsl_rl_ppo_cfg.py` | RSL-RL PPO 하이퍼파라미터 |
| `agents/rl_games_ppo_cfg.yaml` | RL Games PPO config |

## 핵심 코드 분석

### Observation Space (157D Full Mode)

Full mode의 157D observation은 아래 요소들로 구성됩니다:

| Component | 차원 | 설명 |
|---|---|---|
| 손 관절 위치 | 24 | 모든 관절 (actuated + unactuated) |
| 손 관절 속도 | 24 | × vel_obs_scale (0.2) |
| 물체 위치 | 3 | 손바닥 기준 상대 좌표 |
| 물체 orientation | 4 | quaternion |
| 물체 선속도 | 3 | × vel_obs_scale |
| 물체 각속도 | 3 | × vel_obs_scale |
| 목표 orientation | 4 | quaternion |
| orientation 차이 | 4 | target - current |
| 이전 action | 20 | |
| 손끝 위치 (5개) | 15 | 5 fingertip × 3D |
| 손끝-물체 거리 (5개) | 15 | 5 fingertip to object |
| 물체-목표 거리 | 3 | position error |
| 물체-손바닥 거리 | 3 | height from palm |
| Force/torque (5 fingers) | 30 | 5 fingers × 6D wrench |
| 물체 힘/토크 | 6 | 물체에 작용하는 wrench |

이 정보량은 Franka Reach(~34D)의 약 5배입니다. Dexterous manipulation은 손가락 간 협응, 접촉력 제어, 물체 상태 추정이 모두 필요하므로 관측 공간이 큽니다.

### Action Space (20D)

```python
actuated_joint_names = [
    "robot0_WRJ1", "robot0_WRJ0",           # 손목 2 DOF
    "robot0_FFJ3", "robot0_FFJ2", "robot0_FFJ1",  # 검지 3 DOF
    "robot0_MFJ3", "robot0_MFJ2", "robot0_MFJ1",  # 중지 3 DOF
    "robot0_RFJ3", "robot0_RFJ2", "robot0_RFJ1",  # 약지 3 DOF
    "robot0_LFJ4", "robot0_LFJ3", "robot0_LFJ2", "robot0_LFJ1",  # 소지 4 DOF
    "robot0_THJ4", "robot0_THJ3", "robot0_THJ2", "robot0_THJ1", "robot0_THJ0",  # 엄지 5 DOF
]
```

총 24 DOF 중 20개가 actuated. 나머지 4개(FFJ0, MFJ0, RFJ0, LFJ0)는 tendon coupling으로 인접 관절에 종속됩니다. 엄지가 5 DOF로 가장 자유도가 높은데, 이는 in-hand manipulation에서 엄지가 핵심 역할을 하기 때문입니다.

### Reward Structure

```python
# reward scales (환경 config)
dist_reward_scale = -10.0      # 물체-목표 거리 페널티
rot_reward_scale = 1.0         # rotation 추종 보상
rot_eps = 0.1                  # rotation reward smoothing
action_penalty_scale = -0.0002 # action magnitude 페널티
reach_goal_bonus = 250         # 목표 도달 시 큰 보너스
fall_penalty = 0               # full mode: 낙하 페널티 없음 (openai: -50)
fall_dist = 0.24               # 낙하 판정 거리
success_tolerance = 0.1        # 목표 도달 판정 (rad)
```

| Term | Scale | 설계 의도 |
|---|---|---|
| rotation tracking | +1.0 | 큐브 회전 각도를 목표에 맞추기 |
| distance penalty | -10.0 | 물체가 손에서 멀어지면 큰 페널티 |
| reach_goal_bonus | +250 | 목표 도달 시 매우 큰 보상 (sparse) |
| action_penalty | -0.0002 | 과도한 관절 움직임 억제 |
| fall_penalty | 0 / -50 | OpenAI mode에서만 활성 |

**reach_goal_bonus(250)**가 다른 모든 reward를 압도하는 sparse reward다. 정책은 연속적인 rotation tracking + distance penalty로 점진적으로 학습하다가, 목표 도달 시 큰 bonus를 받아 성공 행동을 강화합니다.

### Domain Randomization — 가장 광범위한 설정

Shadow Hand의 domain randomization은 Isaac Lab 전체에서 가장 포괄적이다:

```python
# 로봇 물리 파라미터
robot_physics_material:     static_friction (0.7, 1.3)     # 250 buckets
robot_joint_stiffness:      scale (0.75, 1.5) log_uniform  # 관절 강성
robot_joint_damping:        scale (0.3, 3.0) log_uniform   # 관절 감쇠
robot_joint_pos_limits:     add ±0.01 gaussian             # 관절 한계
robot_tendon_properties:    stiffness/damping scale         # 건 특성

# 물체 물리 파라미터
object_physics_material:    static_friction (0.7, 1.3)
object_scale_mass:          scale (0.5, 1.5) uniform       # 질량 50~150%

# 환경
reset_gravity:              z축 ±0.4 gaussian, 36초 간격    # 중력 변동
```

| Randomization | 분포 | 범위 | 적용 시점 |
|---|---|---|---|
| 마찰 계수 | Uniform | 0.7~1.3 | Reset (720스텝 간격) |
| 관절 강성 | **Log-uniform** | 0.75~1.5× | Reset |
| 관절 감쇠 | **Log-uniform** | 0.3~3.0× | Reset |
| 관절 한계 | Gaussian | ±0.01 rad | Reset |
| 건(tendon) 특성 | Log-uniform | 0.75~1.5× / 0.3~3.0× | Reset |
| 물체 질량 | Uniform | 0.5~1.5× | Reset |
| 중력 | Gaussian | z축 ±0.4 | 36초 간격 |

**Log-uniform 분포**: 관절 강성과 감쇠에 log-uniform을 사용하는 이유는 물리 파라미터가 로그 스케일에서 더 균일하게 분포하기 때문입니다. 예를 들어, 강성이 1→2로 바뀌는 것과 0.5→1로 바뀌는 것은 동역학에 동일한 영향을 미칩니다.

**min_step_count_between_reset=720**: 랜덤화가 720 물리 스텝(= 약 3초)마다만 적용되어, 한 에피소드 내에서 급격한 파라미터 변화를 방지합니다.

### OpenAI Variant — Asymmetric Actor-Critic

```python
class ShadowHandOpenAIEnvCfg(ShadowHandEnvCfg):
    observation_space = 42    # policy sees limited info
    state_space = 187         # critic sees everything
    asymmetric_obs = True
    obs_type = "openai"

    fall_penalty = -50              # 낙하 페널티 활성
    success_tolerance = 0.4         # 더 관대한 성공 기준
    max_consecutive_success = 50    # 50회 연속 성공 시 종료
    act_moving_average = 0.3        # action smoothing

    # Action + observation noise
    action_noise = GaussianNoise(std=0.05) + Bias(std=0.015)
    observation_noise = GaussianNoise(std=0.002) + Bias(std=0.0001)
```

OpenAI variant는 sim-to-real을 위한 설정이다:
1. **Asymmetric obs**: 정책은 42D(실제 센서로 얻을 수 있는 정보), critic은 187D(시뮬레이션 특권 정보)
2. **Action/obs noise**: Gaussian noise + additive bias로 센서/액추에이터 불확실성 모델링
3. **Action smoothing**: `act_moving_average=0.3`으로 급격한 action 변화 방지
4. **Fall penalty(-50)**: 물체 낙하 시 강한 페널티
5. **연속 성공 기준**: 50회 연속 목표 도달 시 에피소드 성공 종료

### 물리 설정

| 파라미터 | Full | OpenAI |
|---|---|---|
| `dt` | 1/120 s | **1/60 s** |
| `decimation` | 2 | **3** |
| 제어 주파수 | 60 Hz | 20 Hz |
| `episode_length_s` | 10.0 s | 8.0 s |
| `num_envs` | **8192** | 8192 |

OpenAI variant는 더 낮은 제어 주파수(20Hz)를 사용합니다. 실제 로봇의 센서 업데이트 주기에 맞추기 위함입니다. 환경 수가 8192로 locomotion(4096)의 2배인 것은 dexterous manipulation의 높은 variance를 줄이기 위한 것입니다.

## 실행 방법

```bash
cd ~/workspace/IsaacLab

# Full observation 학습 (RSL-RL)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Shadow-Hand-Direct-v0 \
    --num_envs 8192

# OpenAI asymmetric 학습 (RL Games 권장)
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py \
    --task Isaac-Shadow-Hand-OpenAI-Direct-v0 \
    --num_envs 8192

# 사전학습 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Shadow-Hand-Direct-v0 \
    --use_pretrained_checkpoint

# Headless 학습 (GPU 메모리 절약)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Shadow-Hand-Direct-v0 \
    --num_envs 8192 --headless
```

### 학습 시 주의사항

- Shadow Hand는 GPU 메모리를 많이 사용합니다. 8192 환경 기준 약 8GB+ VRAM 필요.
- Full mode는 비교적 빠르게 수렴(~5000 iter), OpenAI mode는 noise/asymmetric obs 때문에 더 오래 걸림(~10000 iter).
- RL Games의 PPO 구현이 이 환경에서 RSL-RL보다 안정적인 경향이 있습니다.

## Comparison: Franka Reach/Lift vs Shadow Hand

| | Franka Reach | Franka Lift | Shadow Hand |
|---|---|---|---|
| **패러다임** | Manager-Based | Manager-Based | **Direct** |
| **Obs 차원** | ~34D | ~40D | **157D** |
| **Action 차원** | 7 | 9 (7+2 gripper) | **20** |
| **Reward** | ee tracking | 단계적 (reach→lift) | rotation + sparse bonus |
| **Domain Rand.** | Joint reset만 | + object position | **7종 물리 파라미터** |
| **Num Envs** | 4096 | 4096 | **8192** |
| **학습 복잡도** | 낮음 (~1000 iter) | 중간 | **높음 (~10000 iter)** |

Shadow Hand가 Direct RL인 이유: 157D observation, 20D action, 복잡한 reward 로직, 광범위한 domain randomization을 Manager-Based의 선언적 config로 표현하기 어렵습니다. Direct 방식이 이런 복잡한 환경에서 더 직관적이고 유연합니다.

## Further Reading

- **Shadow Hand env**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/shadow_hand/shadow_hand_env.py`
- **Shadow Hand config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/shadow_hand/shadow_hand_env_cfg.py`
- **Shadow Hand 에셋**: `isaaclab_assets.robots.shadow_hand.SHADOW_HAND_CFG`
- **RSL-RL PPO config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/shadow_hand/agents/`
- **OpenAI Shadow Hand Paper**: "Learning Dexterous In-Hand Manipulation" (OpenAI, 2019)
