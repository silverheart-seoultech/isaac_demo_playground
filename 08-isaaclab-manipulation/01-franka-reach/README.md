# Franka Reach — Manager-Based Manipulation

## Overview

Franka Emika Panda 로봇으로 end-effector의 목표 pose를 추종하는 reaching 환경. Isaac Lab manipulation의 가장 기본적인 태스크로, 물체 접촉 없이 end-effector 위치/자세만 제어합니다. 4가지 action space variant(joint_pos, ik_abs, ik_rel, osc)를 제공하여 **같은 태스크를 다른 action representation으로 학습하는 비교 실험**이 가능합니다.

Reach 환경은 locomotion의 velocity tracking과 유사하게, 공통 베이스(`ReachEnvCfg`)를 로봇별/action type별로 상속하는 구조입니다.

## Architecture

### 상속 구조

```
ManagerBasedRLEnvCfg
    │
    └── ReachEnvCfg (reach_env_cfg.py)
            │   ├── ReachSceneCfg       ← ground + table + robot(MISSING) + light
            │   ├── CommandsCfg         ← UniformPoseCommand (ee_pose)
            │   ├── ObservationsCfg     ← 4개 observation term
            │   ├── RewardsCfg          ← position/orientation tracking + penalties
            │   ├── CurriculumCfg       ← action_rate/joint_vel weight 점진적 증가
            │   └── TerminationsCfg     ← time_out만
            │
            ├── FrankaReachEnvCfg (joint_pos_env_cfg.py)
            │       └── arm_action = JointPositionActionCfg
            │
            ├── FrankaReachIkAbsEnvCfg (ik_abs_env_cfg.py)
            │       └── arm_action = DifferentialIKActionCfg (absolute)
            │
            ├── FrankaReachIkRelEnvCfg (ik_rel_env_cfg.py)
            │       └── arm_action = DifferentialIKActionCfg (relative)
            │
            └── FrankaReachOscEnvCfg (osc_env_cfg.py)
                    └── arm_action = OperationalSpaceControllerActionCfg
```

### Action Space Variants

| Variant | Action Dim | 표현 | 특징 |
|---|---|---|---|
| `joint_pos` | 7 | 관절 목표 위치 (offset) | 가장 직접적, 학습 쉬움, IK 미사용 |
| `ik_abs` | 7 | ee 절대 pose (pos+quat) | IK로 관절 명령 변환, 정밀한 목표 설정 |
| `ik_rel` | 6 | ee 상대 delta (pos+rot) | 현재 위치 기준 증분, 미세 조정에 유리 |
| `osc` | 6 | ee wrench (force+torque) | 임피던스 제어, 접촉 태스크에 적합 |

## Source Files

| 파일 | 역할 |
|---|---|
| `reach_env_cfg.py` | 공통 베이스 config |
| `config/franka/joint_pos_env_cfg.py` | Joint position action variant |
| `config/franka/ik_abs_env_cfg.py` | Absolute IK variant |
| `config/franka/ik_rel_env_cfg.py` | Relative IK variant |
| `config/franka/osc_env_cfg.py` | OSC variant |
| `config/franka/__init__.py` | Gymnasium 등록 (8개 env ID) |
| `config/franka/agents/rsl_rl_ppo_cfg.py` | RSL-RL PPO 하이퍼파라미터 |
| `mdp/` | 공유 MDP term 함수들 |

## 핵심 코드 분석

### Observation Space (4 terms)

```python
class PolicyCfg(ObsGroup):
    joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
    joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
    pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "ee_pose"})
    actions = ObsTerm(func=mdp.last_action)
```

| Term | 차원 | 역할 |
|---|---|---|
| `joint_pos` | 9 | Franka 7 관절 + 2 finger (default 기준 상대값) |
| `joint_vel` | 9 | 관절 속도 |
| `pose_command` | 7 | 목표 ee pose (pos 3 + quat 4) |
| `actions` | 7~9 | 이전 action (variant에 따라 차원 상이) |

Locomotion 대비 observation이 매우 단순합니다(32~34D). Reaching은 로봇 자체의 상태와 목표만 알면 되므로 지형 정보나 접촉 센서가 불필요합니다.

### Reward Decomposition

```python
class RewardsCfg:
    end_effector_position_tracking           weight=-0.2   # L2 position error
    end_effector_position_tracking_fine      weight=0.1    # tanh(pos_error, std=0.1)
    end_effector_orientation_tracking        weight=-0.1   # orientation error

    action_rate                              weight=-0.0001
    joint_vel                                weight=-0.0001
```

| Reward Term | Weight | 수식 | 설계 의도 |
|---|---|---|---|
| `position_tracking` | -0.2 | -\|p_target - p_ee\| | 위치 오차 줄이기 (L2, coarse) |
| `position_tracking_fine` | +0.1 | tanh(\|p_target - p_ee\| / 0.1) | 목표 근처에서 fine-grained 보상 |
| `orientation_tracking` | -0.1 | -\|q_target - q_ee\| | 자세 오차 줄이기 |
| `action_rate` | -0.0001 | -\|a_t - a_{t-1}\|² | action jitter 억제 |
| `joint_vel` | -0.0001 | -Σ v_i² | 관절 속도 억제 |

**Coarse + Fine 이중 추종 전략**: `position_tracking(-0.2)`은 L2 거리로 넓은 범위에서 끌어당기고, `position_tracking_fine(+0.1)`는 tanh로 목표 0.1m 이내에서 sharply rewarding합니다. 이 조합은 빠른 접근 + 정밀한 도달을 모두 학습하게 합니다.

### Curriculum — Penalty Weight 점진적 증가

```python
class CurriculumCfg:
    action_rate = CurrTerm(func=mdp.modify_reward_weight,
        params={"term_name": "action_rate", "weight": -0.005, "num_steps": 4500})
    joint_vel = CurrTerm(func=mdp.modify_reward_weight,
        params={"term_name": "joint_vel", "weight": -0.001, "num_steps": 4500})
```

`action_rate`의 가중치가 -0.0001에서 4500 스텝에 걸쳐 -0.005로 50배 증가합니다. 학습 초기에는 penalty 없이 자유롭게 탐색하다가, 수렴 단계에서 부드러운 움직임을 강제합니다. 이 curriculum 패턴은 "먼저 해결법을 찾고, 그 다음 세련되게 만듭니다"는 전략입니다.

### Pose Command 생성

```python
ee_pose = mdp.UniformPoseCommandCfg(
    body_name="panda_hand",
    resampling_time_range=(4.0, 4.0),   # 4초마다 새 목표
    ranges=mdp.UniformPoseCommandCfg.Ranges(
        pos_x=(0.35, 0.65),              # 테이블 위 x 범위
        pos_y=(-0.2, 0.2),               # 좌우
        pos_z=(0.15, 0.5),               # 높이 (테이블 위)
        roll=(0.0, 0.0), pitch=(π, π),   # 아래를 향한 고정 자세
        yaw=(-π, π),                      # yaw만 자유
    ),
)
```

`pitch=(π, π)`는 end-effector가 항상 아래를 향하도록 고정합니다 (Franka의 z축이 panda_hand에서 아래 방향). `yaw`만 자유도를 주어 도달 가능한 자세 공간을 제한합니다.

### 물리 설정

| 파라미터 | 값 | 의미 |
|---|---|---|
| `dt` | 1/60 s | 60Hz 물리 시뮬레이션 |
| `decimation` | 2 | 물리 2스텝당 1 제어 |
| 제어 주파수 | 30 Hz | dt × decimation = 1/30 |
| `episode_length_s` | 12.0 s | 최대 360 제어 스텝 |
| `action_scale` | 0.5 | joint_pos variant |
| `num_envs` | 4096 | |

Locomotion(200Hz, decimation 4)보다 낮은 물리 주파수입니다. Reaching은 빠른 동적 제어가 필요 없으므로 60Hz로 충분합니다.

## 실행 방법

```bash
cd ~/workspace/IsaacLab

# Joint Position variant 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Reach-Franka-v0 \
    --num_envs 4096

# IK Absolute variant 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Reach-Franka-IK-Abs-v0

# IK Relative variant 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Reach-Franka-IK-Rel-v0

# OSC variant 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Reach-Franka-OSC-v0

# 사전학습 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Reach-Franka-Play-v0 \
    --use_pretrained_checkpoint

# Teleoperation (키보드로 직접 ee 제어)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Reach-Franka-IK-Rel-v0 \
    --teleop_device keyboard
```

### 학습 하이퍼파라미터 (RSL-RL PPO)

| 파라미터 | 값 |
|---|---|
| Network | [64, 64] |
| Activation | ELU |
| Max Iterations | 1000 |
| Num Steps/Env | 24 |
| Learning Rate | 1e-3 (adaptive) |
| Entropy Coef | 0.001 |
| Num Learning Epochs | 8 |
| Discount (γ) | 0.99 |

네트워크가 [64,64]로 locomotion([512,256,128])보다 훨씬 작습니다. Reaching은 7D 관절 → 6D pose의 단순한 매핑이므로 작은 네트워크로 충분합니다.

## Comparison: Action Space Variants

같은 reaching 태스크를 4가지 action representation으로 학습할 수 있습니다:

| | Joint Position | IK Absolute | IK Relative | OSC |
|---|---|---|---|---|
| **Action 차원** | 7 | 7 (pos+quat) | 6 (delta pos+rot) | 6 (force+torque) |
| **제어 레벨** | 관절 공간 | Task 공간 (절대) | Task 공간 (상대) | Task 공간 (힘) |
| **IK 사용** | 없음 | Differential IK | Differential IK | OSC |
| **학습 난이도** | 쉬움 | 중간 | 중간 | 어려움 |
| **장점** | 직관적, 빠른 수렴 | 정밀한 목표 설정 | 증분 제어, 미세 조정 | 접촉 제어 가능 |
| **적합한 태스크** | 기본 reaching | waypoint following | 정밀 조립 | 접촉/힘 제어 |

이 비교는 RL에서 action representation 선택이 학습 효율과 태스크 성능에 미치는 영향을 이해하는 데 핵심적입니다.

## Further Reading

- **Reach 베이스 config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/reach_env_cfg.py`
- **Franka Joint Pos**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/config/franka/joint_pos_env_cfg.py`
- **Franka 에셋**: `isaaclab_assets.FRANKA_PANDA_CFG`
- **RSL-RL config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/config/franka/agents/rsl_rl_ppo_cfg.py`
- **Differential IK**: `isaaclab.controllers.DifferentialIKController`
- **OSC**: `isaaclab.controllers.OperationalSpaceController`
