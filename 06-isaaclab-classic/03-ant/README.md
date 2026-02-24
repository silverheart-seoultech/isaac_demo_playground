# Ant Locomotion

## Overview

8-DOF 4족 Ant 로봇의 보행 환경. CartPole보다 관절 수, observation 차원, reward 복잡도가 크게 증가하며, `LocomotionEnv` 베이스 클래스의 공통 보행 로직을 활용합니다. Direct와 Manager-Based 두 가지 구현이 모두 존재하며, 여기서는 Direct 구현을 중심으로 분석합니다.

## Architecture

### 클래스 계층

```
DirectRLEnv
    └── LocomotionEnv (locomotion_env.py, 280줄)
            │
            ├── _compute_intermediate_values()   ← heading/up projection, 로컬 속도
            ├── _get_observations()              ← 36D observation 벡터
            ├── _get_rewards()                   ← JIT-compiled, 7개 term
            ├── _get_dones()                     ← 높이 기반 종료
            └── _reset_idx()                     ← 균일 랜덤 초기화
                    │
                    └── AntEnv (ant_env.py, 75줄)
                            └── Config 오버라이드: 관절 수, 기어비, 에피소드 길이
```

`LocomotionEnv`는 Ant와 Humanoid가 공유하는 보행 환경 베이스 클래스다. Heading/up projection 계산, potential-based progress reward, 관절 제한 체크 등 보행 공통 로직을 구현합니다.

## Source Files

| 파일 | 역할 |
|---|---|
| `direct/ant/ant_env.py` | Ant 환경 Config (75줄) |
| `direct/locomotion/locomotion_env.py` | 보행 베이스 클래스 (280줄) |
| `manager_based/classic/ant/ant_env_cfg.py` | Manager-Based Ant Config (184줄) |
| `manager_based/classic/ant/agents/rsl_rl_ppo_cfg.py` | RSL-RL PPO Config |
| `isaaclab_assets/robots/ant.py` | Ant 에셋 (8-DOF + 4 foot) |

## 핵심 코드 분석

### Observation Space (36D)

```
[0:1]   torso_z (높이)
[1:4]   velocity_local (body frame 선속도)
[4:7]   angular_velocity_local (body frame 각속도)
[7:8]   yaw
[8:9]   roll
[9:10]  angle_to_target (목표 방향까지 각도)
[10:11] up_proj (수직 유지 정도, 1.0=완벽한 직립)
[11:12] heading_proj (목표 방향 정렬도)
[12:20] dof_pos_scaled (8 관절, 범위 정규화)
[20:28] dof_vel (8 관절 속도)
[28:36] actions (이전 action, 8D)
```

**설계 분석**: 36D 중 처음 12D는 로봇의 전역 상태(위치/자세/목표 방향), 나머지 24D는 관절 상태와 이전 action입니다. `angle_to_target`, `up_proj`, `heading_proj`는 intermediate value로 매 스텝 계산되며, 정책이 목표를 향해 이동하면서 자세를 유지하는 데 직접적인 정보를 제공합니다.

`dof_pos_scaled`는 관절 범위를 [-1, 1]로 정규화한 값입니다. 이는 서로 다른 관절의 범위 차이로 인한 학습 불균형을 방지합니다.

### Reward Decomposition (7 terms)

| Term | Weight | 수식 | 역할 |
|---|---|---|---|
| `progress` | +1.0 | potentials - prev_potentials | 목표까지 거리 감소 보상 |
| `alive` | +0.5 | 매 스텝 상수 | 생존 유지 |
| `upright` | +0.1 | up_proj > 0.93 보너스 | 자세 안정성 |
| `heading` | +0.5 | heading_proj > 0.8 보너스 | 목표 방향 정렬 |
| `action_cost` | -0.005 | Σ(action²) | 불필요한 토크 억제 |
| `energy` | -0.05 | Σ(\|action × vel × gear\|) | 에너지 효율 |
| `joint_limits` | -0.1 | 관절 한계 위반 개수 | 물리적 제약 준수 |

**Potential-Based Reward**: `progress = potentials - prev_potentials`는 목표까지의 거리가 줄어들 때 양수, 늘어날 때 음수를 반환합니다. 이 shaping은 sparse reward 문제를 해결하면서도 최적 정책을 보존합니다 (potential-based shaping theorem).

**Energy Cost**: `Σ(|action × dof_vel × motor_effort_ratio|)`는 관절에 가해진 토크와 관절 속도의 곱으로, 실제 소비 전력에 비례합니다. `motor_effort_ratio`(gear ratio)는 관절마다 다를 수 있어, 큰 토크가 필요한 관절의 에너지 소비를 더 크게 페널티합니다.

### Joint Gear Configuration

```python
joint_gears = [15, 15, 15, 15, 15, 15, 15, 15]  # 8 관절 모두 동일
action_scale = 0.5  # action [-1,1] → torque [-0.5, 0.5] × gear
```

Ant는 모든 관절의 gear ratio가 동일(15)하여 대칭적인 다리 구조를 반영합니다. Humanoid에서는 관절별로 다른 gear ratio를 사용하는데, 이는 관절 크기와 토크 요구사항이 다르기 때문입니다.

### 물리 설정

| 파라미터 | 값 | CartPole과 비교 |
|---|---|---|
| dt | 1/120 s | 동일 |
| decimation | 2 | 동일 |
| episode_length | 15 s | 3배 (5s → 15s) |
| action_dim | 8 | 8배 (1 → 8) |
| obs_dim | 36 | 9배 (4 → 36) |
| network | [400, 200, 100] | 훨씬 큼 ([32, 32]) |
| iterations | 1000 | 6.7배 (150) |

에피소드가 길고 네트워크가 큰 것은 보행이라는 태스크의 복잡도를 반영합니다. 다리 간 협응(coordination)을 학습하는 데 더 많은 파라미터와 경험이 필요합니다.

### 학습 하이퍼파라미터

| 파라미터 | 값 |
|---|---|
| Network | [400, 200, 100] |
| Learning Rate | 5e-4 |
| Num Steps | 32 |
| Max Iterations | 1000 |
| Entropy Coef | 0.0 |

## 실행 방법

```bash
cd ~/workspace/IsaacLab

# Direct 환경 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Ant-Direct-v0 \
    --num_envs 4096

# Manager-Based 환경 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Ant-v0 \
    --num_envs 4096

# 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Ant-Direct-v0 \
    --use_pretrained_checkpoint \
    --num_envs 32
```

## Further Reading

- **LocomotionEnv 소스**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/locomotion/locomotion_env.py`
- **Ant 에셋**: `~/workspace/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/ant.py`
- **Manager-Based Ant**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/classic/ant/`
