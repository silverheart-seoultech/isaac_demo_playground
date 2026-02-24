# Humanoid Locomotion

## Overview

21-DOF 이족 보행 Humanoid 환경. Ant와 동일한 `LocomotionEnv` 베이스를 사용하지만, 관절 수가 21개로 증가하고 관절별 gear ratio가 다르며, 종료 조건이 더 엄격하다(torso 높이 0.8m vs Ant의 0.31m). 이족 보행의 균형 유지가 4족보다 근본적으로 어려우므로, reward 가중치와 학습 설정이 상응하게 조정되어 있다.

## Architecture

Ant와 동일한 `LocomotionEnv` 상속 구조를 따른다. 차이점은 Config 수준에서 발생한다.

### Ant vs Humanoid 구조 비교

| 항목 | Ant | Humanoid |
|---|---|---|
| DOF | 8 | 21 |
| Observation | 36D | 75D |
| Action | 8D | 21D |
| Gear Ratios | 동일 (15×8) | 관절별 상이 (22.5~135) |
| 종료 높이 | < 0.31m | < 0.8m |
| 초기 높이 | 0.5m | 1.34m |

## Source Files

| 파일 | 역할 |
|---|---|
| `direct/humanoid/humanoid_env.py` | Humanoid Config (97줄) |
| `direct/locomotion/locomotion_env.py` | 보행 베이스 클래스 (공유) |
| `manager_based/classic/humanoid/humanoid_env_cfg.py` | Manager-Based Config (221줄) |
| `manager_based/classic/humanoid/mdp/observations.py` | 관측 함수 |
| `manager_based/classic/humanoid/mdp/rewards.py` | 보상 함수 |
| `isaaclab_assets/robots/humanoid.py` | 에셋 (21-DOF, 관절별 PD 게인) |

## 핵심 코드 분석

### Observation Space (75D)

```
[0:1]    torso_z
[1:4]    velocity_local
[4:7]    angular_velocity_local × 0.25    ← 스케일링 주목
[7:8]    yaw
[8:9]    roll
[9:10]   angle_to_target
[10:11]  up_proj
[11:12]  heading_proj
[12:33]  dof_pos_scaled (21 관절)
[33:54]  dof_vel × 0.1                    ← 스케일링 주목
[54:75]  actions (21D)
```

**스케일링 차이**: Humanoid에서는 `angular_velocity × 0.25`, `dof_vel × 0.1`로 스케일링한다. 이는 이족 보행 시 각속도와 관절 속도의 절대값이 4족보다 크기 때문에, 정책 네트워크의 입력 범위를 적절히 조절하여 학습 안정성을 높이기 위함이다. Ant에서는 이러한 스케일링이 적용되지 않는다.

### Reward 가중치 비교

| Term | Ant | Humanoid | 변경 이유 |
|---|---|---|---|
| alive | 0.5 | **2.0** | 이족 균형 유지가 어려워 생존 보상 증가 |
| action_cost | -0.005 | **-0.01** | 21 관절의 총 토크 제어 |
| energy | -0.05 | **-0.005** | 보행 자체가 많은 에너지를 소비하므로 완화 |
| joint_limits | -0.1 | **-0.25** | 관절 제한 위반이 전복으로 직결 |

`alive` 가중치가 0.5 → 2.0으로 4배 증가한 것은 이족 보행에서 넘어지지 않는 것 자체가 중요한 학습 목표임을 반영한다. 반면 `energy` 페널티는 0.05 → 0.005로 10배 감소했는데, 이족 보행은 본질적으로 에너지를 많이 소비하므로 에너지 페널티가 너무 크면 움직임 자체를 억제하게 된다.

### Joint Gear Ratios

```python
joint_gears = [
    67.5, 67.5, 67.5,   # 허리 (torso) — 상체 안정화
    67.5, 67.5, 67.5,   # 오른쪽 엉덩이
    67.5,                # 오른쪽 무릎
    45, 45, 45,          # 오른쪽 발목/발
    135,                 # 왼쪽 엉덩이 (abduction)
    45, 45,              # 왼쪽 무릎/발목
    135,                 # 오른쪽 엉덩이 (abduction)
    45,                  # 오른쪽 발목
    90, 90,              # 왼쪽/오른쪽 어깨
    22.5, 22.5,          # 왼쪽/오른쪽 팔꿈치
    22.5, 22.5,          # 왼쪽/오른쪽 손목
]
```

관절별 gear ratio는 해당 관절이 지탱하는 하중에 비례한다. Hip abduction(135)은 보행 중 측방 안정성을 담당하므로 가장 큰 토크가 필요하고, 팔꿈치/손목(22.5)은 보행에 직접 기여하지 않으므로 작은 토크면 충분하다.

### 로봇 에셋 — 관절별 PD 게인

```python
# isaaclab_assets/robots/humanoid.py (발췌)
actuators = {
    "body": ImplicitActuatorCfg(
        joint_names_expr=[".*"],
        stiffness={
            "abdomen_.*": 20.0,    # 허리: 높은 강성 (상체 안정)
            ".*_thigh_.*": 10.0,   # 허벅지: 중간
            ".*_shin": 5.0,        # 정강이: 낮음
            ".*_upper_arm": 10.0,  # 상완: 중간
            ".*_forearm": 2.0,     # 전완: 매우 낮음
            ".*_foot_.*": 2.0,     # 발: 매우 낮음
        },
        damping={...},  # 유사한 구조
    ),
}
```

**정규표현식 매칭**: `joint_names_expr`에 정규표현식을 사용하여 관절 그룹별로 게인을 설정한다. 허리(`abdomen`)의 stiffness가 20으로 가장 높은 것은 상체를 수직으로 유지하는 데 핵심적이기 때문이다.

### 학습 하이퍼파라미터

| 파라미터 | Ant | Humanoid | 변경 이유 |
|---|---|---|---|
| Network | [400,200,100] | [400,200,100] | 동일 |
| LR | 5e-4 | 5e-4 | 동일 |
| Iterations | 1000 | 1000 | 동일 |
| Obs Normalization | False | **True** | 75D obs 범위 차이 보상 |
| Value Loss Coef | 1.0 | **2.0** | Value function 정확도 향상 |

**Observation Normalization**: Humanoid에서만 활성화된다. 75D observation의 각 차원이 매우 다른 범위를 가지므로(높이 ~1m, 관절속도 ~10rad/s, projection ~1.0), running mean/std로 정규화하여 학습 안정성을 높인다.

## 실행 방법

```bash
cd ~/workspace/IsaacLab

# Direct
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Humanoid-Direct-v0 --num_envs 4096

# Manager-Based
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Humanoid-v0 --num_envs 4096

# 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Humanoid-Direct-v0 \
    --use_pretrained_checkpoint --num_envs 32
```

## Further Reading

- **Humanoid 소스**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/humanoid/humanoid_env.py`
- **Manager-Based**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/classic/humanoid/`
- **MDP rewards**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/classic/humanoid/mdp/rewards.py`
- **에셋**: `~/workspace/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/humanoid.py`
