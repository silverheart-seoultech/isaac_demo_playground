# CartPole — Manager-Based RL Environment

## Overview

Isaac Lab의 Manager-Based 패러다임으로 구현된 CartPole 환경. Direct 방식(`01-cartpole-direct/`)과 동일한 CartPole 문제를 선언적 Config 클래스로 정의합니다. MDP의 각 요소(observation, action, reward, termination, event)를 독립적인 Manager에 위임하여 모듈성과 재사용성을 극대화합니다.

## Architecture

### Manager 구조

```
ManagerBasedRLEnvCfg
    │
    ├── SceneCfg
    │   ├── robot: ArticulationCfg      ← CARTPOLE_CFG
    │   └── ground: GroundPlaneCfg
    │
    ├── ObservationsCfg
    │   └── PolicyCfg
    │       ├── joint_pos_rel           ← ObservationTermCfg
    │       └── joint_vel_rel           ← ObservationTermCfg
    │
    ├── ActionsCfg
    │   └── joint_effort               ← JointEffortActionCfg (scale=100.0)
    │
    ├── RewardsCfg
    │   ├── alive (+1.0)               ← RewardTermCfg
    │   ├── terminating (-2.0)
    │   ├── pole_pos (-1.0)
    │   ├── cart_vel (-0.01)
    │   └── pole_vel (-0.005)
    │
    ├── TerminationsCfg
    │   ├── time_out                   ← DoneTerm
    │   └── cart_pos_limit             ← DoneTerm
    │
    └── EventsCfg
        ├── reset_cart_position        ← EventTermCfg (uniform [-1, 1])
        └── reset_pole_position        ← EventTermCfg (uniform [-π/4, π/4])
```

### Direct와의 근본적 차이

Direct 방식에서는 `_get_observations()`, `_get_rewards()` 등을 개발자가 직접 구현합니다. Manager-Based에서는 이들을 **선언적 Config**로 정의하면, 각 Manager가 런타임에 해당 함수를 조립합니다.

```python
# Direct: 명시적 구현
def _get_observations(self):
    return {"policy": torch.cat([pole_pos, pole_vel, cart_pos, cart_vel], dim=-1)}

# Manager-Based: 선언적 정의
class ObservationsCfg:
    class PolicyCfg(ObsGroup):
        joint_pos_rel = ObservationTermCfg(func=mdp.joint_pos_rel)
        joint_vel_rel = ObservationTermCfg(func=mdp.joint_vel_rel)
```

## Source Files

| 파일 | 역할 |
|---|---|
| `manager_based/classic/cartpole/cartpole_env_cfg.py` | 환경 전체 Config (181줄) |
| `manager_based/classic/cartpole/mdp/rewards.py` | Reward 함수 정의 |
| `manager_based/classic/cartpole/__init__.py` | Gymnasium 등록: `Isaac-Cartpole-v0` |
| `manager_based/classic/cartpole/agents/rsl_rl_ppo_cfg.py` | RSL-RL PPO 하이퍼파라미터 |

## 핵심 코드 분석

### Observation Config

```python
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos_rel = ObservationTermCfg(func=mdp.joint_pos_rel)
        joint_vel_rel = ObservationTermCfg(func=mdp.joint_vel_rel)
        enable_corruption = False
```

`joint_pos_rel`은 기본 위치 대비 상대적 관절 각도를 반환하고, `joint_vel_rel`은 기본 속도 대비 상대적 관절 속도를 반환합니다. Direct 버전이 raw 절대값을 사용하는 반면, Manager-Based 버전은 상대값을 사용합니다는 차이가 있습니다. 결과적으로 observation 공간의 의미가 약간 다르지만, 정책 학습에는 큰 영향이 없습니다.

`enable_corruption = False`로 observation noise를 비활성화했습니다. 이를 `True`로 바꾸고 각 term에 `noise` 파라미터를 추가하면 sim-to-real transfer를 위한 도메인 랜덤화가 가능합니다.

### Reward Config

```python
class RewardsCfg:
    alive = RewardTermCfg(func=mdp.is_alive, weight=1.0)
    terminating = RewardTermCfg(func=mdp.is_terminated, weight=-2.0)
    pole_pos = RewardTermCfg(
        func=mdp.joint_pos_target_l2,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
                "target": 0.0},
    )
    cart_vel = RewardTermCfg(
        func=mdp.joint_vel_l1,
        weight=-0.01,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"])},
    )
    pole_vel = RewardTermCfg(
        func=mdp.joint_vel_l1,
        weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"])},
    )
```

**핵심 패턴**: 각 reward term은 `(func, weight, params)` 3-tuple로 정의됩니다. `func`은 재사용 가능한 MDP 함수(예: `joint_vel_l1`은 어떤 로봇의 어떤 관절에도 적용 가능)이고, `params`에서 대상 관절을 지정합니다.

이 선언적 구조의 장점:
1. **Reward term 추가/제거**가 Config 변경만으로 가능 (코드 수정 불필요)
2. **weight 튜닝**이 Config에서 직접 가능
3. `mdp.joint_vel_l1` 같은 함수를 Ant, Humanoid 등 다른 환경에서도 재사용

### Event (Reset) Config

```python
class EventsCfg:
    reset_cart_position = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
            "position_range": (-1.0, 1.0),
            "velocity_range": (0.0, 0.0),
        },
    )
    reset_pole_position = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
            "position_range": (-0.25 * math.pi, 0.25 * math.pi),
            "velocity_range": (0.0, 0.0),
        },
    )
```

`mode="reset"`은 에피소드 시작 시에만 실행됨을 의미합니다. `mode="interval"`로 바꾸면 학습 중 주기적으로 실행되어 도메인 랜덤화에 활용할 수 있습니다.

### Termination Config

```python
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    cart_pos_limit = DoneTerm(
        func=mdp.joint_pos_out_of_manual_limit,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
                "bounds": (-3.0, 3.0)},
    )
```

`time_out=True`는 이 종료가 시간 초과에 의한 것(truncation)임을 표시합니다. RL에서 truncation과 termination의 구분은 value function bootstrapping에 영향을 미친다 — 시간 초과는 실제 실패가 아니므로 다음 상태의 value를 bootstrap해야 합니다.

## 실행 방법

```bash
cd ~/workspace/IsaacLab

# 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Cartpole-v0 \
    --num_envs 4096

# 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Cartpole-v0 \
    --use_pretrained_checkpoint \
    --num_envs 32
```

## Comparison: Manager-Based의 장점

| 시나리오 | Manager-Based 이점 |
|---|---|
| Reward 실험 | Config에서 weight만 변경, 코드 수정 없음 |
| Observation 변경 | Term 추가/제거로 observation 공간 확장 |
| 새 로봇 적용 | `SceneEntityCfg`의 로봇 이름만 변경 |
| 팀 협업 | MDP 함수를 라이브러리로 공유 |
| 도메인 랜덤화 | EventTermCfg 추가로 물리 파라미터 랜덤화 |

## Further Reading

- **ManagerBasedRLEnv**: `~/workspace/IsaacLab/source/isaaclab/isaaclab/envs/manager_based_rl_env.py`
- **MDP 함수 라이브러리**: `~/workspace/IsaacLab/source/isaaclab/isaaclab/envs/mdp/`
- **ObservationManager**: `~/workspace/IsaacLab/source/isaaclab/isaaclab/managers/observation_manager.py`
- **RewardManager**: `~/workspace/IsaacLab/source/isaaclab/isaaclab/managers/reward_manager.py`
