# Direct vs Manager-Based 환경 비교

## 개요

Isaac Lab은 RL 환경을 정의하는 두 가지 패러다임을 제공한다:

- **Direct**: 단일 클래스에서 `_get_observations()`, `_get_rewards()`, `_get_dones()`, `_reset_idx()`를 명시적으로 구현
- **Manager-Based**: 선언적 `Cfg` 클래스에서 `ObservationManager`, `RewardManager`, `TerminationManager` 등에 MDP 요소를 위임

CartPole이 두 패러다임으로 모두 구현되어 있어, 동일한 환경을 다른 방식으로 설계하는 비교가 가능하다.

## 구조 비교

### Direct 환경 (`DirectRLEnv` 상속)

```python
class CartpoleEnv(DirectRLEnv):
    cfg: CartpoleEnvCfg

    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        # joint 인덱스 저장, action scale 설정 등

    def _setup_scene(self):
        """씬 구성 — 로봇, 지형, 조명"""
        self.cartpole = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["cartpole"] = self.cartpole

    def _pre_physics_step(self, actions):
        """action 전처리 → 물리 명령 변환"""
        self.actions = actions.clone()

    def _apply_action(self):
        """실제 물리 명령 적용"""
        effort = self.cfg.action_scale * self.actions
        self.cartpole.set_joint_effort_target(effort, joint_ids=self._cart_dof_idx)

    def _get_observations(self):
        """observation tensor 구성"""
        obs = torch.cat([self.joint_pos, self.joint_vel], dim=-1)
        return {"policy": obs}

    def _get_rewards(self):
        """reward 계산 (JIT 가능)"""
        return compute_rewards(self.joint_pos, self.joint_vel, ...)

    def _get_dones(self):
        """종료 조건"""
        terminated = (joint_pos[:, 1].abs() > 0.2617)  # pole angle > 15°
        return terminated, time_out

    def _reset_idx(self, env_ids):
        """특정 환경만 리셋"""
        joint_pos = self.cartpole.data.default_joint_pos[env_ids] + offset
        self.cartpole.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
```

### Manager-Based 환경 (`ManagerBasedRLEnv` 상속)

```python
@configclass
class CartpoleEnvCfg(ManagerBasedRLEnvCfg):
    # 씬 구성
    scene: CartpoleSceneCfg = CartpoleSceneCfg(num_envs=4096, env_spacing=4.0)

    # Action
    actions: ActionsCfg = ActionsCfg()

    # Observation
    observations: ObservationsCfg = ObservationsCfg()

    # Reward
    rewards: RewardsCfg = RewardsCfg()

    # Termination
    terminations: TerminationsCfg = TerminationsCfg()

    # Event (리셋 랜덤화)
    events: EventCfg = EventCfg()


@configclass
class ActionsCfg:
    joint_effort = JointEffortActionCfg(
        asset_name="robot",
        joint_names=["slider_to_cart"],
        scale=100.0,
    )

@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)
    policy: PolicyCfg = PolicyCfg()

@configclass
class RewardsCfg:
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    cart_vel = RewTerm(func=mdp.joint_vel_l1, weight=-0.01, params={...})
    pole_vel = RewTerm(func=mdp.joint_vel_l1, weight=-0.005, params={...})
    pole_pos = RewTerm(func=mdp.joint_pos_target_l2, weight=-1.0, params={...})
```

## 상세 비교

### 코드 구조

| 항목 | Direct | Manager-Based |
|---|---|---|
| 클래스 수 | 1 (Env) + 1 (Cfg) | 1 (Cfg) + N (Sub-Cfg) |
| 환경 로직 위치 | Python 메서드 | Cfg 클래스 + mdp 함수 |
| Reward 정의 | `_get_rewards()` 메서드 | `RewardsCfg` 선언 |
| Observation 정의 | `_get_observations()` 메서드 | `ObservationsCfg` 선언 |
| Reset 정의 | `_reset_idx()` 메서드 | `EventCfg` 선언 |
| 커스텀 로직 | 무제한 자유도 | Manager 프레임워크 내 |

### Reward 설계

**Direct — 명시적 계산**:
```python
@torch.jit.script
def compute_rewards(pole_angle, pole_vel, cart_vel, cart_pos, reset_terminated):
    reward = 1.0 - pole_angle.abs() / math.pi
    reward -= 0.01 * torch.abs(cart_vel)
    reward -= 0.005 * torch.abs(pole_vel)
    reward -= 0.01 * torch.abs(cart_pos)
    reward -= 2.0 * reset_terminated.float()
    return reward
```

- `@torch.jit.script`로 JIT 컴파일 → GPU에서 최적 실행
- 단일 함수에서 모든 reward 항을 합산
- reward 항의 가중치가 코드에 하드코딩

**Manager-Based — 선언적 구성**:
```python
@configclass
class RewardsCfg:
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    cart_vel = RewTerm(func=mdp.joint_vel_l1, weight=-0.01, params={"asset_cfg": SceneEntityCfg("robot")})
    pole_pos = RewTerm(func=mdp.joint_pos_target_l2, weight=-1.0, params={"target": 0.0, ...})
```

- 각 reward 항이 독립적인 `RewTerm` 객체
- `weight`를 config에서 조정 가능 (코드 수정 불필요)
- `mdp` 모듈의 재사용 가능한 함수 조합
- 새 reward 추가 = `RewTerm` 한 줄 추가

### Action Space

**Direct**:
```python
def _apply_action(self):
    effort = self.cfg.action_scale * self.actions
    self.cartpole.set_joint_effort_target(effort, joint_ids=self._cart_dof_idx)
```
직접 joint effort를 설정. action → 물리 명령 변환을 완전히 제어.

**Manager-Based**:
```python
@configclass
class ActionsCfg:
    joint_effort = JointEffortActionCfg(asset_name="robot", joint_names=["slider_to_cart"], scale=100.0)
```
`JointEffortActionCfg`가 자동으로 action → effort 변환을 처리. `JointPositionActionCfg`, `DifferentialInverseKinematicsActionCfg` 등으로 교체하면 동일 환경에서 다른 action space를 사용할 수 있다.

### Reset 처리

**Direct**:
```python
def _reset_idx(self, env_ids):
    joint_pos = self.cartpole.data.default_joint_pos[env_ids].clone()
    joint_pos[:, 0] += sample_uniform(-1.0, 1.0, ...)    # cart 위치
    joint_pos[:, 1] += sample_uniform(-0.125, 0.125, ...) # pole 각도
    self.cartpole.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
```

**Manager-Based**:
```python
@configclass
class EventCfg:
    reset_cart_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={"asset_cfg": ..., "position_range": (-1.0, 1.0), "velocity_range": (-0.5, 0.5)},
    )
```

## 선택 가이드

### Direct를 선택하는 경우

- **커스텀 물리 로직**: observation/reward 계산에 비표준적인 연산이 필요할 때
- **성능 최적화**: `@torch.jit.script`로 reward 계산을 JIT 컴파일하여 최대 성능을 추구할 때
- **복잡한 상태 머신**: reward 계산이 이전 스텝의 상태에 의존하거나, 복잡한 조건 분기가 있을 때
- **빠른 프로토타이핑**: Manager 구조를 학습하지 않고 바로 구현하고 싶을 때
- **예시**: `06-isaaclab-classic/01-cartpole-direct/`, `03-ant/`, `04-humanoid/`

### Manager-Based를 선택하는 경우

- **모듈성**: 동일한 로봇에 대해 reward 함수만 교체하면서 실험할 때
- **재현성**: config 파일만으로 환경 설정을 완전히 기록할 때
- **팀 작업**: reward, observation, action을 독립적으로 개발/수정할 때
- **Action space 실험**: 동일 환경에서 joint_pos, ik_abs, ik_rel, osc 등을 쉽게 전환할 때
- **도메인 랜덤화**: EventManager로 다양한 랜덤화를 선언적으로 구성할 때
- **예시**: `06-isaaclab-classic/02-cartpole-manager/`, `07-isaaclab-locomotion/`, `08-isaaclab-manipulation/`

### 실제 사용 패턴

Isaac Lab의 공식 환경은 대부분 **Manager-Based**로 구현되어 있다. 특히 locomotion과 manipulation 환경은 `velocity_env_cfg.py` 같은 공통 base config를 상속하여 로봇별로 최소한의 수정만 적용하는 패턴이 주류다.

Direct 환경은 CartPole, Ant, Humanoid 같은 **고전 제어 환경**에서 주로 사용되며, 단순한 환경에서 오버헤드 없이 빠르게 구현할 때 적합하다.

## 코드량 비교 (CartPole 기준)

| 항목 | Direct | Manager-Based |
|---|---|---|
| 환경 클래스 | ~100줄 | ~10줄 (wrapper만) |
| Config 클래스 | ~60줄 | ~130줄 (모든 Cfg 합산) |
| 총 코드량 | ~160줄 | ~140줄 |
| Reward 항 추가 시 | 함수 수정 | `RewTerm` 1줄 추가 |
| Action space 변경 시 | `_apply_action()` 전체 수정 | `ActionsCfg` 1줄 교체 |
