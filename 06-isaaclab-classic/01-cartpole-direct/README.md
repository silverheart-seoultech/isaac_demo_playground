# CartPole — Direct RL Environment
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

Isaac Lab의 Direct RL 패러다임으로 구현된 CartPole 환경. 단일 클래스(`CartpoleEnv`)에서 observation 구성, reward 계산, reset 로직을 명시적으로 구현합니다. 4차원 observation, 1차원 action의 최소 구성으로 Isaac Lab의 환경 구조를 이해하는 진입점입니다.

Manager-Based 버전(`02-cartpole-manager/`)과 비교하면 Direct 패러다임의 특성 — 명시적 제어, 단일 파일 구현, JIT 컴파일 가능한 리워드 — 을 명확히 파악할 수 있습니다.

## Architecture

### 클래스 계층

```
DirectRLEnv (isaaclab.envs)
    │
    └── CartpoleEnv (cartpole_env.py)
            │
            ├── _setup_scene()          ← Articulation + GroundPlane 생성
            ├── _pre_physics_step()     ← Action 스케일링 → effort 적용
            ├── _get_observations()     ← 4D observation 벡터 구성
            ├── _get_rewards()          ← 5개 reward term 계산
            ├── _get_dones()            ← 종료 조건 (위치/각도/시간)
            └── _reset_idx()            ← 환경별 초기 상태 랜덤화
```

### 시뮬레이션 루프

```
학습 프레임워크 (RSL-RL)
    │
    ├── env.reset()
    │   └── _reset_idx(env_ids)  ← 관절 위치/속도 랜덤 초기화
    │
    └── for iteration in range(max_iterations):
            │
            ├── obs = env.step(action)
            │       │
            │       ├── _pre_physics_step(action)
            │       │       └── effort = action × 100.0  ← 스케일링
            │       │
            │       ├── [PhysX step × decimation]  ← dt=1/120 × 2회
            │       │
            │       ├── _get_observations()
            │       │       └── [joint_pos, joint_vel, cart_pos, cart_vel]
            │       │
            │       ├── _get_rewards()
            │       │       └── alive + termination + pole_pos + cart_vel + pole_vel
            │       │
            │       └── _get_dones()
            │               └── cart > 3m OR pole > π/2 OR time > 5s
            │
            └── policy.update(obs, reward, done)
```

## Source Files

| 파일 | 역할 |
|---|---|
| `isaaclab_tasks/direct/cartpole/cartpole_env.py` | 환경 전체 구현 (175줄) |
| `isaaclab_tasks/direct/cartpole/__init__.py` | Gymnasium 등록: `Isaac-Cartpole-Direct-v0` |
| `isaaclab_tasks/direct/cartpole/agents/rsl_rl_ppo_cfg.py` | RSL-RL PPO 하이퍼파라미터 |
| `isaaclab_assets/robots/cartpole.py` | 로봇 에셋 config (2-DOF) |

## 핵심 코드 분석

### Observation Space (4D)

```python
def _get_observations(self) -> dict:
    obs = torch.cat(
        (
            self.joint_pos[:, self._pole_dof_idx[0]].unsqueeze(dim=1),   # pole 각도
            self.joint_vel[:, self._pole_dof_idx[0]].unsqueeze(dim=1),   # pole 각속도
            self.joint_pos[:, self._cart_dof_idx[0]].unsqueeze(dim=1),   # cart 위치
            self.joint_vel[:, self._cart_dof_idx[0]].unsqueeze(dim=1),   # cart 속도
        ),
        dim=-1,
    )
    return {"policy": obs}
```

**설계 근거**: CartPole은 pole 각도/속도와 cart 위치/속도 4개 값으로 시스템 상태가 완전히 결정됩니다 (fully observable). 별도의 노이즈나 정규화 없이 raw 값을 직접 사용하는데, 이는 시스템이 단순하여 정책이 raw 값의 범위를 빠르게 학습할 수 있기 때문입니다.

`"policy"` 키로 반환하는 구조는 Isaac Lab의 관례로, 학습 프레임워크가 이 키로 observation을 참조합니다.

### Reward Decomposition

```python
def _get_rewards(self) -> torch.Tensor:
    total_reward = compute_rewards(
        self.cfg.rew_scale_alive,        # +1.0  — 생존 보너스
        self.cfg.rew_scale_terminated,   # -2.0  — 종료 페널티
        self.cfg.rew_scale_pole_pos,     # -1.0  — pole 직립도
        self.cfg.rew_scale_cart_vel,     # -0.01 — cart 속도 억제
        self.cfg.rew_scale_pole_vel,     # -0.005 — pole 진동 억제
        self.joint_pos[:, self._pole_dof_idx[0]],
        self.joint_vel[:, self._pole_dof_idx[0]],
        self.joint_pos[:, self._cart_dof_idx[0]],
        self.joint_vel[:, self._cart_dof_idx[0]],
        self.reset_terminated,
    )
    return total_reward
```

| Reward Term | Weight | 수식 | 설계 의도 |
|---|---|---|---|
| `rew_alive` | +1.0 | 매 스텝 상수 | 최대한 오래 생존하도록 유도 |
| `rew_terminated` | -2.0 | 종료 시 1회 | 실패의 비용을 명시 |
| `rew_pole_pos` | -1.0 | Σ(pole_angle²) | pole을 수직으로 유지 |
| `rew_cart_vel` | -0.01 | \|cart_vel\| | cart의 불필요한 이동 억제 |
| `rew_pole_vel` | -0.005 | \|pole_vel\| | pole의 진동/흔들림 억제 |

**가중치 분석**: `rew_alive(+1.0)`가 가장 크므로 정책은 우선 생존을 최적화합니다. `rew_pole_pos(-1.0)`가 다음으로 커서 pole 직립에 집중합니다. 속도 관련 페널티는 상대적으로 작아(0.01, 0.005) 안정성보다는 약한 regularizer 역할을 합니다.

### JIT-Compiled Reward Function

```python
@torch.jit.script
def compute_rewards(
    rew_scale_alive: float,
    rew_scale_terminated: float,
    rew_scale_pole_pos: float,
    rew_scale_cart_vel: float,
    rew_scale_pole_vel: float,
    pole_pos: torch.Tensor,
    pole_vel: torch.Tensor,
    cart_pos: torch.Tensor,
    cart_vel: torch.Tensor,
    reset_terminated: torch.Tensor,
) -> torch.Tensor:
    ...
```

`@torch.jit.script`는 Python 오버헤드를 제거하고 CUDA 커널로 컴파일합니다. 4096+ 환경을 병렬 실행할 때 reward 계산이 bottleneck이 되지 않도록 최적화한 것입니다. Manager-Based 방식에서는 이 최적화가 자동으로 적용되지 않으므로, 성능 민감한 환경에서 Direct 방식이 유리할 수 있습니다.

### Reset Strategy

```python
def _reset_idx(self, env_ids: Sequence[int] | None):
    if env_ids is None:
        env_ids = self.cartpole._ALL_INDICES

    joint_pos = self.cartpole.data.default_joint_pos[env_ids]
    joint_pos[:, self._pole_dof_idx] += sample_uniform(
        -0.25 * math.pi, 0.25 * math.pi,  # pole: ±45°
        joint_pos[:, self._pole_dof_idx].shape, ...
    )
    joint_pos[:, self._cart_dof_idx] += sample_uniform(
        -1.0, 1.0,                          # cart: ±1m
        joint_pos[:, self._cart_dof_idx].shape, ...
    )
    joint_vel = self.cartpole.data.default_joint_vel[env_ids]  # 0으로 초기화
    self.cartpole.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
```

**설계 근거**: Pole 각도를 ±45°, cart 위치를 ±1m 범위에서 랜덤 초기화합니다. 속도는 0으로 시작합니다. 이 초기 분포는 정책이 다양한 초기 조건에서 복구하는 능력을 학습하게 합니다. 범위가 너무 넓으면(±90°) 초기 상태에서 이미 종료 조건에 도달하여 학습이 비효율적입니다.

### 물리 설정

```python
class CartpoleEnvCfg(DirectRLEnvCfg):
    decimation = 2           # 물리 2스텝당 1 제어 스텝
    episode_length_s = 5.0   # 최대 에피소드 길이

    sim: SimCfg = SimCfg(dt=1/120)  # 120Hz 물리 시뮬레이션
    # → 제어 주파수 = 120/2 = 60Hz
```

| 파라미터 | 값 | 의미 |
|---|---|---|
| `dt` | 1/120 s | PhysX 시뮬레이션 타임스텝 (120Hz) |
| `decimation` | 2 | 물리 2스텝마다 1회 action 적용 |
| 제어 주파수 | 60 Hz | dt × decimation = 1/60 |
| `episode_length_s` | 5.0 s | 최대 300 제어 스텝 (5 × 60) |
| `action_scale` | 100.0 | action [-1,1] → force [-100, 100] N |

### 로봇 에셋

```python
# isaaclab_assets/robots/cartpole.py
CARTPOLE_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    actuators={
        "cart_actuator": ImplicitActuatorCfg(
            joint_names_expr=["slider_to_cart"],
            effort_limit=400.0,
            stiffness=0.0,    # 위치 게인 없음 (effort 제어)
            damping=10.0,     # 속도 감쇠
        ),
        "pole_actuator": ImplicitActuatorCfg(
            joint_names_expr=["cart_to_pole"],
            effort_limit=400.0,
            stiffness=0.0,
            damping=0.0,      # 자유 회전 (감쇠 없음)
        ),
    },
)
```

`{ENV_REGEX_NS}`는 Isaac Lab이 병렬 환경 생성 시 자동으로 `/World/envs/env_0`, `/World/envs/env_1`, ...으로 치환합니다. Cart의 damping=10.0은 과도한 속도를 물리적으로 억제하며, pole의 damping=0.0은 자유 회전을 허용하여 제어 문제의 본질을 유지합니다.

## 실행 방법

```bash
cd ~/workspace/IsaacLab

# 학습 (4096 병렬 환경)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 4096

# 사전학습 체크포인트로 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Cartpole-Direct-v0 \
    --use_pretrained_checkpoint \
    --num_envs 32

# Headless 학습 (서버)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 4096 \
    --headless

# 환경 수 줄여서 빠른 테스트
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 16 \
    --max_iterations 10
```

### 학습 하이퍼파라미터 (RSL-RL PPO)

| 파라미터 | 값 | 설명 |
|---|---|---|
| Network | [32, 32] | Actor/Critic MLP 크기 |
| Learning Rate | 1e-3 | Adam optimizer |
| Num Steps | 16 | Rollout length per env |
| Max Iterations | 150 | 총 학습 반복 |
| Discount (γ) | 0.99 | 할인율 |
| GAE (λ) | 0.95 | Generalized Advantage Estimation |
| Entropy Coef | 0.0 | Entropy bonus 없음 |

네트워크가 [32, 32]으로 매우 작은 것은 CartPole이 4D observation → 1D action의 단순한 매핑이기 때문입니다. 150 iteration이면 약 10M 환경 스텝(4096 × 16 × 150)으로 수렴합니다.

## Comparison: Direct vs Manager-Based

| 항목 | Direct (이 문서) | Manager-Based (`02-cartpole-manager/`) |
|---|---|---|
| 구현 파일 수 | 1 (cartpole_env.py) | 3+ (config + mdp 모듈) |
| Observation 정의 | 명시적 `torch.cat()` | 선언적 `ObservationTermCfg` |
| Reward 정의 | JIT-compiled 함수 | 개별 `RewardTermCfg` |
| Reset 로직 | `_reset_idx()` 직접 구현 | `EventTermCfg`로 선언 |
| 코드 재사용 | 낮음 (복사 필요) | 높음 (MDP term 공유) |
| 성능 | JIT 최적화 가능 | Manager 오버헤드 있음 |
| 적합한 경우 | 연구 프로토타이핑, 성능 최적화 | 복잡한 환경, 팀 프로젝트 |

## Further Reading

- **DirectRLEnv 베이스 클래스**: `~/workspace/IsaacLab/source/isaaclab/isaaclab/envs/direct_rl_env.py`
- **CartPole 소스**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/cartpole/cartpole_env.py`
- **CartPole 에셋**: `~/workspace/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/cartpole.py`
- **RSL-RL PPO Config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/cartpole/agents/rsl_rl_ppo_cfg.py`
