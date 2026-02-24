# 물리 시뮬레이션 타이밍 구조

## 핵심 시간 파라미터

Isaac Sim/Lab의 시뮬레이션 루프에는 세 가지 독립적인 시간 스케일이 존재합니다. 이 세 가지의 관계를 정확히 이해하는 것이 시뮬레이션 안정성과 학습 성능의 핵심입니다.

```
┌──────────────────────────────────────────────────────┐
│ Policy Step (control_dt)                             │
│  = physics_dt × decimation                           │
│  RL policy가 action을 출력하는 주기                    │
│                                                      │
│  ┌────────────┬────────────┬──── ─ ─ ─┐              │
│  │ Physics dt │ Physics dt │  ...     │ × decimation │
│  │  (sim.dt)  │  (sim.dt)  │          │              │
│  └────────────┴────────────┴──── ─ ─ ─┘              │
│                                                      │
│  Rendering: 매 render_interval번째 physics step에서   │
└──────────────────────────────────────────────────────┘
```

| 파라미터 | 설정 위치 | 기본값 | 의미 |
|---|---|---|---|
| `physics_dt` | `SimulationCfg.dt` | 1/60 s | PhysX 엔진의 물리 연산 주기 |
| `render_interval` | `SimulationCfg.render_interval` | 1 | 렌더링 수행 간격 (physics step 단위) |
| `decimation` | `DirectRLEnvCfg.decimation` / `ManagerBasedEnvCfg.decimation` | MISSING (필수 설정) | policy step 당 physics step 횟수 |
| `control_dt` | 자동 계산 | `dt × decimation` | RL policy의 action 출력 주기 |
| `rendering_dt` | 자동 계산 | `dt × render_interval` | 화면 갱신 주기 |

## physics_dt (SimulationCfg.dt)

PhysX 엔진이 한 번의 물리 연산을 수행하는 시간 간격입니다. `SimulationCfg` 클래스에서 설정합니다:

```python
from isaaclab.sim import SimulationCfg

sim_cfg = SimulationCfg(
    dt=1.0 / 120.0,          # 120 Hz 물리 시뮬레이션
    render_interval=2,        # 2번의 physics step마다 1번 렌더링 (60 Hz 렌더링)
    gravity=(0.0, 0.0, -9.81),
)
```

### 태스크별 physics_dt 참고값

| 태스크 유형 | physics_dt | 이유 |
|---|---|---|
| CartPole (단순 제어) | 1/120 s | 낮은 주파수의 동역학으로 충분 |
| Franka manipulation | 1/120 ~ 1/400 s | 접촉 안정성과 IK 정밀도 요구 |
| 보행 (ANYmal, Spot) | 1/200 ~ 1/500 s | 다족 로봇의 빈번한 접촉/이탈 사이클 |
| Shadow Hand | 1/120 s | 고차원이지만 큰 힘 불필요 |

physics_dt를 작게 설정할수록 시뮬레이션 정확도가 높아지지만, 동일한 실시간 구간에 대해 더 많은 step을 계산해야 하므로 연산 비용이 증가합니다.

## decimation

`decimation`은 하나의 RL policy step 동안 반복 실행되는 physics step의 횟수입니다. Isaac Lab 환경의 `step()` 메서드 내부에서 다음과 같이 동작합니다:

```python
# DirectRLEnv.step() 내부 (간략화)
def step(self, action):
    # action을 decimation 횟수만큼 반복 적용
    for _ in range(self.cfg.decimation):
        self._sim_step_counter += 1
        self.scene.write_data_to_sim()
        self.sim.step(render=False)      # physics만 진행
        self.scene.update(dt=self.physics_dt)
    
    # decimation 완료 후 observation, reward 계산
    obs = self._get_observations()
    rewards = self._get_rewards()
    return obs, rewards, dones, infos
```

### control_dt 계산

```python
control_dt = physics_dt × decimation
```

예를 들어, `dt=1/120`이고 `decimation=4`이면:
- physics_dt = 8.33 ms (120 Hz)
- control_dt = 33.3 ms (30 Hz) — policy가 초당 30번 action을 출력

### decimation이 학습에 미치는 영향

| decimation | control_dt (dt=1/120 기준) | 특성 |
|---|---|---|
| 1 | 8.33 ms | 매 physics step마다 action 갱신. 반응 빠르지만 고주파 노이즈에 취약 |
| 2–4 | 16.7–33.3 ms | 일반적인 로봇 제어 주기와 유사. 대부분의 태스크에 적합 |
| 10+ | 83.3+ ms | 느린 policy 주기. 고수준 의사결정 태스크에 사용 |

실제 로봇의 제어 주기(보통 50–500 Hz)와 맞추려면 `physics_dt`와 `decimation`의 곱이 해당 주기의 역수가 되도록 설정합니다. 예를 들어, 실제 로봇이 200 Hz로 제어된다면 control_dt = 5 ms가 되어야 하므로, `dt=1/1000, decimation=5` 또는 `dt=1/200, decimation=1` 등으로 설정합니다.

## render_interval

`render_interval`은 렌더링을 수행하는 physics step 간격입니다. 학습 시에는 렌더링이 불필요하므로, `render_interval`을 `decimation`과 같거나 그보다 크게 설정하여 성능을 확보합니다:

```python
sim_cfg = SimulationCfg(
    dt=1.0 / 120.0,
    render_interval=4,  # decimation과 동일하게 설정 → policy step당 1회 렌더
)
```

Isaac Lab은 `render_interval < decimation`일 경우 경고를 출력합니다:

> "The render interval is smaller than the decimation. Multiple render calls will happen for each environment step."

카메라/LiDAR 센서를 사용하지 않는 학습에서는 `--headless` 플래그로 렌더링을 완전히 비활성화하는 것이 가장 빠릅니다.

## 에피소드 길이 계산

`episode_length_s`(초 단위)에서 실제 step 수를 계산하는 공식입니다:

```python
import math

episode_length_steps = math.ceil(episode_length_s / (decimation * physics_dt))
```

예시:
- `episode_length_s = 10.0`, `dt = 0.01`, `decimation = 10`
- `episode_length_steps = ceil(10.0 / (10 × 0.01)) = 100` policy steps

## Isaac Sim Standalone의 타이밍

Isaac Sim standalone 스크립트에서는 `decimation` 개념이 없으며, 직접 `world.step()`을 호출합니다:

```python
from isaacsim import SimulationApp
app = SimulationApp({"headless": False})

from isaacsim.core.api import World
world = World(physics_dt=1.0/120.0, rendering_dt=1.0/60.0)

# rendering_dt / physics_dt = 2 → 2번의 physics step마다 1번 렌더링
while app.is_running():
    world.step(render=True)   # physics + 렌더링
    # 또는
    world.step(render=False)  # physics만 (센서 불필요 시)
```

Isaac Sim standalone에서는 `rendering_dt`를 직접 지정할 수 있으며, 내부적으로 `rendering_dt / physics_dt`가 render_interval 역할을 합니다.

## 타이밍 설계 가이드라인

1. **physics_dt는 시뮬레이션 안정성에 맞춰 설정합니다.** 접촉이 빈번한 태스크(manipulation, locomotion)에서는 1/120 이하로 설정합니다. 물체가 관통하거나 폭발하듯 날아가면 dt를 줄여야 합니다.

2. **decimation은 원하는 policy 주기에 맞춰 설정합니다.** `control_dt = dt × decimation`이 실제 로봇 제어 주기와 비슷하도록 합니다.

3. **render_interval은 decimation 이상으로 설정합니다.** 학습 시 불필요한 렌더링은 순수한 성능 낭비입니다.

4. **solver iteration도 함께 조정합니다.** physics_dt를 줄이는 것만으로 불안정이 해결되지 않으면, PhysX solver의 position/velocity iteration count를 높입니다:

```python
from isaaclab.sim import SimulationCfg, PhysxCfg

sim_cfg = SimulationCfg(
    dt=1.0 / 200.0,
    physx=PhysxCfg(
        solver_type=1,                      # TGS solver
        min_position_iteration_count=4,     # 기본값 1 → 4로 증가
        min_velocity_iteration_count=1,     # 기본값 0 → 1로 증가
    ),
)
```

## 소스 코드 참조

| 파일 | 내용 |
|---|---|
| `IsaacLab/source/isaaclab/isaaclab/sim/simulation_cfg.py` | `SimulationCfg`, `PhysxCfg` 정의 |
| `IsaacLab/source/isaaclab/isaaclab/envs/direct_rl_env_cfg.py` | `decimation`, `episode_length_s` 정의 |
| `IsaacLab/source/isaaclab/isaaclab/envs/direct_rl_env.py` | `step()` 내부 decimation 루프 구현 |
| `IsaacLab/source/isaaclab/isaaclab/envs/manager_based_env_cfg.py` | Manager-Based 환경의 decimation 정의 |
