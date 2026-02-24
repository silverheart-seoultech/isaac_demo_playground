# ANYmal C — Standalone 보행 정책 배포 (LSTM Actuator Network)
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

ANYbotics ANYmal C 사족보행 로봇의 학습된 보행 정책을 **Standalone 스크립트**로 배포하는 데모. Spot/H1과 동일한 `PolicyController` 프레임워크를 사용하지만, 두 가지 핵심적인 차이가 있다:

1. **Effort (Torque) Control**: Spot/H1의 position control과 달리, 정책 출력을 LSTM 기반 actuator network가 토크로 변환하여 관절에 직접 적용합니다.
2. **Standalone 실행**: GUI Extension이 아닌 독립 Python 스크립트로 실행됩니다. `SimulationApp` → `World` → 물리 루프를 직접 구성합니다.

Warehouse 환경에서 ANYmal이 보행하며, 키보드로 실시간 제어할 수 있습니다.

## Architecture

### 클래스 계층

```
PolicyController (controllers/policy_controller.py)
    │
    └── AnymalFlatTerrainPolicy (robots/anymal.py)
            │
            ├── _compute_observation()  ← 48D observation (Spot과 동일 구조)
            ├── forward()               ← 정책 출력 → LSTM SEA → 토크 적용
            └── initialize()            ← control_mode="effort" 오버라이드
                    └── LstmSeaNetwork   ← LSTM actuator network 로드

Anymal_runner (anymal_standalone.py)
    │
    ├── __init__()      ← World + Warehouse + ANYmal 생성
    ├── setup()         ← 키보드 리스너 + 물리 콜백 등록
    ├── on_physics_step()  ← 매 물리 스텝마다 forward() 호출
    └── run()           ← 메인 시뮬레이션 루프
```

### Standalone 실행 흐름

```
anymal_standalone.py
    │
    ├── SimulationApp({"headless": False})    ← Omniverse Kit 초기화
    │
    ├── Anymal_runner(physics_dt=1/200, render_dt=1/60)
    │       ├── World(physics_dt=1/200, rendering_dt=1/60)
    │       ├── Warehouse USD 로드
    │       └── AnymalFlatTerrainPolicy 생성
    │
    ├── world.reset() → runner.setup() → runner.run()
    │
    └── run() 내부 루프:
            while simulation_app.is_running():
                world.step(render=True)
                    │
                    └── on_physics_step(step_size)  ← 200Hz
                            │
                            ├── [첫 프레임] anymal.initialize()
                            │       ├── control_mode = "effort"
                            │       └── LstmSeaNetwork.setup(sea_net_jit2.pt)
                            │
                            └── [이후] anymal.forward(dt, base_command)
                                    │
                                    ├── obs = _compute_observation()  ← 48D
                                    ├── action = _compute_action(obs) ← TorchScript 정책
                                    │
                                    ├── actuator_network.compute_torques(
                                    │       joint_pos, joint_vel, action × 0.5
                                    │   )
                                    │
                                    └── robot.set_joint_efforts(torques)
```

## Source Files

| 파일 | 역할 |
|---|---|
| `standalone_examples/api/isaacsim.robot.policy.examples/anymal_standalone.py` | Standalone 데모 메인 (158줄) |
| `isaacsim.robot.policy.examples/robots/anymal.py` | ANYmal 정책 래퍼 (147줄) |
| `isaacsim.robot.policy.examples/controllers/policy_controller.py` | 정책 로드/추론 베이스 클래스 |
| `isaacsim.robot.policy.examples/utils/actuator_network.py` | LstmSeaNetwork + SeaNetwork |
| `Nucleus: Isaac/Samples/Policies/Anymal_Policies/anymal_policy.pt` | 사전학습된 TorchScript 정책 |
| `Nucleus: Isaac/Samples/Policies/Anymal_Policies/anymal_env.yaml` | 환경 설정 |
| `Nucleus: Isaac/Samples/Policies/Anymal_Policies/sea_net_jit2.pt` | LSTM actuator network (TorchScript) |
| `Nucleus: Isaac/Robots/ANYbotics/anymal_c/anymal_c.usd` | ANYmal C USD 에셋 |
| `Nucleus: Isaac/Environments/Simple_Warehouse/warehouse.usd` | Warehouse 환경 |

## 핵심 코드 분석

### LSTM SEA Actuator Network

ANYmal의 가장 중요한 차별점은 **actuator network**입니다. Spot/H1은 정책 출력을 바로 관절 위치 명령으로 변환하지만, ANYmal은 정책 출력을 **LSTM 기반 Series Elastic Actuator(SEA) 네트워크**가 토크로 변환합니다.

```python
class LstmSeaNetwork:
    def __init__(self):
        self._hidden_state = torch.zeros((2, 12, 8))  # 2-layer LSTM, 12 joints, 8 hidden
        self._cell_state = torch.zeros((2, 12, 8))

    @torch.no_grad()
    def compute_torques(self, joint_pos, joint_vel, actions):
        actuator_net_input = torch.zeros((12, 1, 2))
        # 입력 1: 위치 오차 = (desired_pos - current_pos)
        actuator_net_input[:, 0, 0] = torch.from_numpy(
            actions + self._default_joint_pos - joint_pos
        )
        # 입력 2: 현재 관절 속도 ([-20, 20] 클리핑)
        actuator_net_input[:, 0, 1] = torch.from_numpy(
            np.clip(joint_vel, -20.0, 20)
        )

        # LSTM 추론 (hidden state 유지)
        torques, (self._hidden_state, self._cell_state) = self._network(
            actuator_net_input, (self._hidden_state, self._cell_state)
        )

        return torques.detach().clip(-80.0, 80.0).numpy(), ...
```

**SEA (Series Elastic Actuator)**: ANYmal의 실제 하드웨어는 관절에 탄성 요소(spring)가 직렬로 연결된 SEA를 사용합니다. 이 탄성 요소의 비선형 특성을 정확히 모델링하기 위해 학습된 LSTM 네트워크를 actuator model로 사용합니다.

**네트워크 구조**:
- 입력: (위치 오차, 관절 속도) — 각 관절 독립, 2D
- LSTM: 2-layer, hidden size 8 — 각 관절 별도의 hidden state 유지
- 출력: 토크 ([-80, 80] Nm 클리핑)
- 상태: hidden/cell state가 매 스텝 유지되어 시간적 맥락을 반영

**왜 LSTM인가**: 탄성 요소의 변형 이력이 현재 토크에 영향을 미치므로, memoryless MLP보다 시간 의존성을 포착하는 LSTM이 적합합니다. 관절 속도의 클리핑(±20 rad/s)은 비현실적인 입력을 방지합니다.

### Forward (Effort Control)

```python
def forward(self, dt, command):
    if self._policy_counter % self._decimation == 0:
        obs = self._compute_observation(command)
        self.action = self._compute_action(obs)
        self._previous_action = self.action.copy()

    current_joint_pos = self.robot.get_joint_positions()
    current_joint_vel = self.robot.get_joint_velocities()

    joint_torques, _ = self._actuator_network.compute_torques(
        current_joint_pos, current_joint_vel, self._action_scale * self.action
    )
    self.robot.set_joint_efforts(joint_torques)
    self._policy_counter += 1
```

**Spot/H1과의 핵심 차이**:

| 단계 | Spot/H1 | ANYmal |
|---|---|---|
| 정책 출력 해석 | 관절 위치 오프셋 | actuator network 입력 |
| 변환 과정 | `default_pos + action × scale` | LSTM SEA → 토크 계산 |
| 적용 방식 | `robot.apply_action(ArticulationAction)` | `robot.set_joint_efforts(torques)` |
| 제어 모드 | Position | Effort |
| 매 물리 스텝 | 동일 action 재적용 | 매 스텝 토크 재계산 |

ANYmal은 **매 물리 스텝마다** actuator network를 통해 토크를 재계산합니다(정책은 decimation 주기로만 업데이트). 이는 현재 관절 상태(위치, 속도)를 반영한 토크 보정이 매 스텝 필요하기 때문입니다.

### Initialize (Effort Mode)

```python
def initialize(self, physics_sim_view=None):
    super().initialize(physics_sim_view=physics_sim_view, control_mode="effort")

    assets_root_path = get_assets_root_path()
    file_content = omni.client.read_file(
        assets_root_path + "/Isaac/Samples/Policies/Anymal_Policies/sea_net_jit2.pt"
    )[2]
    file = io.BytesIO(memoryview(file_content).tobytes())
    self._actuator_network = LstmSeaNetwork()
    self._actuator_network.setup(file, self.default_pos)
    self._actuator_network.reset()
```

`control_mode="effort"`는 PhysX가 관절을 PD 컨트롤러로 구동하지 않고, 직접 지정된 토크를 적용하게 합니다. Actuator network가 사실상 학습된 PD 컨트롤러 역할을 대체합니다.

### Standalone 실행 패턴

```python
# GUI Extension과 달리 SimulationApp부터 직접 초기화
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# World 생성 (물리 설정 포함)
self._world = World(
    stage_units_in_meters=1.0,
    physics_dt=physics_dt,       # 1/200
    rendering_dt=render_dt       # 1/60
)

# Warehouse 환경 로드 (USD Reference)
prim = define_prim("/World/Warehouse", "Xform")
prim.GetReferences().AddReference(asset_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")

# 메인 루프
def run(self):
    while simulation_app.is_running():
        self._world.step(render=True)
        if self._world.is_stopped():
            self.needs_reset = True
```

**GUI Extension vs Standalone 비교**:

| 항목 | GUI Extension (Spot/H1) | Standalone (ANYmal) |
|---|---|---|
| 초기화 | Extension `on_startup()` → BaseSample | `SimulationApp` → `World` 직접 생성 |
| 씬 구성 | `setup_scene()` 메서드 | 생성자에서 직접 USD 로드 |
| 물리 루프 | GUI Play 버튼 → 자동 콜백 | `while simulation_app.is_running()` 수동 루프 |
| 환경 | 빈 ground plane | Warehouse USD (실내 환경) |
| 렌더링 | GUI가 관리 | `world.step(render=True)` |
| 종료 | GUI Stop → `world_cleanup()` | `simulation_app.close()` |
| Headless | 불가 | `SimulationApp({"headless": True})` |

Standalone 방식은 GUI 없이도 실행 가능하므로, 서버 환경에서의 대규모 평가나 데이터 수집에 적합합니다.

### 물리 설정

| 파라미터 | 값 | Spot 대비 |
|---|---|---|
| `physics_dt` | 1/200 s (200Hz) | Spot 1/500 (2.5배 낮음) |
| `render_dt` | 1/60 s | Spot 1/50 |
| `action_scale` | 0.5 | Spot 0.2 (2.5배 큼) |
| 제어 모드 | Effort (토크) | Position |
| 토크 클리핑 | ±80 Nm | N/A |
| 속도 클리핑 | ±20 rad/s (actuator input) | N/A |
| 초기 높이 | 0.7 m | Spot 0.8 m |
| 환경 | Warehouse (실내) | Ground Plane |

### 키보드 제어

```python
self._input_keyboard_mapping = {
    "UP":    [1.0, 0.0, 0.0],    # 전진 +1.0 m/s
    "DOWN":  [-1.0, 0.0, 0.0],   # 후진 -1.0 m/s
    "RIGHT": [0.0, -1.0, 0.0],   # 우측 -1.0 m/s
    "LEFT":  [0.0, 1.0, 0.0],    # 좌측 +1.0 m/s
    "N":     [0.0, 0.0, 1.0],    # 반시계 회전
    "M":     [0.0, 0.0, -1.0],   # 시계 회전
}
```

Spot(±2.0 m/s)보다 보수적인 ±1.0 m/s 명령 범위를 사용합니다. Spot과 동일한 6방향(전후/좌우/회전) 명령을 지원합니다.

### Reset 처리

```python
def on_physics_step(self, step_size):
    if self.first_step:
        self._anymal.initialize()
        self.first_step = False
    elif self.needs_reset:
        self._world.reset(True)
        self.needs_reset = False
        self.first_step = True
    else:
        self._anymal.forward(step_size, self._base_command)
```

Standalone에서는 GUI의 Stop/Play와 달리 `is_stopped()` 상태를 감지하여 수동으로 reset을 처리합니다. `needs_reset` 플래그를 통해 다음 물리 스텝에서 초기화가 이루어집니다. Reset 시 `first_step = True`로 재설정하여 `initialize()`가 다시 호출되며, 이 과정에서 LSTM hidden state도 리셋됩니다.

## 실행 방법

```bash
cd ~/workspace/IsaacSim

# GUI 모드
python standalone_examples/api/isaacsim.robot.policy.examples/anymal_standalone.py

# Headless 모드 (코드 수정 필요: headless=True)
# SimulationApp({"headless": True}) 로 변경
```

### 키보드 조작

| 키 | 명령 |
|---|---|
| `↑` / `Numpad 8` | 전진 (+1.0 m/s) |
| `↓` / `Numpad 2` | 후진 (-1.0 m/s) |
| `←` / `Numpad 4` | 좌측 이동 (+1.0 m/s) |
| `→` / `Numpad 6` | 우측 이동 (-1.0 m/s) |
| `N` / `Numpad 7` | 반시계 회전 (+1.0 rad/s) |
| `M` / `Numpad 9` | 시계 회전 (-1.0 rad/s) |

## Comparison: 세 보행 데모의 Actuator 비교

| 항목 | Spot | H1 | ANYmal (이 문서) |
|---|---|---|---|
| 제어 방식 | Position | Position | Effort (Torque) |
| Action → 관절 | `default + action × 0.2` → PD | `default + action × 0.5` → PD | `action × 0.5` → LSTM SEA → torque |
| Actuator Model | PhysX PD Controller | PhysX PD Controller | 학습된 LSTM SEA Network |
| 물리적 근거 | 단순 위치 서보 | 단순 위치 서보 | 실제 하드웨어의 탄성 액추에이터 |
| 토크 재계산 | 물리 엔진 내부 | 물리 엔진 내부 | 매 물리 스텝 명시적 계산 |
| Sim-to-Real Gap | 높음 (PD ≠ 실제) | 높음 | 낮음 (SEA 모델링) |

ANYmal의 actuator network 접근은 **sim-to-real transfer**에서 중요한 의미를 가집니다. 실제 로봇의 액추에이터 특성(탄성, 마찰, 비선형성)을 학습된 네트워크로 시뮬레이션에서 재현하므로, 학습된 정책이 실제 하드웨어에서 더 잘 동작할 가능성이 높습니다.

## Further Reading

- **AnymalFlatTerrainPolicy**: `~/workspace/IsaacSim/exts/isaacsim.robot.policy.examples/.../robots/anymal.py`
- **LstmSeaNetwork**: `~/workspace/IsaacSim/exts/isaacsim.robot.policy.examples/.../utils/actuator_network.py`
- **PolicyController**: `~/workspace/IsaacSim/exts/isaacsim.robot.policy.examples/.../controllers/policy_controller.py`
- **Standalone Script**: `~/workspace/IsaacSim/standalone_examples/api/isaacsim.robot.policy.examples/anymal_standalone.py`
- **Isaac Lab ANYmal 학습 환경**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/anymal_c/`
- **정책 배포 튜토리얼**: https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html
