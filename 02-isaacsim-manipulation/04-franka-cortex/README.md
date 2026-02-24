# Franka Cortex — Decider Network 기반 행동 프레임워크

## Overview

Isaac Sim의 **Cortex** 행동 프레임워크를 사용하여 Franka가 큐브를 조작하는 데모. Cortex는 **Decider Network** 패턴으로 로봇의 행동을 정의하며, 상태 머신(state machine)과 행동 트리(behavior tree)의 장점을 결합한 의사결정 구조다.

RMPflow/RRT가 모션 레벨의 제어를 담당한다면, Cortex는 **태스크 레벨의 의사결정**을 담당한다: "어떤 큐브를 먼저 집을까?", "어디에 놓을까?", "다음 행동은 무엇인가?"를 결정한다.

## Architecture

```
CortexBase (cortex_base.py)
    │
    └── FrankaCortex (franka_cortex.py)
            │
            ├── setup_scene()
            │       ├── CortexFranka 로봇 추가 (add_franka_to_stage)
            │       └── 4개 DynamicCuboid (Red, Blue, Yellow, Green)
            │
            ├── load_behavior(behavior_module_path)
            │       └── DeciderNetwork = make_decider_network(robot)
            │
            └── _on_physics_step()
                    └── world.step(False, False) → Cortex 내부 갱신

CortexWorld → DeciderNetwork → Behavior Modules
    │
    ├── decider_network._decider_state.stack  ← 현재 의사결정 스택
    ├── context.diagnostics_message           ← 행동 상태 텍스트
    └── LogicalStateMonitor                   ← 상태 모니터링
```

## Source Files

| 파일 | 역할 |
|---|---|
| `franka_cortex/franka_cortex.py` | 데모 메인 (140줄) |
| `franka_cortex/franka_cortex_extension.py` | Extension 등록 + behavior 선택 UI |
| `isaacsim.cortex.framework/cortex_world.py` | CortexWorld, Behavior, LogicalStateMonitor |
| `isaacsim.cortex.framework/dfb.py` | DfDiagnosticsMonitor (의사결정 진단) |
| `isaacsim.cortex.framework/robot.py` | CortexFranka (Cortex 전용 Franka 래퍼) |
| `isaacsim.cortex.framework/cortex_utils.py` | load_behavior_module() |

## 핵심 코드 분석

### Decider Network 로딩

```python
async def load_behavior(self, behavior):
    self.decider_network = load_behavior_module(self.behavior).make_decider_network(self.robot)
    self.decider_network.context.add_monitor(self.context_monitor.monitor)
    world.add_decider_network(self.decider_network)
```

`load_behavior_module()`은 Python 모듈을 동적으로 로드하여 `make_decider_network()` 팩토리 함수를 호출한다. 각 behavior 모듈은 독립적인 의사결정 로직을 구현하며, 런타임에 교체할 수 있다.

### CortexWorld 통합

```python
def setup_scene(self):
    self.robot = world.add_robot(add_franka_to_stage(name="franka", prim_path="/World/Franka"))

    # 조작 대상: 4색 큐브
    obs_specs = [
        CubeSpec("RedCube", [0.7, 0.0, 0.0]),
        CubeSpec("BlueCube", [0.0, 0.0, 0.7]),
        CubeSpec("YellowCube", [0.7, 0.7, 0.0]),
        CubeSpec("GreenCube", [0.0, 0.7, 0.0]),
    ]
    for i, (x, spec) in enumerate(zip(np.linspace(0.3, 0.7, len(obs_specs)), obs_specs)):
        obj = DynamicCuboid(prim_path=f"/World/Obs/{spec.name}", size=0.0515, ...)
        self.robot.register_obstacle(obj)    # 충돌 회피 대상으로 등록
```

**`CortexFranka`**: 일반 `Franka`와 달리, Cortex 전용 래퍼로 `register_obstacle()`을 통해 조작/회피 대상을 등록할 수 있다.

**큐브 배치**: 0.3m ~ 0.7m 범위에서 등간격으로 4개 큐브를 배치한다. 크기 5.15cm의 DynamicCuboid로, 물리적으로 잡고 이동할 수 있다.

### 의사결정 모니터링

```python
class ContextStateMonitor(DfDiagnosticsMonitor):
    def print_diagnostics(self, context):
        if self.diagnostic_fn:
            self.diagnostic_fn(context)

def _on_monitor_update(self, context):
    diagnostic = context.diagnostics_message if hasattr(context, "diagnostics_message") else ""
    decision_stack = "\n".join([
        f"{'  ' * i}{element}"
        for i, element in enumerate(str(i) for i in self.decider_network._decider_state.stack)
    ])
```

Decider Network의 현재 상태(어떤 행동을 실행 중인지, 의사결정 스택)를 UI에 실시간으로 표시한다. `decision_stack`은 중첩된 행동의 계층을 들여쓰기로 시각화한다.

### Physics Step

```python
def _on_physics_step(self, step_size):
    world.step(False, False)    # physics=False, render=False
```

Cortex는 `CortexWorld.step()` 내부에서 decider network를 갱신하고, 결정된 행동에 따른 모션 명령을 생성한다. `(False, False)` 파라미터는 물리/렌더링은 외부에서 관리됨을 의미한다.

## 실행 방법

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI: Isaac Examples > Cortex > Franka Cortex
# 1. Load
# 2. Behavior 드롭다운에서 행동 선택
# 3. Play → 큐브 조작 관찰
```

## Comparison: Cortex vs 다른 제어 방식

| 항목 | RMPflow / RRT | RL 정책 | Cortex (이 문서) |
|---|---|---|---|
| 추상화 수준 | 모션 레벨 | 모션 레벨 | 태스크 레벨 |
| 의사결정 | 없음 (목표 주어짐) | 암묵적 (보상 학습) | 명시적 (Decider Network) |
| 유연성 | 목표 변경만 가능 | 재학습 필요 | 행동 모듈 교체 가능 |
| 디버깅 | 경로 시각화 | Reward 분석 | 의사결정 스택 모니터링 |
| 적합한 경우 | 단순 도달 | End-to-end 최적화 | 복잡한 순차 작업 |

## Further Reading

- **FrankaCortex**: `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/.../franka_cortex/franka_cortex.py`
- **Cortex Framework**: `~/workspace/IsaacSim/exts/isaacsim.cortex.framework/`
- **CortexWorld**: `~/workspace/IsaacSim/exts/isaacsim.cortex.framework/.../cortex_world.py`
- **Behavior Modules**: `~/workspace/IsaacSim/exts/isaacsim.cortex.sample_behaviors/`
