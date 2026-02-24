# Robo Factory — 4x Franka 병렬 스태킹
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

4대의 Franka Emika Panda가 동시에 큐브 스태킹(쌓기) 작업을 수행하는 데모. 각 로봇이 독립적인 `Stacking` Task와 `StackingController`를 가지며, y축 방향으로 2m 간격으로 배치됩니다. Isaac Sim의 **다중 Task 병렬 실행** 패턴을 보여주는 핵심 예제입니다.

단일 물리 루프(`_on_physics_step`)에서 4개의 독립적인 Task-Controller 쌍을 동기적으로 실행하는 구조로, 다중 로봇 시뮬레이션의 가장 기본적인 스케일링 패턴을 학습할 수 있습니다.

## Architecture

```
BaseSample
    │
    └── RoboFactory (robo_factory.py, 83줄)
            │
            ├── setup_scene()
            │       └── for i in range(4):
            │               world.add_task(Stacking(offset=[0, i*2-3, 0]))
            │
            ├── setup_post_load()
            │       └── for i in range(4):
            │               robots[i] = scene.get_object(task.robot_name)
            │               controllers[i] = StackingController(gripper, robot, cube_names)
            │               articulation_controllers[i] = robot.get_articulation_controller()
            │
            └── _on_start_factory_physics_step(step_size)
                    │
                    ├── observations = world.get_observations()   ← 단일 호출
                    └── for i in range(4):
                            actions = controllers[i].forward(observations)
                            articulation_controllers[i].apply_action(actions)
```

### 핵심 설계: 단일 Observation → 다중 Controller

`get_observations()`를 **한 번만** 호출하여 씬 전체의 상태를 가져오고, 각 Controller가 자신의 로봇/큐브에 해당하는 key만 참조합니다. 이 패턴은 observation 수집 비용을 O(N)에서 O(1)로 줄입니다.

## Source Files

| 파일 | 역할 |
|---|---|
| `robo_factory/robo_factory.py` | 데모 메인 — 4-Task 생성, 통합 물리 루프 (83줄) |
| `franka/tasks/stacking.py` | Stacking Task — Franka + 큐브 세트 씬 구성, observation 제공 |
| `franka/controllers/stacking_controller.py` | StackingController — pick-place 상태 머신 |

## 핵심 코드 분석

### 4-Task 공간 분리

```python
self._num_of_tasks = 4

def setup_scene(self):
    for i in range(self._num_of_tasks):
        task = Stacking(name="task" + str(i), offset=np.array([0, (i * 2) - 3, 0]))
        world.add_task(task)
```

`offset`으로 각 Task를 y축 방향으로 분리합니다: `[-3, -1, 1, 3]`m. 각 Stacking Task가 독립적인 Franka + 큐브 세트를 생성하므로, 4개의 완전히 독립적인 workspace가 만들어집니다.

Task 간 물리적 간섭이 없도록 2m 간격을 두는데, Franka의 reach 범위(약 0.855m)를 고려하면 workspace 겹침이 발생하지 않습니다.

### 통합 물리 루프

```python
def _on_start_factory_physics_step(self, step_size):
    observations = self._world.get_observations()    # 전체 씬 상태 1회 수집
    for i in range(self._num_of_tasks):
        actions = self._controllers[i].forward(
            observations=observations,
            end_effector_offset=np.array([0, 0, 0])
        )
        self._articulation_controllers[i].apply_action(actions)
```

**순차 action 계산 → 일괄 물리 시뮬레이션**: 4대 로봇의 action을 순서대로 계산하고 적용합니다. `apply_action()`은 PhysX에 명령을 큐잉할 뿐, 실제 물리 스텝은 이 콜백 함수가 끝난 후 한 번에 실행됩니다. 따라서 4대 로봇의 action이 동일한 물리 타임스텝에서 동시에 적용됩니다.

### StackingController 상태 머신

각 Controller는 내부적으로 다음 상태를 순환합니다:

```
PICK: EE를 큐브 위로 이동 → 하강 → 그리퍼 닫기
PLACE: 스태킹 위치로 이동 → 그리퍼 열기
NEXT: 다음 큐브 선택 → PICK으로 복귀
```

`picking_order_cube_names`로 큐브 스태킹 순서를 지정합니다. 각 Controller가 독립적인 상태를 유지하므로, 4대 로봇이 서로 다른 진행 상태에 있을 수 있습니다.

### 02-isaacsim-manipulation/05-simple-stack과의 관계

`05-simple-stack/`은 이 RoboFactory의 단일-Task 버전이 아니라, **동일한 코드**입니다. 차이점은 분석 관점:

- `05-simple-stack/`: StackingController의 pick-place 상태 머신 자체에 초점
- `01-robo-factory/`: 다중 Task 병렬화 패턴에 초점

## 실행 방법

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI: Isaac Examples > Multi Robot > Simple Stack
# Load → Start Stacking 버튼 클릭
```

## 확장 포인트

- `_num_of_tasks`를 변경하면 로봇 수를 늘릴 수 있습니다. GPU 메모리가 허용하는 한 수십 대까지 스케일링 가능.
- offset 간격을 줄이면 로봇 간 물리적 간섭이 발생합니다 — 충돌 회피가 필요한 시나리오를 테스트할 때 활용.
- Isaac Lab의 병렬 환경(4096+ 동시 실행)은 이 패턴의 GPU 가속 버전입니다.

## Further Reading

- **RoboFactory 소스**: `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/.../robo_factory/robo_factory.py`
- **Stacking Task**: `~/workspace/IsaacSim/exts/isaacsim.robot.manipulators.examples/.../franka/tasks/stacking.py`
- **StackingController**: `~/workspace/IsaacSim/exts/isaacsim.robot.manipulators.examples/.../franka/controllers/stacking_controller.py`
- **관련 데모**: [02-robo-party](../02-robo-party/) — 이종 로봇(Franka + UR10 + Kaya + Jetbot) 조합
