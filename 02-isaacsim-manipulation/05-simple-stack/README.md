# Robo Factory — 4x Franka 병렬 스태킹

## Overview

4대의 Franka Emika Panda가 동시에 큐브 스태킹(쌓기) 작업을 수행하는 데모. 각 로봇이 독립적인 `Stacking` Task와 `StackingController`를 가지며, y축 방향으로 2m 간격으로 배치됩니다. Isaac Sim의 **다중 Task 병렬 실행** 패턴을 보여줍니다.

이 데모는 `04-isaacsim-multirobot/01-robo-factory/`와 동일한 코드를 분석합니다. 핵심은 단일 물리 루프에서 여러 독립적인 Task-Controller 쌍을 관리하는 구조입니다.

## Architecture

```
BaseSample
    │
    └── RoboFactory (robo_factory.py)
            │
            ├── setup_scene()
            │       └── for i in range(4):
            │               world.add_task(Stacking(offset=[0, i*2-3, 0]))
            │
            ├── setup_post_load()
            │       └── for i in range(4):
            │               StackingController(gripper, robot, cube_names)
            │
            └── _on_start_factory_physics_step()
                    └── for i in range(4):
                            actions = controllers[i].forward(observations)
                            articulation_controllers[i].apply_action(actions)
```

## Source Files

| 파일 | 역할 |
|---|---|
| `robo_factory/robo_factory.py` | 데모 메인 (83줄) |
| `isaacsim.robot.manipulators.examples/franka/tasks/stacking.py` | Stacking Task (큐브 + Franka 씬) |
| `isaacsim.robot.manipulators.examples/franka/controllers/stacking_controller.py` | StackingController (pick-place 상태 머신) |

## 핵심 코드 분석

### 4-Task 병렬 구성

```python
self._num_of_tasks = 4

def setup_scene(self):
    for i in range(self._num_of_tasks):
        task = Stacking(name="task" + str(i), offset=np.array([0, (i * 2) - 3, 0]))
        world.add_task(task)
```

`offset`으로 각 Task를 y축 방향으로 분리합니다: `[-3, -1, 1, 3]`m. 각 Stacking Task가 독립적인 Franka + 큐브 세트를 생성합니다.

### 통합 물리 루프

```python
def _on_start_factory_physics_step(self, step_size):
    observations = self._world.get_observations()    # 전체 씬의 observation
    for i in range(self._num_of_tasks):
        actions = self._controllers[i].forward(
            observations=observations,
            end_effector_offset=np.array([0, 0, 0])
        )
        self._articulation_controllers[i].apply_action(actions)
```

**단일 `get_observations()` 호출**: 모든 로봇과 큐브의 상태를 한 번에 가져옵니다. 각 Controller가 자신의 로봇/큐브에 해당하는 observation만 참조합니다.

**순차 action 적용**: 4대 로봇의 action을 순서대로 계산하고 적용합니다. 물리 엔진은 모든 action이 적용된 후 한 번에 시뮬레이션합니다.

### StackingController

`StackingController`는 내부적으로 상태 머신(state machine)을 구현합니다:
1. **Pick**: 큐브 위치로 EE 이동 → 하강 → 그리퍼 닫기
2. **Place**: 목표 스태킹 위치로 이동 → 그리퍼 열기
3. 다음 큐브로 반복

`picking_order_cube_names`로 큐브 스태킹 순서를 지정합니다.

## 실행 방법

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI: Isaac Examples > Manipulation > Simple Stack
# Load → Start Stacking 버튼 클릭
```

## Further Reading

- **RoboFactory**: `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/.../robo_factory/robo_factory.py`
- **StackingController**: `~/workspace/IsaacSim/exts/isaacsim.robot.manipulators.examples/.../franka/controllers/stacking_controller.py`
- **Stacking Task**: `~/workspace/IsaacSim/exts/isaacsim.robot.manipulators.examples/.../franka/tasks/stacking.py`
