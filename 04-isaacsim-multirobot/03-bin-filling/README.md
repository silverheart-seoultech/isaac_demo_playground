# Bin Filling — UR10 나사 빈 채우기
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

UR10e 로봇이 나사(screw) 부품을 집어서 빈(bin)에 채우는 산업용 pick-and-place 데모. 단순히 오브젝트를 옮기는 것을 넘어, **런타임에 동적으로 오브젝트를 추가**하는 패턴과 **이벤트 기반 동기화**(Controller 상태에 따른 오브젝트 스폰)를 보여줍니다.

이 데모는 `02-isaacsim-manipulation/06-ur10-palletizing/`에서 다룬 UR10 pick-place의 확장으로, Controller의 특정 이벤트(event 6)에서 시뮬레이션을 일시 정지하고 나사를 추가하는 **비동기 씬 수정** 패턴에 초점을 맞춥니다.

## Architecture

```
BaseSample
    │
    └── BinFilling (bin_filling.py, 80줄)
            │
            ├── setup_scene()
            │       └── world.add_task(BinFillingTask("bin_filling"))
            │
            ├── setup_post_load()
            │       ├── ur10 = scene.get_object(task.robot_name)
            │       ├── controller = PickPlaceController(gripper, robot)
            │       └── articulation_controller = ur10.get_articulation_controller()
            │
            └── _on_fill_bin_physics_step(step_size)
                    │
                    ├── observations = world.get_observations()
                    ├── actions = controller.forward(
                    │       picking_position, placing_position,
                    │       current_joint_positions,
                    │       end_effector_offset=[0, -0.098, 0.03],
                    │       end_effector_orientation=euler_to_quat([π, 0, π/2])
                    │   )
                    ├── if event==6 && !added_screws:
                    │       controller.pause()
                    │       task.add_screws(20)     ← 런타임 오브젝트 스폰
                    │       added_screws = True
                    └── articulation_controller.apply_action(actions)
```

## Source Files

| 파일 | 역할 |
|---|---|
| `bin_filling/bin_filling.py` | 데모 메인 — 이벤트 기반 나사 추가 (80줄) |
| `universal_robots/tasks/bin_filling.py` | BinFillingTask — UR10 + bin + screw 씬, observation |
| `universal_robots/controllers/pick_place_controller.py` | PickPlaceController — pick-place 상태 머신 |

## 핵심 코드 분석

### PickPlaceController의 이벤트 기반 실행

```python
def _on_fill_bin_physics_step(self, step_size):
    observations = self._world.get_observations()
    actions = self._controller.forward(
        picking_position=observations[self._task_params["bin_name"]["value"]]["position"],
        placing_position=observations[self._task_params["bin_name"]["value"]]["target_position"],
        current_joint_positions=observations[self._task_params["robot_name"]["value"]]["joint_positions"],
        end_effector_offset=np.array([0, -0.098, 0.03]),
        end_effector_orientation=euler_angles_to_quat(np.array([np.pi, 0, np.pi / 2.0])),
    )
```

**end_effector_offset**: `[0, -0.098, 0.03]` — UR10 그리퍼의 실제 TCP(Tool Center Point)와 flange 좌표계 간의 오프셋. y축 -9.8cm, z축 +3cm.

**end_effector_orientation**: `euler_to_quat([π, 0, π/2])` — 그리퍼가 아래를 향하도록(z축 반전) + 90도 회전. UR10 그리퍼의 마운팅 방향을 보상합니다.

### 런타임 오브젝트 스폰

```python
if not self._added_screws and self._controller.get_current_event() == 6 \
   and not self._controller.is_paused():
    self._controller.pause()
    self._ur10_task.add_screws(screws_number=20)
    self._added_screws = True
```

**이벤트 6**: PickPlaceController의 내부 상태 머신에서 place 동작이 완료된 시점. 이 시점에서:

1. `controller.pause()` — Controller를 일시 정지하여 추가 action 생성을 막는다
2. `task.add_screws(20)` — BinFillingTask가 bin 내부에 20개 나사를 스폰합니다
3. `added_screws = True` — 플래그로 중복 스폰을 방지합니다

이 패턴은 실제 산업 시나리오에서 "로봇이 작업 완료 후 새 부품이 컨베이어에 공급되는" 상황을 시뮬레이션합니다.

### 완료 조건

```python
if self._controller.is_done():
    self._world.pause()
```

Controller의 pick-place 시퀀스가 완전히 끝나면 시뮬레이션을 정지합니다. 무한 루프가 아닌 단발 실행 패턴.

### RoboFactory/RoboParty와의 차이

| 항목 | RoboFactory | BinFilling |
|---|---|---|
| 로봇 수 | 4대 (동종) | 1대 |
| Controller 타입 | StackingController | PickPlaceController |
| observation 전달 | `forward(observations)` | `forward(picking_pos, placing_pos, joint_pos, offset, orientation)` |
| 오브젝트 관리 | 씬 초기화 시 고정 | 런타임에 동적 추가 |
| 실행 모드 | 연속 (모든 큐브 스택) | 단발 (1회 pick-place + screw 추가) |

**PickPlaceController vs StackingController**: StackingController는 내부에 picking_position/placing_position을 자동으로 계산하지만, PickPlaceController는 외부에서 매 스텝 전달받는다. 이 인터페이스 차이 때문에 BinFilling에서는 observation에서 직접 position을 추출하여 전달합니다.

## 실행 방법

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI: Isaac Examples > Multi Robot > Bin Filling
# Load → Fill Bin 버튼 클릭
```

**Standalone 버전**:

```bash
cd ~/workspace/IsaacSim
python standalone_examples/api/isaacsim.robot.manipulators/universal_robots/bin_filling.py
```

## Further Reading

- **BinFilling 소스**: `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/.../bin_filling/bin_filling.py`
- **BinFillingTask**: `~/workspace/IsaacSim/exts/isaacsim.robot.manipulators.examples/.../universal_robots/tasks/bin_filling.py`
- **PickPlaceController**: `~/workspace/IsaacSim/exts/isaacsim.robot.manipulators.examples/.../universal_robots/controllers/pick_place_controller.py`
- **Standalone 버전**: `~/workspace/IsaacSim/standalone_examples/api/isaacsim.robot.manipulators/universal_robots/bin_filling.py`
- **관련 데모**: [02-isaacsim-manipulation/06-ur10-palletizing](../../02-isaacsim-manipulation/06-ur10-palletizing/) — UR10 기본 팔레타이징
