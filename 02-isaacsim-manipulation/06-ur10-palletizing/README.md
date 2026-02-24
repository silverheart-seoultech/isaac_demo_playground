# UR10 Bin Filling — Pick-Place + 동적 오브젝트 생성
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

Universal Robots UR10이 나사(screw)를 빈(bin)에서 집어 다른 빈으로 옮기는 pick-and-place 데모. 핵심 특징은 **동적 오브젝트 생성**: 작업 중간에 20개의 나사를 런타임에 추가하는 패턴을 보여줍니다.

Franka 기반 데모들과 달리 UR10(6 DOF 산업용 로봇)과 `PickPlaceController`를 사용하며, end-effector orientation과 offset을 명시적으로 지정합니다.

## Architecture

```
BaseSample
    │
    └── BinFilling (bin_filling.py)
            │
            ├── setup_scene() → BinFillingTask (UR10 + 2 bins)
            ├── setup_post_load() → PickPlaceController(gripper, robot)
            │
            └── _on_fill_bin_physics_step()
                    ├── controller.forward(picking_pos, placing_pos, ...)
                    │
                    ├── if event == 6 (placing 완료):
                    │       task.add_screws(screws_number=20)  ← 동적 생성
                    │
                    └── if controller.is_done(): world.pause()
```

## Source Files

| 파일 | 역할 |
|---|---|
| `bin_filling/bin_filling.py` | 데모 메인 (80줄) |
| `isaacsim.robot.manipulators.examples/universal_robots/tasks/bin_filling.py` | BinFillingTask |
| `isaacsim.robot.manipulators.examples/universal_robots/controllers/pick_place_controller.py` | PickPlaceController |

## 핵심 코드 분석

### Pick-Place 제어 루프

```python
def _on_fill_bin_physics_step(self, step_size):
    observations = self._world.get_observations()
    actions = self._controller.forward(
        picking_position=observations[bin_name]["position"],
        placing_position=observations[bin_name]["target_position"],
        current_joint_positions=observations[robot_name]["joint_positions"],
        end_effector_offset=np.array([0, -0.098, 0.03]),
        end_effector_orientation=euler_angles_to_quat(np.array([np.pi, 0, np.pi / 2.0])),
    )
```

**`end_effector_offset`**: EE 좌표계에서의 오프셋 `[0, -0.098, 0.03]`m. 그리퍼 끝단과 EE 프레임 원점 사이의 물리적 차이를 보정합니다.

**`end_effector_orientation`**: `[π, 0, π/2]` (Euler angles). 그리퍼가 아래를 향하도록(π = 180° 회전) 하고, z축 기준 90° 회전하여 빈 방향에 맞춥니다.

### 동적 나사 생성

```python
if not self._added_screws and self._controller.get_current_event() == 6 and not self._controller.is_paused():
    self._controller.pause()
    self._ur10_task.add_screws(screws_number=20)
    self._added_screws = True
```

Controller의 이벤트 번호 6(placing 완료)에서 일시 정지 → 20개 나사 추가 → 재개. 이 패턴은 시뮬레이션 중 환경을 동적으로 변경하는 방법을 보여줍니다.

**이벤트 기반 제어**: `PickPlaceController`는 내부 상태 머신의 현재 이벤트를 `get_current_event()`로 노출합니다. 외부에서 특정 이벤트에 맞춰 환경을 조작할 수 있습니다.

### 완료 감지

```python
if self._controller.is_done():
    self._world.pause()
```

모든 pick-place 사이클이 완료되면 시뮬레이션을 자동으로 일시정지합니다.

## 실행 방법

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI: Isaac Examples > Manipulation > Bin Filling
# Load → Fill Bin 버튼 클릭
```

## Comparison: Franka vs UR10

| 항목 | Franka (Follow Target 등) | UR10 (이 문서) |
|---|---|---|
| DOF | 7 + 2 (그리퍼) | 6 + 그리퍼 |
| 제조사 | Franka Emika | Universal Robots |
| 특성 | 연구용, 토크 센서 내장 | 산업용, 협동로봇 |
| Controller | RMPflow / StackingController | PickPlaceController |
| EE Offset | 없음 (기본) | 명시적 지정 필요 |

## Further Reading

- **BinFilling**: `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/.../bin_filling/bin_filling.py`
- **PickPlaceController (UR10)**: `~/workspace/IsaacSim/exts/isaacsim.robot.manipulators.examples/.../universal_robots/controllers/pick_place_controller.py`
- **BinFillingTask**: `~/workspace/IsaacSim/exts/isaacsim.robot.manipulators.examples/.../universal_robots/tasks/bin_filling.py`
