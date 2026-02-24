# Robo Party — 이종 로봇 4대 동시 운용

## Overview

Franka (매니퓰레이터), UR10 (매니퓰레이터), Kaya (홀로노믹 이동 로봇), Jetbot (디퍼렌셜 이동 로봇) 총 4종의 로봇이 하나의 씬에서 동시에 동작하는 데모. 매니퓰레이터는 큐브 스태킹을, 이동 로봇은 시간 기반 주행 명령을 수행한다.

이 데모의 핵심은 **이종 로봇(heterogeneous robot) 통합 관리**다. 각 로봇이 서로 다른 Controller 타입(IK 기반 매니퓰레이션, 홀로노믹 구동, 디퍼렌셜 구동)을 사용하지만, 단일 물리 루프에서 통합 관리된다.

## Architecture

```
BaseSample
    │
    └── RoboParty (robo_party.py, 171줄)
            │
            ├── setup_scene()
            │       ├── FrankaStacking(offset=[0, -2, 0])    → task_0
            │       ├── UR10Stacking(offset=[0.5, 0.5, 0])   → task_1
            │       ├── WheeledRobot("Kaya", position=[-1, 0, 0])
            │       └── WheeledRobot("Jetbot", position=[-1.5, -1.5, 0])
            │
            ├── setup_post_load()
            │       ├── robots[0]: Franka  → FrankaStackingController
            │       ├── robots[1]: UR10    → UR10StackingController
            │       ├── robots[2]: Kaya    → HolonomicController
            │       └── robots[3]: Jetbot  → DifferentialController
            │
            └── _on_start_party_physics_step(step_size)
                    ├── observations = world.get_observations()
                    ├── Franka:  controllers[0].forward(observations)
                    ├── UR10:    controllers[1].forward(observations, offset=[0,0,0.02])
                    ├── Kaya:    time-based holonomic commands
                    └── Jetbot:  time-based differential commands
```

## Source Files

| 파일 | 역할 |
|---|---|
| `robo_party/robo_party.py` | 데모 메인 — 4종 로봇 통합 (171줄) |
| `franka/tasks/stacking.py` | Franka Stacking Task |
| `franka/controllers/stacking_controller.py` | FrankaStackingController |
| `universal_robots/tasks/stacking.py` | UR10 Stacking Task |
| `universal_robots/controllers/stacking_controller.py` | UR10StackingController |
| `wheeled_robots/controllers/holonomic_controller.py` | HolonomicController (Kaya) |
| `wheeled_robots/controllers/differential_controller.py` | DifferentialController (Jetbot) |

## 핵심 코드 분석

### 4종 로봇 씬 구성

```python
def setup_scene(self):
    world = self.get_world()
    # 매니퓰레이터: Task API로 추가 (로봇 + 큐브 + observation 일체형)
    self._tasks.append(FrankaStacking(name="task_0", offset=np.array([0, -2, 0])))
    world.add_task(self._tasks[-1])
    self._tasks.append(UR10Stacking(name="task_1", offset=np.array([0.5, 0.5, 0])))
    world.add_task(self._tasks[-1])

    # 이동 로봇: Scene에 직접 추가 (Task 없음)
    world.scene.add(WheeledRobot(
        prim_path="/World/Kaya", name="my_kaya",
        wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
        usd_path=kaya_asset_path, position=np.array([-1, 0, 0]),
    ))
    world.scene.add(WheeledRobot(
        prim_path="/World/Jetbot", name="my_jetbot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        usd_path=jetbot_asset_path, position=np.array([-1.5, -1.5, 0]),
    ))
```

**두 가지 로봇 추가 패턴**:
- **매니퓰레이터**: `world.add_task()`로 Task를 통해 추가. Task가 로봇 + 오브젝트 + observation을 캡슐화.
- **이동 로봇**: `world.scene.add()`로 직접 추가. Task 없이 로봇만 존재하며, 별도의 observation 프레임워크를 사용하지 않는다.

이 차이는 이동 로봇이 observation 기반 피드백 제어가 아닌, 시간 기반 오픈 루프 명령으로 동작하기 때문이다.

### 4종 Controller 초기화

```python
async def setup_post_load(self):
    # Franka: RmpFlow 기반 IK + 그리퍼 제어
    self._controllers.append(FrankaStackingController(
        gripper=self._robots[0].gripper,
        robot_articulation=self._robots[0],
        picking_order_cube_names=self._tasks[0].get_cube_names(),
    ))
    # UR10: 동일한 stacking 패턴이지만 다른 kinematics
    self._controllers.append(UR10StackingController(
        gripper=self._robots[1].gripper,
        robot_articulation=self._robots[1],
        picking_order_cube_names=self._tasks[1].get_cube_names(),
    ))
    # Kaya: 3-wheel 옴니디렉셔널 (mecanum)
    kaya_setup = HolonomicRobotUsdSetup(
        robot_prim_path="/World/Kaya",
        com_prim_path="/World/Kaya/base_link/control_offset"
    )
    wheel_radius, wheel_positions, wheel_orientations, mecanum_angles, wheel_axis, up_axis = \
        kaya_setup.get_holonomic_controller_params()
    self._controllers.append(HolonomicController(
        wheel_radius=wheel_radius, wheel_positions=wheel_positions,
        wheel_orientations=wheel_orientations, mecanum_angles=mecanum_angles,
    ))
    # Jetbot: 2-wheel differential drive
    self._controllers.append(DifferentialController(
        wheel_radius=0.03, wheel_base=0.1125
    ))
```

**HolonomicRobotUsdSetup**: USD 파일에서 wheel 파라미터(반지름, 위치, 방향, mecanum 각도)를 자동으로 추출한다. 이 패턴 덕분에 홀로노믹 로봇의 kinematics 파라미터를 하드코딩하지 않아도 된다.

**DifferentialController**: `wheel_radius=0.03`, `wheel_base=0.1125`를 직접 지정한다. Jetbot은 단순한 2-wheel 구성이므로 USD에서 자동 추출하지 않는다.

### 시간 기반 이동 로봇 명령

```python
def _on_start_party_physics_step(self, step_size):
    # 매니퓰레이터: observation 기반 closed-loop 제어
    observations = self._world.get_observations()
    actions = self._controllers[0].forward(observations=observations, ...)
    actions = self._controllers[1].forward(observations=observations, end_effector_offset=[0,0,0.02])

    # 이동 로봇: 시간 기반 open-loop 명령
    t = self._world.current_time_step_index
    if 0 <= t < 500:
        self._robots[2].apply_wheel_actions(self._controllers[2].forward(command=[0.2, 0.0, 0.0]))  # Kaya: x방향
        self._robots[3].apply_wheel_actions(self._controllers[3].forward(command=[0.1, 0]))          # Jetbot: 전진
    elif 500 <= t < 1000:
        self._robots[2].apply_wheel_actions(self._controllers[2].forward(command=[0, 0.2, 0.0]))     # Kaya: y방향
        self._robots[3].apply_wheel_actions(self._controllers[3].forward(command=[0.0, np.pi/10]))   # Jetbot: 회전
    elif 1000 <= t < 1500:
        self._robots[2].apply_wheel_actions(self._controllers[2].forward(command=[0, 0.0, 0.06]))    # Kaya: 제자리 회전
        self._robots[3].apply_wheel_actions(self._controllers[3].forward(command=[0.1, 0]))          # Jetbot: 전진
```

### Controller command 비교

| Controller | command 형식 | 설명 |
|---|---|---|
| HolonomicController (Kaya) | `[vx, vy, omega]` | 3-DOF: 전후, 좌우, 회전을 독립 제어 |
| DifferentialController (Jetbot) | `[v_linear, omega]` | 2-DOF: 전진 속도 + 회전 속도 |
| FrankaStackingController | `forward(observations)` | Observation 기반 상태 머신 |
| UR10StackingController | `forward(observations, offset)` | 동일, `end_effector_offset=[0,0,0.02]` 추가 |

UR10의 `end_effector_offset=[0,0,0.02]`는 그리퍼 형상 차이를 보상한다. UR10 그리퍼가 Franka보다 약간 길기 때문에 2cm 오프셋을 적용한다.

### 홀로노믹 vs 디퍼렌셜 구동 비교

**Kaya (Holonomic)**: 3개의 mecanum wheel로 구성. `[vx, vy, omega]` 3-DOF 명령을 받아, 전후/좌우/회전을 독립적으로 제어할 수 있다. Non-holonomic constraint가 없으므로, 제자리에서 횡이동(strafing)이 가능하다.

```python
HolonomicController.forward(command=[vx, vy, omega])
# → wheel_velocities = J^{-1} @ [vx, vy, omega]
# J: wheel 배치에 따른 Jacobian (3x3)
```

**Jetbot (Differential)**: 2개의 독립 구동 wheel로 구성. `[v, omega]` 2-DOF 명령만 받을 수 있다. Non-holonomic constraint 때문에 횡이동이 불가능하고, 방향 전환 시 반드시 회전이 필요하다.

```python
DifferentialController.forward(command=[v, omega])
# → left_vel  = (v - omega * wheel_base/2) / wheel_radius
# → right_vel = (v + omega * wheel_base/2) / wheel_radius
```

## 실행 방법

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI: Isaac Examples > Multi Robot > Robo Party
# Load → Start Party 버튼 클릭
```

## Further Reading

- **RoboParty 소스**: `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/.../robo_party/robo_party.py`
- **HolonomicController**: `~/workspace/IsaacSim/exts/isaacsim.robot.wheeled_robots/...controllers/holonomic_controller.py`
- **DifferentialController**: `~/workspace/IsaacSim/exts/isaacsim.robot.wheeled_robots/...controllers/differential_controller.py`
- **HolonomicRobotUsdSetup**: `~/workspace/IsaacSim/exts/isaacsim.robot.wheeled_robots/.../holonomic_robot_usd_setup.py`
- **관련 데모**: [01-robo-factory](../01-robo-factory/) — 동종 로봇 4대 병렬, [03-bin-filling](../03-bin-filling/) — UR10 단일 pick-place
