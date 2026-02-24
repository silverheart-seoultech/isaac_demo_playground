# IMU Sensor — 관성 측정 장치

## Overview

IMU(Inertial Measurement Unit) 센서의 생성과 데이터 수집을 다루는 예제. 가속도계(accelerometer) 3축, 자이로스코프(gyroscope) 3축, 방향(orientation) 4원수(quaternion) 총 10개 값을 실시간으로 측정한다.

Extension 버전은 Ant 로봇의 중심 body에 IMU를 부착하고, Standalone 버전은 Nova Carter 이동 로봇의 바퀴에 IMU를 부착하여 주행 중 관성 데이터를 수집한다.

## Architecture

### Extension 버전 (GUI)

```
Extension (imu_sensor.py, 212줄)
    │
    ├── on_startup()
    │       └── register_example("IMU Sensor", category="Sensors")
    │
    ├── _load_scene()
    │       ├── is = acquire_imu_sensor_interface()    ← C++ 바인딩
    │       ├── subscribe_physics_step_events(_on_update)
    │       ├── open_stage("ant_colored.usd")
    │       └── IsaacSensorCreateImuSensor(
    │               parent="/Ant/Sphere",
    │               sensor_period=-1.0
    │           )
    │
    └── _on_update(dt)
            └── reading = is.get_sensor_reading("/Ant/Sphere/sensor")
                ├── lin_acc_x/y/z  → 슬라이더 0-2 (m/s²)
                ├── ang_vel_x/y/z  → 슬라이더 3-5 (rad/s)
                └── orientation[0-3] → 슬라이더 6-9 (quaternion)
```

### Standalone 버전

```
imu_sensor.py (99줄)
    │
    ├── World(stage_units_in_meters=1.0)
    ├── add_reference_to_stage("nova_carter.usd")
    ├── Articulation("/World/Carter")
    │
    ├── IMUSensor(
    │       prim_path="/World/Carter/caster_wheel_left/imu_sensor",
    │       frequency=60, translation=[0,0,0]
    │   )
    │
    ├── DifferentialController(wheel_radius=0.04295, wheel_base=0.4132)
    │
    └── while running:
            actions = controller.forward(command)   # 시간 기반 주행
            carter.apply_action(actions)
            print(imu_sensor.get_current_frame())
```

## Source Files

| 파일 | 역할 |
|---|---|
| `isaacsim.sensors.physics.examples/.../imu_sensor.py` | Extension 버전 — GUI + C++ 인터페이스 (212줄) |
| `standalone_examples/api/isaacsim.sensors.physics/imu_sensor.py` | Standalone 버전 — Carter 주행 + IMU 수집 (99줄) |
| `isaacsim.sensors.physics/impl/imu_sensor.py` | IMUSensor 클래스 구현체 |

## 핵심 코드 분석

### 센서 생성

**Extension (Kit Command)**:

```python
result, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path="/sensor",
    parent="/Ant/Sphere",               # Ant의 중심 body
    sensor_period=-1.0,                  # 매 물리 스텝마다
    translation=Gf.Vec3d(0, 0, 0),      # body 중심에 배치
    orientation=Gf.Quatd(1, 0, 0, 0),   # 기본 방향 (항등 회전)
)
```

**Standalone (Python API)**:

```python
imu_sensor = IMUSensor(
    prim_path="/World/Carter/caster_wheel_left/imu_sensor",
    name="imu",
    frequency=60,                        # 60Hz 업데이트
    translation=np.array([0, 0, 0]),
)
```

`frequency=60`은 초당 60회 센서 업데이트를 의미한다. 물리 시뮬레이션이 더 높은 주파수로 실행되더라도, IMU 데이터는 60Hz로 다운샘플링된다. 이는 실제 IMU 하드웨어의 동작을 모사한다.

### IMU 데이터 구조 (10개 값)

| 인덱스 | 필드 | 단위 | 설명 |
|---|---|---|---|
| 0-2 | `lin_acc_x/y/z` | m/s² | 선형 가속도 (중력 포함) |
| 3-5 | `ang_vel_x/y/z` | rad/s | 각속도 |
| 6-9 | `orientation[0-3]` | quaternion (xyzw) | 센서의 절대 방향 |

**중력 포함**: `lin_acc`는 중력 가속도를 포함한다. 정지 상태에서 z축 가속도가 약 9.81 m/s²로 측정된다. 실제 IMU와 동일하게, 관성 프레임 기준의 가속도에서 중력을 분리하려면 별도의 보정이 필요하다.

### 데이터 읽기

**Extension (C++ 바인딩)**:

```python
reading = self._is.get_sensor_reading(self.body_path + "/sensor")
if reading.is_valid:
    acc_x = float(reading.lin_acc_x) * self.meters_per_unit
    acc_y = float(reading.lin_acc_y) * self.meters_per_unit
    acc_z = float(reading.lin_acc_z) * self.meters_per_unit
    gyro_x = float(reading.ang_vel_x)      # 각속도는 단위 변환 불필요
    gyro_y = float(reading.ang_vel_y)
    gyro_z = float(reading.ang_vel_z)
    orient = reading.orientation[0:4]        # quaternion (x, y, z, w)
```

가속도는 `meters_per_unit`로 단위 변환이 필요하지만, 각속도(rad/s)와 방향(quaternion)은 무차원이므로 변환 불필요.

**Standalone (Python API)**:

```python
frame = imu_sensor.get_current_frame()
# frame은 dictionary:
# {
#     'lin_acc': np.array([ax, ay, az]),
#     'ang_vel': np.array([wx, wy, wz]),
#     'orientation': np.array([qx, qy, qz, qw]),
#     'time': float
# }
```

### Standalone — Carter 주행 패턴

```python
my_controller = DifferentialController(
    name="simple_control", wheel_radius=0.04295, wheel_base=0.4132
)

if i >= 0 and i < 1000:        # 전진
    command = [0.05, 0]
elif i >= 1000 and i < 1265:    # 회전 (90도)
    command = [0.0, np.pi / 12]
elif i >= 1265 and i < 2000:    # 전진
    command = [0.05, 0]
elif i == 2000:
    i = 0                        # 반복
```

Carter가 직진 → 회전 → 직진 패턴으로 주행하면서 IMU 데이터를 수집한다. 이 패턴에서:
- **직진 구간**: `lin_acc`는 초기 가속 후 거의 0, `ang_vel`은 0
- **회전 구간**: `lin_acc`에 원심 가속도 발생, `ang_vel_z`가 `π/12` rad/s
- **방향 전환 시**: `orientation` quaternion이 점진적으로 변화

이 데이터를 통해 IMU의 물리적 정확성을 검증할 수 있다.

### ArticulationActions 변환 패턴

```python
actions = ArticulationActions()
actions.joint_velocities = np.expand_dims(
    my_controller.forward(command=[0.05, 0]).joint_velocities, axis=0
)
# controller 출력: [left_vel, right_vel]  → shape (2,)
# ArticulationActions 요구: shape (1, num_dof)

joint_actions = ArticulationActions()
joint_actions.joint_velocities = np.zeros([1, my_carter.num_dof])
for j in range(len(wheel_dof_indices)):
    joint_actions.joint_velocities[0, wheel_dof_indices[j]] = actions.joint_velocities[0, j]
```

`DifferentialController`의 출력은 2개 wheel의 속도만 포함하지만, Carter는 더 많은 DOF를 가진다. 전체 DOF 크기의 zero 벡터를 만들고, wheel DOF 인덱스에만 값을 채우는 패턴이다. 이는 multi-DOF 로봇에서 특정 joint만 제어할 때의 표준 패턴이다.

## 실행 방법

**Extension (GUI)**:

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.sensors.physics.examples

# GUI: Isaac Examples > Sensors > IMU Sensor
# Load Scene → PLAY
# Shift + 좌클릭으로 Ant를 드래그하면 IMU 값 변화를 관찰
```

**Standalone**:

```bash
cd ~/workspace/IsaacSim
python standalone_examples/api/isaacsim.sensors.physics/imu_sensor.py
```

## Further Reading

- **Extension 소스**: `~/workspace/IsaacSim/exts/isaacsim.sensors.physics.examples/.../imu_sensor.py`
- **Standalone 소스**: `~/workspace/IsaacSim/standalone_examples/api/isaacsim.sensors.physics/imu_sensor.py`
- **IMUSensor 구현체**: `~/workspace/IsaacSim/exts/isaacsim.sensors.physics/.../impl/imu_sensor.py`
- **공식 문서**: https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_imu.html
