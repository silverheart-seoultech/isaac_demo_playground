# Contact Sensor — 접촉력 측정
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

PhysX 기반 접촉 센서(Contact Sensor)의 생성, 구성, 데이터 수집을 다루는 예제. Ant 로봇의 4개 발(foot)에 접촉 센서를 부착하여, 각 발이 지면에 가하는 힘을 실시간으로 측정합니다.

Contact Sensor는 지정된 구형(spherical) 영역 내에서 body surface에 가해지는 모든 힘의 합(net force)을 반환합니다. Isaac Sim에서 가장 기본적인 센서 타입으로, 보행 로봇의 발 접촉 감지, 그리퍼의 파지력 측정 등에 활용됩니다.

## Architecture

### Extension 버전 (GUI)

```
Extension (contact_sensor.py, 178줄)
    │
    ├── on_startup()
    │       └── register_example("Contact Sensor", category="Sensors")
    │
    ├── _load_scene()
    │       ├── cs = acquire_contact_sensor_interface()    ← C++ 바인딩
    │       ├── subscribe_physics_step_events(_on_update)
    │       ├── open_stage("ant_colored.usd")
    │       └── for i in range(4):
    │               IsaacSensorCreateContactSensor(
    │                   parent=leg_paths[i],
    │                   radius=0.12, min_threshold=0, max_threshold=10000000
    │               )
    │
    └── _on_update(dt)
            └── for i in range(4):
                    reading = cs.get_sensor_reading(leg_paths[i] + "/sensor")
                    if reading.is_valid:
                        display(reading.value * meters_per_unit)
```

### Standalone 버전

```
contact_sensor.py (83줄)
    │
    ├── World(stage_units_in_meters=1.0)
    ├── add_reference_to_stage("ant.usd", "/World/Ant")
    ├── Articulation("/World/Ant/torso")
    │
    ├── for i in range(4):
    │       ContactSensor(
    │           prim_path="/World/Ant/{foot}/contact_sensor",
    │           radius=0.1, translation=translations[i]
    │       )
    │
    ├── ant_sensors[0].add_raw_contact_data_to_frame()
    │
    └── while running:
            world.step(render=True)
            print(ant_sensors[0].get_current_frame())
```

## Source Files

| 파일 | 역할 |
|---|---|
| `isaacsim.sensors.physics.examples/.../contact_sensor.py` | Extension 버전 — GUI + C++ 인터페이스 직접 사용 (178줄) |
| `standalone_examples/api/isaacsim.sensors.physics/contact_sensor.py` | Standalone 버전 — Python API 사용 (83줄) |
| `isaacsim.sensors.physics/impl/contact_sensor.py` | ContactSensor 클래스 구현체 |

## 핵심 코드 분석

### 센서 생성 — 두 가지 방법

**방법 1: Kit Command (Extension)**

```python
result, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateContactSensor",
    path="/sensor",
    parent=self.leg_paths[i],          # "/Ant/Arm_01/Lower_Arm"
    min_threshold=0,
    max_threshold=10000000,
    color=self.color[i],
    radius=0.12,
    sensor_period=-1,                   # -1 = 매 물리 스텝마다 업데이트
    translation=self.sensor_offsets[i], # Gf.Vec3d(40, 0, 0)
)
```

**방법 2: Python API (Standalone)**

```python
ContactSensor(
    prim_path="/World/Ant/right_back_foot/contact_sensor",
    name="ant_contact_sensor_0",
    min_threshold=0,
    max_threshold=10000000,
    radius=0.1,
    translation=np.array([0.38202, -0.40354, -0.0887]),
)
```

Kit Command는 USD stage에 직접 prim을 생성하는 저수준 방식이고, Python API(`ContactSensor` 클래스)는 이를 래핑한 고수준 인터페이스다. Standalone 스크립트에서는 Python API가 더 적합합니다.

### 센서 파라미터

| 파라미터 | 값 | 설명 |
|---|---|---|
| `radius` | 0.12 (ext) / 0.1 (standalone) | 접촉 감지 구형 영역의 반지름 (m) |
| `min_threshold` | 0 | 최소 감지 력 (N). 0이면 모든 접촉 감지 |
| `max_threshold` | 10000000 | 최대 감지 력 (N). 센서 saturation 값 |
| `sensor_period` | -1 | 업데이트 주기. -1 = 물리 스텝과 동기 |
| `translation` | 발 끝 좌표 | 센서의 로컬 좌표계 오프셋 |

`radius`는 센서의 감지 범위를 결정합니다. 이 구형 영역이 body surface와 교차하는 부분의 접촉력만 합산합니다. radius가 너무 작으면 접촉을 놓치고, 너무 크면 인접 body의 접촉까지 잡힙니다.

### 데이터 읽기

**Extension (C++ 바인딩)**:

```python
self._cs = _sensor.acquire_contact_sensor_interface()
reading = self._cs.get_sensor_reading(self.leg_paths[i] + "/sensor")
if reading.is_valid:
    force_newtons = reading.value * self.meters_per_unit
```

`acquire_contact_sensor_interface()`는 PhysX의 C++ Contact Sensor 인터페이스에 대한 Python 바인딩을 획득합니다. `reading.value`는 kg⋅m⋅s⁻² 단위(= N)이며, stage의 meters_per_unit을 곱해서 일관된 단위로 변환합니다.

**Standalone (Python API)**:

```python
ant_sensors[0].add_raw_contact_data_to_frame()
# ...
frame = ant_sensors[0].get_current_frame()
```

`add_raw_contact_data_to_frame()`을 호출하면 매 프레임에 raw contact 데이터(접촉점, 법선, 충격량 등)가 포함됩니다. `get_current_frame()`은 dictionary로 센서 데이터를 반환합니다.

### Ant 로봇의 센서 배치

```python
leg_paths = ["/Ant/Arm_{:02d}/Lower_Arm".format(i + 1) for i in range(4)]
# → ["/Ant/Arm_01/Lower_Arm", "/Ant/Arm_02/Lower_Arm", ...]

# Standalone에서의 foot 이름
ant_foot_prim_names = ["right_back_foot", "left_back_foot", "front_right_foot", "front_left_foot"]

# 각 발의 위치 오프셋
translations = np.array([
    [0.38202, -0.40354, -0.0887],
    [-0.4, -0.40354, -0.0887],
    [-0.4, 0.4, -0.0887],
    [0.4, 0.4, -0.0887]
])
```

Extension 버전은 `Arm_XX/Lower_Arm` 하위에 센서를 부착하고, Standalone 버전은 `{foot_name}` 하위에 부착합니다. 같은 물리적 위치를 다른 USD 경로로 참조하는 것인데, 이는 Extension과 Standalone에서 사용하는 USD 파일이 약간 다르기 때문입니다 (`ant_colored.usd` vs `ant.usd`).

## 실행 방법

**Extension (GUI)**:

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.sensors.physics.examples

# GUI: Isaac Examples > Sensors > Contact Sensor
# Load Scene → PLAY
# Shift + 좌클릭으로 Ant를 드래그하면 센서 값 변화를 관찰할 수 있다
```

**Standalone**:

```bash
cd ~/workspace/IsaacSim
python standalone_examples/api/isaacsim.sensors.physics/contact_sensor.py
```

## RL 환경에서의 활용

Contact Sensor는 Isaac Lab의 보행 환경에서 핵심적으로 사용됩니다:

- **발 접촉 감지**: `feet_air_time` reward — 발이 공중에 있는 시간을 측정하여 자연스러운 보행 패턴 유도
- **접촉력 기반 종료**: 과도한 접촉력이 감지되면 에피소드 종료
- **Observation 구성**: 접촉 상태(binary)를 관측 벡터에 포함하여 정책이 발 상태를 인식

Isaac Lab에서는 `ContactSensorCfg`로 선언적 구성이 가능합니다:

```python
contact_forces = ContactSensorCfg(
    prim_path="{ENV_REGEX_NS}/Robot/.*_foot",
    update_period=0.0,
    history_length=3,
)
```

## Further Reading

- **Extension 소스**: `~/workspace/IsaacSim/exts/isaacsim.sensors.physics.examples/.../contact_sensor.py`
- **Standalone 소스**: `~/workspace/IsaacSim/standalone_examples/api/isaacsim.sensors.physics/contact_sensor.py`
- **ContactSensor 구현체**: `~/workspace/IsaacSim/exts/isaacsim.sensors.physics/.../impl/contact_sensor.py`
- **공식 문서**: https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_contact.html
