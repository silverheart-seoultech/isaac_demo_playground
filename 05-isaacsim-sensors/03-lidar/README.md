# LiDAR Sensor — PhysX 및 RTX 라이다

## Overview

Isaac Sim의 두 가지 LiDAR 구현인 **PhysX LiDAR**와 **RTX LiDAR**의 생성, 구성, 데이터 수집을 다루는 예제. PhysX LiDAR는 PhysX ray-cast 기반으로 동작하며, RTX LiDAR는 GPU ray-tracing 파이프라인(RTX)을 사용하여 더 현실적인 센서 시뮬레이션을 제공합니다.

Extension 버전은 정적 환경에서 LiDAR 속성을 직접 설정하고 데이터를 시각화하며, Standalone 버전들은 Carter 이동 로봇에 LiDAR를 장착하고 주행하면서 depth/point cloud 데이터를 수집합니다.

## Architecture

### Extension — PhysX LiDAR (GUI)

```
Extension (lidar_info.py, 260줄)
    │
    ├── on_startup()
    │       └── register_example("Physx Lidar Sensor", category="Sensors")
    │
    ├── _on_spawn_lidar_button()
    │       ├── new_stage_async()
    │       ├── UsdPhysics.Scene.Define()       ← PhysX 필수
    │       ├── RangeSensorSchema.Lidar.Define("/World/Lidar")
    │       │       ├── HorizontalFov = 360°
    │       │       ├── VerticalFov = 10°
    │       │       ├── HorizontalResolution = 1°
    │       │       ├── VerticalResolution = 1°
    │       │       ├── MinRange = 0.4m
    │       │       ├── MaxRange = 100m
    │       │       └── RotationRate = 0.5 Hz
    │       └── _on_spawn_obstacles_button()     ← 충돌 큐브 생성
    │
    └── _get_info_function()
            ├── depth = li.get_depth_data(lidarPath)     ← uint16 → float
            ├── zenith = li.get_zenith_data(lidarPath)
            └── azimuth = li.get_azimuth_data(lidarPath)
```

### Standalone — PhysX LiDAR

```
rotating_lidar_physX.py (106줄)
    │
    ├── WheeledRobot("Carter", carter_v1_physx_lidar.usd)
    ├── RotatingLidarPhysX("/World/Carter/chassis_link/lidar")
    ├── DifferentialController(wheel_radius=0.24, wheel_base=0.56)
    │
    ├── my_lidar.add_depth_data_to_frame()
    ├── my_lidar.add_point_cloud_data_to_frame()
    ├── my_lidar.enable_visualization()
    │
    └── while running:
            carter.apply_wheel_actions(controller.forward(command))
            # 데이터는 get_current_frame()으로 접근
```

### Standalone — RTX LiDAR

```
rotating_lidar_rtx.py (105줄)
    │
    ├── SimulationApp({"enable_motion_bvh": True})    ← RTX 가속 필수
    ├── WheeledRobot("Carter", nova_carter.usd)
    ├── LidarRtx("/World/Carter/.../PandarXT_32_10hz")
    ├── DifferentialController(wheel_radius=0.04295, wheel_base=0.4132)
    │
    ├── my_lidar.attach_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
    ├── my_lidar.enable_visualization()
    │
    └── while running:
            print(my_lidar.get_current_frame())
            carter.apply_wheel_actions(controller.forward(command))
```

## Source Files

| 파일 | 역할 |
|---|---|
| `isaacsim.sensors.physx.examples/.../lidar_info.py` | Extension — PhysX LiDAR GUI + 데이터 시각화 (260줄) |
| `standalone/.../isaacsim.sensors.physx/rotating_lidar_physX.py` | Standalone — PhysX LiDAR + Carter (106줄) |
| `standalone/.../isaacsim.sensors.rtx/rotating_lidar_rtx.py` | Standalone — RTX LiDAR + Carter (105줄) |

## 핵심 코드 분석

### PhysX LiDAR — USD Schema로 생성

```python
# LiDAR prim 생성 (RangeSensorSchema)
self.lidar = RangeSensorSchema.Lidar.Define(stage, Sdf.Path("/World/Lidar"))

# 파라미터 설정
self.lidar.CreateHorizontalFovAttr().Set(360.0)     # 수평 시야각 (°)
self.lidar.CreateVerticalFovAttr().Set(10)           # 수직 시야각 (°)
self.lidar.CreateRotationRateAttr().Set(20.0)        # 회전 속도 (Hz)
self.lidar.CreateHorizontalResolutionAttr().Set(1.0) # 수평 해상도 (°)
self.lidar.CreateVerticalResolutionAttr().Set(1.0)   # 수직 해상도 (°)
self.lidar.CreateMinRangeAttr().Set(0.4)             # 최소 감지 거리 (m)
self.lidar.CreateMaxRangeAttr().Set(100.0)           # 최대 감지 거리 (m)

# 시각화 옵션
self.lidar.CreateHighLodAttr().Set(True)             # 모든 ray 표시
self.lidar.CreateDrawPointsAttr().Set(False)
self.lidar.CreateDrawLinesAttr().Set(True)           # ray line 표시
```

PhysX LiDAR는 USD의 `RangeSensorSchema`를 사용하여 정의합니다. **각 attribute를 반드시 `Create`한 후 `Set`해야 합니다** — attribute가 schema에 정의되어 있더라도 instance에서 생성하지 않으면 런타임 에러가 발생합니다.

### PhysX LiDAR 파라미터 분석

| 파라미터 | 값 | 물리적 의미 |
|---|---|---|
| HFov = 360° | 전방위 스캔 | 실제 Velodyne/Ouster와 동일 |
| VFov = 10° | 수직 ±5° | 좁은 수직 FOV — 평면 스캔에 적합 |
| HRes = 1° | 360개 수평 ray | 360°/1° = 360 ray/revolution |
| VRes = 1° | 11개 수직 ray | 10°/1° + 1 = 11 ray |
| MinRange = 0.4m | 최소 감지 거리 | 근거리 blind zone |
| MaxRange = 100m | 최대 감지 거리 | 100m 이상은 무응답 |
| RotationRate = 0.5Hz | 2초에 1회전 | 시각화를 위해 느리게 설정 |

**총 ray 수**: 360 × 11 = 3,960 ray/revolution. 각 ray는 PhysX의 line trace(ray-cast)를 실행하여 충돌 거리를 반환합니다.

### PhysX LiDAR — Depth 데이터 변환

```python
depth = self._li.get_depth_data(self.lidarPath)       # uint16 2D array
zenith = self._li.get_zenith_data(self.lidarPath)      # float array (°)
azimuth = self._li.get_azimuth_data(self.lidarPath)    # float array (°)

# uint16 → 실제 거리(m)로 변환
maxDepth = self.lidar.GetMaxRangeAttr().Get()
distance_m = ray * maxDepth / 65535.0
```

C++ 내부에서 depth는 uint16으로 저장됩니다. `[0, 65535]` 범위가 `[0, maxRange]`에 매핑되므로, 거리 해상도는 `maxRange / 65535 ≈ 0.0015m` (1.5mm)입니다.

### PhysX LiDAR — Python API (Standalone)

```python
my_lidar = RotatingLidarPhysX(
    prim_path="/World/Carter/chassis_link/lidar",
    name="lidar",
    translation=np.array([-0.06, 0, 0.38])    # 차체 상단에 배치
)

my_lidar.add_depth_data_to_frame()         # depth 데이터 활성화
my_lidar.add_point_cloud_data_to_frame()   # point cloud 활성화
my_lidar.enable_visualization()             # viewport에 ray 표시
```

`add_depth_data_to_frame()`, `add_point_cloud_data_to_frame()`은 필요한 데이터 타입만 선택적으로 활성화합니다. 모든 데이터를 항상 계산하면 성능 오버헤드가 크기 때문입니다.

### RTX LiDAR — GPU Ray Tracing

```python
# RTX 가속 활성화
simulation_app = SimulationApp({"headless": False, "enable_motion_bvh": True})

# RTX LiDAR 생성 — 실제 센서 모델 기반
my_lidar = LidarRtx(
    prim_path="/World/Carter/chassis_link/sensors/XT_32/PandarXT_32_10hz",
    name="lidar"
)

# RTX Annotator 연결 — point cloud 추출기
my_lidar.attach_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
my_lidar.enable_visualization()
```

**RTX LiDAR의 핵심 차이점**:

| 항목 | PhysX LiDAR | RTX LiDAR |
|---|---|---|
| Ray-cast 방식 | PhysX CPU/GPU ray-cast | RTX GPU ray-tracing |
| 센서 모델 | 수동 파라미터 설정 | 실제 센서 프로파일 (PandarXT_32, VLP-16 등) |
| 반사/투과 | 지원 안 함 | 지원 (유리, 거울 등) |
| 노이즈 모델 | 없음 | 실제 센서 노이즈 시뮬레이션 가능 |
| 성능 | 빠름 (단순 collision) | 느림 (full ray-tracing) |
| 데이터 추출 | `get_depth_data()` | Annotator 기반 |

**`enable_motion_bvh=True`**: RTX LiDAR는 Bounding Volume Hierarchy(BVH) 가속 구조를 사용하여 ray-tracing을 효율화합니다. 이 옵션을 켜지 않으면 RTX LiDAR가 정상 동작하지 않습니다.

**`PandarXT_32_10hz`**: Hesai 社의 PandarXT-32 LiDAR를 모사합니다. 32채널, 10Hz 회전 속도. USD 파일에 이미 센서 프로파일이 포함되어 있어 별도의 파라미터 설정이 불필요합니다.

### Carter 주행 패턴 (PhysX/RTX 공통)

```python
# PhysX: Carter v1 (구형)
WheeledRobot(wheel_dof_names=["left_wheel", "right_wheel"], ...)
DifferentialController(wheel_radius=0.24, wheel_base=0.56)

# RTX: Nova Carter (신형)
WheeledRobot(wheel_dof_names=["joint_wheel_left", "joint_wheel_right"], ...)
DifferentialController(wheel_radius=0.04295, wheel_base=0.4132)
```

PhysX와 RTX 예제가 서로 다른 Carter 모델을 사용합니다. wheel 파라미터(반지름, 축간 거리)가 크게 다르므로 주의.

## 실행 방법

**Extension — PhysX LiDAR (GUI)**:

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.sensors.physx.examples

# GUI: Isaac Examples > Sensors > Physx Lidar Sensor
# Load LIDAR → Load LIDAR Scene → PLAY → Show Data Stream 체크
```

**Standalone — PhysX LiDAR**:

```bash
cd ~/workspace/IsaacSim
python standalone_examples/api/isaacsim.sensors.physx/rotating_lidar_physX.py
```

**Standalone — RTX LiDAR**:

```bash
cd ~/workspace/IsaacSim
python standalone_examples/api/isaacsim.sensors.rtx/rotating_lidar_rtx.py
```

## PhysX vs RTX LiDAR 선택 기준

- **학습 데이터 대량 생성** (속도 우선) → PhysX LiDAR
- **Sim-to-Real 전이** (정확도 우선) → RTX LiDAR
- **실시간 SLAM/Navigation 테스트** → PhysX LiDAR
- **센서 노이즈 분석** → RTX LiDAR
- **커스텀 스캔 패턴** → PhysX LiDAR (파라미터 자유도 높음)
- **실제 센서 모델 재현** → RTX LiDAR (프로파일 기반)

## Further Reading

- **Extension 소스**: `~/workspace/IsaacSim/exts/isaacsim.sensors.physx.examples/.../lidar_info.py`
- **PhysX LiDAR Standalone**: `~/workspace/IsaacSim/standalone_examples/api/isaacsim.sensors.physx/rotating_lidar_physX.py`
- **RTX LiDAR Standalone**: `~/workspace/IsaacSim/standalone_examples/api/isaacsim.sensors.rtx/rotating_lidar_rtx.py`
- **공식 문서 (PhysX)**: https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physx_lidar.html
