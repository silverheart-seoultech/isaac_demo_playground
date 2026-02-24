# Camera Sensor — RGB/Depth 카메라
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

Isaac Sim의 Camera 센서를 사용하여 RGB 이미지, depth map, motion vector를 캡처하고, 2D↔3D 좌표 변환(projection/backprojection)을 수행하는 예제. 256×256 해상도, 20Hz 주기로 동작하며, `matplotlib`으로 캡처 결과를 저장합니다.

Camera 센서는 Isaac Sim의 Hydra 렌더링 파이프라인을 통해 동작하며, USD Camera prim으로 정의됩니다. RL 환경에서 vision-based policy의 observation으로 활용하거나, synthetic data generation 파이프라인의 핵심 컴포넌트로 사용됩니다.

## Architecture

```
camera.py (100줄)
    │
    ├── World(stage_units_in_meters=1.0)
    ├── DynamicCuboid("/new_cube_2", position=[5, 3, 1], color=RED)
    ├── DynamicCuboid("/new_cube_3", position=[-5, 1, 3], color=BLUE, linear_velocity=[0,0,0.4])
    │
    ├── Camera(
    │       prim_path="/World/camera",
    │       position=[0, 0, 25],              ← 높은 위치에서 아래를 봄
    │       frequency=20,                      ← 20Hz 캡처
    │       resolution=(256, 256),
    │       orientation=euler_to_quat([0, 90, 0])  ← 90° 피치 (하방)
    │   )
    │
    ├── camera.initialize()
    ├── camera.add_motion_vectors_to_frame()
    │
    └── while running:
            world.step(render=True)
            print(camera.get_current_frame())
            if (i+1) % 100 == 0:
                # 2D ↔ 3D 좌표 변환
                points_2d = camera.get_image_coords_from_world_points(world_points)
                points_3d = camera.get_world_points_from_image_coords(points_2d, depths)
                # 이미지 저장
                plt.imshow(camera.get_rgba()[:, :, :3])
                plt.savefig(f"camera.frame{i:03d}.png")
```

## Source Files

| 파일 | 역할 |
|---|---|
| `standalone/.../isaacsim.sensors.camera/camera.py` | Camera 생성, 데이터 수집, 좌표 변환 (100줄) |

## 핵심 코드 분석

### Camera 생성

```python
camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 25.0]),           # 25m 높이
    frequency=20,                                    # 20Hz
    resolution=(256, 256),                           # 256×256 pixels
    orientation=rot_utils.euler_angles_to_quats(
        np.array([0, 90, 0]), degrees=True           # pitch 90° → 아래를 봄
    ),
)
```

**`frequency=20`**: 물리 시뮬레이션 주파수와 독립적으로, 카메라 이미지는 20Hz로 캡처됩니다. 물리가 더 빈번하게 실행되어도 카메라 데이터는 초당 20프레임만 갱신됩니다. 이는 실제 카메라의 프레임 레이트를 모사합니다.

**`orientation`**: Euler 각도 `[0, 90, 0]`는 pitch 90° 회전으로, 카메라가 정면 대신 아래를 향합니다. z=25m 높이에서 하방을 보는 bird's-eye view 구성.

### 초기화 순서 (중요)

```python
my_world.reset()         # 1. World 초기화
camera.initialize()      # 2. Camera 초기화 (반드시 reset 후)
```

`camera.initialize()`는 `world.reset()` **이후에** 호출해야 합니다. `reset()`이 PhysX와 렌더링 파이프라인을 초기화하며, 그 후에 Camera가 렌더링 Annotator에 연결됩니다.

### 데이터 타입

```python
camera.add_motion_vectors_to_frame()     # motion vector 활성화

# 기본 데이터
frame = camera.get_current_frame()
# frame 구조:
# {
#     'rgba': np.array shape (256, 256, 4),      # RGBA 이미지
#     'depth': np.array shape (256, 256),         # depth map (m)
#     'motion_vectors': np.array shape (256, 256, 4),  # optical flow
#     'time': float
# }

# RGB만 추출
rgba = camera.get_rgba()          # shape (256, 256, 4)
rgb = rgba[:, :, :3]              # alpha 채널 제거
```

**Motion Vector**: 각 픽셀의 이전 프레임 대비 2D 이동량. optical flow의 ground truth로 활용할 수 있습니다. `add_motion_vectors_to_frame()`으로 명시적 활성화가 필요합니다.

### 2D ↔ 3D 좌표 변환

```python
# 3D world → 2D image
points_2d = camera.get_image_coords_from_world_points(
    np.array([cube_3.get_world_pose()[0], cube_2.get_world_pose()[0]])
)
# 입력: world 좌표 (N, 3)
# 출력: image 좌표 (N, 2) — 픽셀 좌표

# 2D image → 3D world (depth 필요)
points_3d = camera.get_world_points_from_image_coords(
    points_2d, np.array([24.94, 24.9])     # 각 점의 depth (m)
)
# 입력: image 좌표 (N, 2) + depth (N,)
# 출력: world 좌표 (N, 3)
```

**Projection (3D → 2D)**: 카메라의 intrinsic matrix(focal length, principal point)와 extrinsic matrix(position, orientation)를 사용하여 world 좌표를 픽셀 좌표로 변환합니다.

**Backprojection (2D → 3D)**: 픽셀 좌표 + depth 값으로 world 좌표를 복원합니다. depth가 필요한 이유는 단일 픽셀이 3D 공간의 ray를 나타내므로, 깊이 정보 없이는 고유한 3D 점을 결정할 수 없기 때문입니다.

**depth 값 `[24.94, 24.9]`**: 카메라가 z=25m에 있고 큐브가 z≈0m에 있으므로, depth는 약 25m. 정확한 값은 큐브의 높이에 따라 미세하게 다릅니다.

### 씬 구성 — 동적 오브젝트

```python
cube_2 = DynamicCuboid(
    position=np.array([5.0, 3, 1.0]),
    scale=np.array([0.6, 0.5, 0.2]),
    color=np.array([255, 0, 0]),           # RED
)

cube_3 = DynamicCuboid(
    position=np.array([-5, 1, 3.0]),
    scale=np.array([0.1, 0.1, 0.1]),
    color=np.array([0, 0, 255]),           # BLUE
    linear_velocity=np.array([0, 0, 0.4]),  # 위로 던져짐
)
```

`cube_3`에 초기 속도(`linear_velocity`)를 부여하여, 시뮬레이션 시작 시 위로 던져집니다. 중력에 의해 포물선 궤적을 그리며 떨어지는 과정에서:
- RGB 이미지에서 큐브 위치 변화를 관찰
- Motion vector에서 이동 방향/속도를 관찰
- 2D↔3D 변환의 정확도를 동적 환경에서 검증

### 이미지 저장

```python
if (i + 1) % 100 == 0:    # 100 프레임마다
    imgplot = plt.imshow(camera.get_rgba()[:, :, :3])
    plt.draw()
    plt.savefig(f"camera.frame{i:03d}.png")
```

매 100 물리 스텝마다(≈5초 간격, 20Hz 카메라 기준) PNG 파일로 저장합니다. `matplotlib`을 사용하는 간단한 방식이지만, 대규모 데이터 수집에서는 `cv2.imwrite()`나 Isaac Sim의 Replicator를 사용하는 것이 효율적입니다.

## 실행 방법

```bash
cd ~/workspace/IsaacSim
python standalone_examples/api/isaacsim.sensors.camera/camera.py

# debug 출력 비활성화
python standalone_examples/api/isaacsim.sensors.camera/camera.py --disable_output

# 테스트 모드 (600프레임 후 자동 종료)
python standalone_examples/api/isaacsim.sensors.camera/camera.py --test
```

실행 후 현재 디렉토리에 `camera.frame099.png`, `camera.frame199.png` 등의 파일이 생성됩니다.

## RL 환경에서의 활용

Isaac Lab에서 vision-based RL 환경을 구성할 때:

```python
# TiltedCamera — 기울어진 카메라 관측
tiled_camera = TiledCameraCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base_link/camera",
    offset=TiledCameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.5), rot=(1.0, 0.0, 0.0, 0.0)),
    data_types=["rgb", "depth"],
    spawn=PinholeCameraCfg(
        focal_length=1.93, horizontal_aperture=2.56,
        clipping_range=(0.01, 10.0),
    ),
    width=80, height=80,
)
```

`TiledCamera`는 4096+ 환경에서 동시에 카메라 렌더링을 수행하는 GPU 가속 버전입니다. 각 환경의 카메라 이미지를 single batch로 처리하여 대규모 병렬 vision-based RL을 가능하게 합니다.

## Further Reading

- **Camera 소스**: `~/workspace/IsaacSim/standalone_examples/api/isaacsim.sensors.camera/camera.py`
- **Camera 클래스**: `~/workspace/IsaacSim/exts/isaacsim.sensors.camera/`
- **Replicator (대규모 데이터 생성)**: Isaac Sim Replicator는 Camera 데이터를 대량으로 수집하고 다양한 augmentation을 적용하는 전용 파이프라인입니다.
