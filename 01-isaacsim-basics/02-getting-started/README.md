# Getting Started
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

Isaac Sim의 씬 구성 기본 요소를 단계적으로 학습하는 데모. `World` 클래스를 통한 시뮬레이션 관리, USD Prim 타입(Visual/Dynamic/Rigid), 물리/충돌 속성의 동적 추가, 그리고 물리 스테핑 루프를 다룹니다. Interactive 버전은 UI 버튼을 통해 씬 요소를 하나씩 추가하며 각 단계의 효과를 시각적으로 확인할 수 있습니다.

## Architecture

### Standalone 실행 흐름

```
World(stage_units_in_meters=1.0)
    │
    ├── Iteration 1: GroundPlane + VisualCuboid + DynamicCuboid
    │   └── 500 steps → VisualCuboid는 정지, DynamicCuboid는 낙하
    │
    ├── Iteration 2: + RigidPrim(physics 속성 추가)
    │   └── 500 steps → 기존 Visual Cuboid가 물리 오브젝트로 변환
    │
    └── Iteration 3: + GeometryPrim.apply_collision_apis()
        └── 500 steps → 충돌 메시가 추가되어 물리 상호작용 가능
```

3단계 반복을 통해 USD Prim에 물리 속성을 점진적으로 추가하는 과정을 보여줍니다. 이는 USD의 composition 아키텍처 — Prim 자체와 물리/충돌 API가 분리된 구조 — 를 직관적으로 이해하게 합니다.

### Interactive UI 구조

```
GettingStartedUI(BaseSampleUITemplate)
    │
    ├── _add_ground_plane()          → GroundPlane 추가
    ├── _add_light_source()          → DistantLight 추가
    ├── _add_visual_cube()           → VisualCuboid (렌더링만)
    ├── _add_physics_cube()          → DynamicCuboid (물리+렌더링)
    ├── _add_physics_properties()    → 기존 Prim에 RigidBody API
    └── _add_collision_properties()  → 기존 Prim에 CollisionAPI
```

## Source Files

| 파일 | 역할 |
|---|---|
| `standalone_examples/tutorials/getting_started.py` | Standalone 튜토리얼 (85줄). 3단계 물리 속성 추가 시연 |
| `exts/.../getting_started/getting_started.py` | Interactive 예제 (48줄). BaseSample, 카메라 설정 |
| `exts/.../getting_started/getting_started_extension.py` | Extension UI (208줄). 6개 버튼으로 씬 구성 단계 시연 |
| `exts/.../getting_started/start_with_robot.py` | 로봇 버전 (69줄). 물리 콜백으로 로봇 상태 출력 |

## 핵심 코드 분석

### World 클래스와 물리 스테핑

```python
from omni.isaac.core import World

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# 씬 오브젝트 추가
world.scene.add(VisualCuboid(prim_path="/World/visual", ...))
world.scene.add(DynamicCuboid(prim_path="/World/dynamic", ...))

world.reset()  # 물리 초기화 + 모든 오브젝트 기본 상태로

for i in range(500):
    world.step(render=True)  # 물리 1스텝 + 렌더링
```

`World.step(render=True)`는 한 번의 호출로 물리 시뮬레이션과 렌더링을 모두 수행합니다. `render=False`로 설정하면 렌더링을 건너뛰어 headless 학습에서 성능을 최적화할 수 있습니다.

### USD Prim 타입 계층

```
                  Prim (USD 기본 단위)
                   │
        ┌──────────┼──────────┐
        │          │          │
   VisualCuboid  DynamicCuboid  RigidPrim
   (렌더링만)    (물리+렌더링)   (기존 Prim에
                              물리 속성 추가)
```

- **VisualCuboid**: 렌더링 메시만 존재. 물리 시뮬레이션에 참여하지 않아 중력에도 움직이지 않음.
- **DynamicCuboid**: `RigidBodyAPI` + `CollisionAPI`가 적용된 상태로 생성. 즉시 물리 시뮬레이션에 참여.
- **RigidPrim**: 이미 존재하는 USD Prim에 `RigidBodyAPI`를 동적으로 추가. Visual → Dynamic 변환에 사용.

### 물리/충돌 속성 분리

```python
# 1) VisualCuboid 생성 (물리 없음)
cube = VisualCuboid(prim_path="/World/cube", size=0.1, color=np.array([1, 0, 0]))

# 2) RigidBody 속성 추가 → 중력의 영향을 받음
rigid = RigidPrim(prim_path="/World/cube")

# 3) Collision 속성 추가 → 다른 오브젝트와 충돌
geom = GeometryPrim(prim_path="/World/cube")
geom.apply_collision_apis()
```

USD의 Composition Arc 구조에서는 Prim의 형상(Mesh)과 물리적 속성(RigidBody, Collision)이 별도의 API 스키마로 관리됩니다. 이 분리 덕분에 동일한 Mesh에 물리 속성을 사후에 추가/제거할 수 있습니다.

## 실행 방법

```bash
# Standalone
cd ~/workspace/IsaacSim
python standalone_examples/tutorials/getting_started.py

# Interactive (GUI)
./isaac-sim.sh --enable isaacsim.examples.interactive
# 메뉴: Isaac Examples > Getting Started
# UI 버튼을 순서대로 클릭하여 씬 구성 단계 확인
```

## 핵심 개념

### World vs SimulationContext

`World`는 `SimulationContext`의 상위 래퍼로, `scene` 속성을 통한 오브젝트 관리, 물리 콜백 등록, 리셋 시 오브젝트 기본 상태 복원 기능을 추가로 제공합니다. 대부분의 경우 `World`를 사용하되, 물리 엔진을 직접 제어해야 할 때만 `SimulationContext`를 사용합니다.

### GroundPlane의 역할

`add_default_ground_plane()`은 무한 평면 충돌체를 추가합니다. 이 평면은 `PhysicsGroundPlane`으로 구현되며, 렌더링되지 않는 충돌 전용 오브젝트다. 시각적 바닥은 별도의 Mesh로 추가해야 합니다.

### 물리 스텝과 렌더링 분리

`physics_dt`와 `rendering_dt`가 다를 수 있습니다. 예를 들어 `physics_dt=1/120`, `rendering_dt=1/60`이면 렌더링 1프레임당 물리 2스텝이 실행됩니다. `world.step()`은 이 비율에 따라 물리 서브스텝을 자동으로 처리합니다.

## Further Reading

- **World API**: `~/workspace/IsaacSim/exts/isaacsim.core/` 내부 구현
- **USD Prim 구조**: [OpenUSD Primer](https://openusd.org/release/intro.html)
- **PhysX API Schema**: Isaac Sim 내 Physics 관련 API 문서
