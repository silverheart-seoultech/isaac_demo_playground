# Robot Loading

## Overview

USD 로봇 에셋을 씬에 로딩하고 `Articulation` API로 관절을 제어하는 기본 패턴을 다룹니다. Franka Emika Panda(7-DOF 매니퓰레이터)와 NovaCarter(differential drive 모바일 로봇) 두 종류의 로봇을 로드하여, 관절 위치 제어와 관절 속도 제어라는 서로 다른 제어 패러다임을 비교합니다.

## Architecture

```
World
 │
 ├── add_reference_to_stage(usd_path, prim_path)  ← USD 에셋 → Stage에 참조 추가
 │
 ├── Articulation("/World/Arm")      ← Franka: 7-DOF, 위치 제어
 │   ├── set_joint_positions()
 │   └── get_joint_positions()
 │
 └── Articulation("/World/Carter")   ← NovaCarter: 2-DOF, 속도 제어
     ├── set_joint_velocities()
     └── get_joint_positions()
```

`add_reference_to_stage()`는 USD Composition의 Reference Arc를 사용하여 외부 USD 파일을 현재 Stage에 마운트합니다. 원본 USD 파일을 수정하지 않으면서 로봇을 씬에 배치할 수 있습니다.

## Source Files

| 파일 | 역할 |
|---|---|
| `standalone_examples/tutorials/getting_started_robot.py` | 로봇 로딩 튜토리얼 (83줄). Franka + NovaCarter |

## 핵심 코드 분석

### USD 로봇 에셋 로딩

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

assets_root_path = get_assets_root_path()  # Nucleus 서버 URL 반환
arm_usd = assets_root_path + "/Isaac/Robots/FrankaEmika/panda_instanceable.usd"

add_reference_to_stage(usd_path=arm_usd, prim_path="/World/Arm")
```

`get_assets_root_path()`는 NVIDIA Nucleus 서버의 에셋 루트 경로를 반환합니다. 로봇 USD는 Nucleus에서 다운로드되어 로컬 캐시에 저장됩니다. `_instanceable.usd` 접미사는 GPU instancing을 지원하는 에셋으로, 수천 개의 환경을 병렬 시뮬레이션할 때 VRAM 사용량을 크게 줄입니다.

### Articulation API

```python
from omni.isaac.core.articulations import Articulation

arm = Articulation(prim_paths_expr="/World/Arm")
car = Articulation(prim_paths_expr="/World/Carter")

world.reset()  # Articulation 초기화 필수

# 위치 제어 (Franka)
positions = arm.get_joint_positions()        # shape: (7,) — 7 관절 각도
arm.set_joint_positions(target_positions)     # 목표 관절 각도 설정

# 속도 제어 (NovaCarter)
car.set_joint_velocities(np.array([1.0, 1.0]))  # 좌/우 바퀴 속도
```

`Articulation`은 관절로 연결된 강체 체인(kinematic tree)을 추상화합니다. `world.reset()` 이후에만 `get_joint_positions()` 등의 메서드가 유효합니다 — 이는 PhysX가 Articulation 데이터 구조를 `reset()` 시점에 초기화하기 때문입니다.

### 관절 제어 모드

| 제어 모드 | API | 용도 | 예시 |
|---|---|---|---|
| Position | `set_joint_positions()` | 정밀한 자세 제어 | Franka 매니퓰레이션 |
| Velocity | `set_joint_velocities()` | 연속 운동 | 모바일 로봇 이동 |
| Effort | `set_joint_efforts()` | 토크 직접 제어 | RL 정책 출력 |

Franka는 관절 위치 제어가 기본이며, PD 제어기가 내부적으로 목표 위치까지 토크를 계산합니다. NovaCarter의 differential drive는 좌/우 바퀴 속도로 직진/회전을 결정합니다.

## 실행 방법

```bash
cd ~/workspace/IsaacSim
python standalone_examples/tutorials/getting_started_robot.py
```

## 핵심 개념

### USD Reference vs Payload

- **Reference**: 에셋을 항상 로드. 작은 로봇에 적합.
- **Payload**: 사용자가 활성화할 때만 로드. 대규모 씬에서 선택적 로딩에 유용.

`add_reference_to_stage()`는 Reference Arc를 사용합니다. 대규모 환경에서 수천 개의 로봇 인스턴스를 로드할 때는 instanceable USD와 함께 사용하여 메모리를 절약합니다.

### prim_paths_expr

`Articulation(prim_paths_expr="/World/Arm")`에서 `prim_paths_expr`은 정규표현식 패턴을 지원합니다. 예를 들어 `"/World/Env_*/Robot"`은 `Env_0/Robot`, `Env_1/Robot`, ... 모든 환경의 로봇을 한 번에 제어할 수 있습니다. Isaac Lab의 병렬 환경이 이 패턴을 활용합니다.

### 초기화 순서 주의

```
add_reference_to_stage() → Articulation() → world.reset() → get_joint_positions()
                                                   │
                                            PhysX Articulation
                                            데이터 구조 초기화
```

`world.reset()` 전에 관절 데이터를 읽으려 하면 에러가 발생합니다. 이는 PhysX의 Articulation Cache가 `reset()` 시점에 생성되기 때문입니다.

## Further Reading

- **Articulation API**: `~/workspace/IsaacSim/exts/isaacsim.core/isaacsim/core/articulations/`
- **Franka USD**: `~/workspace/IsaacSim/data/` 또는 Nucleus `Isaac/Robots/FrankaEmika/`
- **NovaCarter USD**: Nucleus `Isaac/Robots/NovaCarter/`
