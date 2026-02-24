# 좌표계와 쿼터니언 컨벤션

## Z-up 좌표계

Isaac Sim/Lab은 **Z-up** 좌표계를 사용합니다. 중력 벡터는 -Z 방향으로 설정됩니다:

```python
# SimulationCfg 기본값
gravity = (0.0, 0.0, -9.81)  # Z축이 위
```

```
        Z (up)
        │
        │
        │_______ Y
       /
      /
     X
```

이는 Omniverse/USD의 기본 좌표계입니다. ROS(X-forward, Y-left, Z-up)나 일부 게임 엔진(Y-up)과 다르므로, 외부 데이터를 가져올 때 축 변환에 주의해야 합니다.

### ROS와의 좌표계 차이

| 프레임워크 | Forward | Left | Up | 비고 |
|---|---|---|---|---|
| Isaac Sim/Lab | +X | +Y | +Z | USD/Omniverse 기본 |
| ROS (REP 103) | +X | +Y | +Z | body frame 기준 동일 |
| ROS (카메라) | +Z (optical) | -X | -Y | 카메라 광축이 Z |

Isaac Sim과 ROS는 world frame에서 동일한 Z-up 좌표계를 사용하지만, 카메라 센서의 local frame은 optical frame 컨벤션을 따르므로 변환이 필요합니다. Isaac Lab의 카메라 센서는 내부적으로 이 변환을 처리합니다.

## 쿼터니언 순서 (wxyz vs xyzw)

Isaac 생태계에서 가장 흔한 혼동 중 하나가 쿼터니언의 요소 순서입니다. **컴포넌트 간에 서로 다른 순서를 사용**하므로 반드시 변환해야 합니다.

| 컴포넌트 | 쿼터니언 순서 | 형식 |
|---|---|---|
| **Isaac Lab 내부** (torch 연산) | **(w, x, y, z)** | scalar-first |
| **PhysX** (GPU 버퍼) | **(x, y, z, w)** | scalar-last |
| **USD** (prim attribute) | **(w, x, y, z)** | scalar-first |
| **SciPy** (`scipy.spatial.transform.Rotation`) | **(x, y, z, w)** | scalar-last |
| **ROS** (`geometry_msgs/Quaternion`) | **(x, y, z, w)** | scalar-last |

### 변환 함수

Isaac Lab은 `isaaclab.utils.math` 모듈에 쿼터니언 변환 함수를 제공합니다:

```python
from isaaclab.utils.math import convert_quat

# Isaac Lab (wxyz) → PhysX/ROS (xyzw)
quat_xyzw = convert_quat(quat_wxyz, to="xyzw")

# PhysX/ROS (xyzw) → Isaac Lab (wxyz)
quat_wxyz = convert_quat(quat_xyzw, to="wxyz")
```

`convert_quat()`은 torch.Tensor와 numpy.ndarray 모두 지원합니다. 내부적으로 `torch.roll()` 또는 `np.roll()`을 사용하여 첫 번째(또는 마지막) 원소를 이동합니다.

### 실수하기 쉬운 패턴

```python
# 잘못된 예: PhysX에서 읽은 xyzw 데이터를 변환 없이 사용
root_quat = env.scene["robot"].data.root_quat_w  # 이미 (w, x, y, z) 형태

# 잘못된 예: scipy Rotation에 Isaac Lab 쿼터니언을 직접 전달
from scipy.spatial.transform import Rotation
# Rotation.from_quat()은 (x, y, z, w) 순서를 기대
r = Rotation.from_quat(root_quat.cpu().numpy())  # 오류! w가 맨 앞에 있음

# 올바른 예: 변환 후 전달
from isaaclab.utils.math import convert_quat
quat_xyzw = convert_quat(root_quat, to="xyzw")
r = Rotation.from_quat(quat_xyzw.cpu().numpy())
```

## 오일러 각도

Isaac Lab에서 오일러 각도를 쿼터니언으로 변환하는 함수입니다:

```python
import torch
from isaaclab.utils.math import quat_from_euler_xyz

roll = torch.tensor([0.0])    # X축 회전 (라디안)
pitch = torch.tensor([0.0])   # Y축 회전 (라디안)
yaw = torch.tensor([1.57])    # Z축 회전 (라디안, ~90도)

# 결과: (w, x, y, z) 형태
quat = quat_from_euler_xyz(roll, pitch, yaw)
```

### 각도 단위 주의사항

| 인터페이스 | 단위 |
|---|---|
| **USD prim attribute** | 도 (degrees) |
| **Isaac Lab Python API** | 라디안 (radians) |
| **Isaac Sim GUI** | 도 (degrees) |
| **PhysX 내부** | 라디안 (radians) |

USD 파일에서 회전값을 읽거나 GUI에서 값을 확인할 때는 도 단위이지만, Python API로 연산할 때는 라디안을 사용합니다. 이 차이를 무시하면 의도한 것보다 ~57배 큰(또는 작은) 회전이 적용됩니다.

```python
import math

# USD에서 읽은 45도 → 라디안 변환
angle_deg = 45.0
angle_rad = math.radians(angle_deg)  # 0.7854 rad
```

## 주요 수학 유틸리티

`isaaclab.utils.math` 모듈에서 자주 사용하는 함수들입니다:

| 함수 | 입력 | 출력 | 설명 |
|---|---|---|---|
| `convert_quat(q, to)` | (..., 4) | (..., 4) | wxyz ↔ xyzw 변환 |
| `quat_from_euler_xyz(r, p, y)` | 스칼라 각각 | (..., 4) wxyz | 오일러 → 쿼터니언 |
| `euler_xyz_from_quat(q)` | (..., 4) wxyz | (roll, pitch, yaw) | 쿼터니언 → 오일러 |
| `matrix_from_quat(q)` | (..., 4) wxyz | (..., 3, 3) | 쿼터니언 → 회전행렬 |
| `quat_from_matrix(m)` | (..., 3, 3) | (..., 4) wxyz | 회전행렬 → 쿼터니언 |
| `quat_mul(q1, q2)` | (..., 4) wxyz | (..., 4) wxyz | 쿼터니언 곱 |
| `quat_conjugate(q)` | (..., 4) wxyz | (..., 4) wxyz | 켤레 쿼터니언 |
| `quat_inv(q)` | (..., 4) wxyz | (..., 4) wxyz | 역 쿼터니언 |
| `quat_apply(q, v)` | q: (..., 4), v: (..., 3) | (..., 3) | 쿼터니언으로 벡터 회전 |

이 함수들은 모두 torch 텐서 배치 연산을 지원합니다. GPU에서 수천 개의 환경에 대해 동시에 연산할 수 있습니다.

## 프레임 변환

Isaac Lab에서 로봇의 상태는 일반적으로 world frame으로 표현됩니다:

```python
# 로봇 root의 world frame 위치와 회전
root_pos_w = env.scene["robot"].data.root_pos_w      # (num_envs, 3)
root_quat_w = env.scene["robot"].data.root_quat_w    # (num_envs, 4) wxyz

# 로봇 body의 world frame 위치와 회전
body_pos_w = env.scene["robot"].data.body_pos_w      # (num_envs, num_bodies, 3)
body_quat_w = env.scene["robot"].data.body_quat_w    # (num_envs, num_bodies, 4) wxyz
```

world frame에서 local frame(로봇 기준)으로 변환하려면 역 쿼터니언을 적용합니다:

```python
from isaaclab.utils.math import quat_inv, quat_apply

# target의 world 좌표를 로봇 local frame으로 변환
target_pos_w = torch.tensor([[1.0, 2.0, 0.5]])  # world frame
root_pos_w = env.scene["robot"].data.root_pos_w
root_quat_w = env.scene["robot"].data.root_quat_w

# 1) world → root 기준 상대 위치
relative_pos = target_pos_w - root_pos_w

# 2) root 쿼터니언의 역으로 회전하여 local frame 변환
target_pos_local = quat_apply(quat_inv(root_quat_w), relative_pos)
```

## 소스 코드 참조

| 파일 | 내용 |
|---|---|
| `IsaacLab/source/isaaclab/isaaclab/utils/math.py` | 쿼터니언 변환, 오일러 변환, 행렬 연산 함수 전체 |
| `IsaacLab/source/isaaclab/isaaclab/sim/simulation_cfg.py` | gravity 벡터 정의 (Z-up 좌표계) |
| `IsaacLab/source/isaaclab/isaaclab/utils/wrench_composer.py` | `convert_quat` 실사용 예시 (wxyz → xyzw 변환) |
