# 자주 빠지는 함정 (Common Gotchas)

## 1. 모듈 네이밍 변경 (omni.isaac.* → isaacsim.*)

Isaac Sim 4.5부터 핵심 모듈의 네임스페이스가 `omni.isaac.*`에서 `isaacsim.*`으로 변경되었습니다. 공식 문서나 오래된 튜토리얼의 import 경로가 더 이상 작동하지 않을 수 있습니다.

### 주요 변경 사항

| 이전 (Isaac Sim < 4.5) | 현재 (Isaac Sim 4.5+) |
|---|---|
| `omni.isaac.core` | `isaacsim.core.api` |
| `omni.isaac.core.utils.prims` | `isaaclab.sim.utils.prims` (Isaac Lab 경유) |
| `omni.isaac.core.utils.stage` | `isaaclab.sim.utils.stage` (Isaac Lab 경유) |
| `omni.isaac.core.simulation_context` | `isaacsim.core.api.simulation_context` |
| `omni.isaac.core.World` | `isaacsim.core.api.World` |

이전 이름의 extension은 `omni.isaac.core_archive` 등의 이름으로 하위 호환용으로 남아 있지만, 새 코드에서는 사용하지 않는 것을 권장합니다.

### Isaac Lab 내부 변경 (v0.48~v0.51)

Isaac Lab 자체도 `isaacsim.*` 직접 import를 줄이고 자체 유틸리티로 대체하는 마이그레이션을 진행했습니다:

```python
# 이전
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.version import get_version

# 현재
from isaaclab.sim.utils import get_prim_at_path   # isaaclab 경유
from isaaclab.utils.version import get_isaac_sim_version
```

CHANGELOG.rst의 v0.49.1, v0.48.5, v0.51.1 항목에서 구체적인 변경 내역을 확인할 수 있습니다.

### 확인 방법

코드에서 `omni.isaac.*` import가 실패하면, 먼저 Isaac Sim의 extension 목록에서 대응하는 `isaacsim.*` 모듈을 찾습니다:

```bash
# Isaac Sim의 extension 디렉토리 확인
ls ~/workspace/IsaacSim/exts/ | grep isaacsim
```

## 2. SimulationApp 임포트 순서

Isaac Sim의 Python 스크립트에서 **가장 먼저** `SimulationApp`을 생성해야 합니다. 이 순서를 지키지 않으면 segmentation fault가 발생합니다.

```python
# 올바른 순서
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

# SimulationApp 생성 후에 다른 Isaac 모듈 import 가능
from isaacsim.core.api import World
import torch
```

```python
# 잘못된 순서 — segfault 또는 import 에러 발생
import torch
from isaacsim.core.api import World   # SimulationApp이 아직 없으므로 실패
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
```

Isaac Lab을 사용할 때는 `AppLauncher`가 이 순서를 자동으로 처리합니다:

```python
from isaaclab.app import AppLauncher

# AppLauncher가 내부에서 SimulationApp을 먼저 생성
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

# 이후 Isaac Lab 모듈 import
from isaaclab.envs import DirectRLEnv
```

`AppLauncher`의 소스 코드를 보면, 내부적으로 extension 로딩 순서를 보장하는 로직이 있습니다. 이 순서가 깨지면 일부 extension이 초기화되지 않아 예기치 않은 crash가 발생합니다.

## 3. USD 관련 주의사항

### 단위 시스템

USD는 기본적으로 **미터(m)** 단위를 사용합니다. cm나 mm 단위의 에셋을 가져올 때 스케일을 변환해야 합니다.

```python
# USD 스테이지의 단위 확인
from pxr import UsdGeom
stage = omni.usd.get_context().get_stage()
meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
print(f"1 stage unit = {meters_per_unit} meters")
```

### Prim 경로 규칙

USD prim 경로는 `/`로 시작하는 절대 경로를 사용합니다:

```python
# 올바른 prim 경로
robot_path = "/World/Robot"
env_path = "/World/envs/env_0/Robot"

# 잘못된 prim 경로 (슬래시 누락)
robot_path = "World/Robot"  # 오류
```

Isaac Lab의 병렬 환경에서는 `{ENV_REGEX_NS}` 매크로를 사용하여 환경별 경로를 자동 생성합니다:

```python
# config에서 환경 regex 사용
prim_path="{ENV_REGEX_NS}/Robot"
# 실제로는 /World/envs/env_0/Robot, /World/envs/env_1/Robot, ... 으로 확장
```

### 각도 단위의 이중성

USD attribute에서 회전은 **도(degree)** 단위이지만, Python API의 쿼터니언 및 오일러 함수는 **라디안(radian)**을 사용합니다. 이 혼동은 매우 빈번하게 발생합니다. 자세한 내용은 [coordinate-systems.md](./coordinate-systems.md)를 참조합니다.

## 4. GPU 버퍼 오버플로우

Isaac Lab에서 대규모 병렬 환경(수천 개)을 실행할 때, PhysX의 GPU 버퍼가 부족하면 시뮬레이션이 crash하거나 예기치 않은 동작이 발생합니다.

### 증상

- "PhysX error: PxgContactManager: found/lost pair buffer overflow" 류의 경고
- 물체가 갑자기 사라지거나 관통
- 에이전트가 학습 초기에 이상하게 행동하다가 crash

### 해결 방법

`PhysxCfg`에서 GPU 버퍼 크기를 늘립니다:

```python
from isaaclab.sim import SimulationCfg, PhysxCfg

sim_cfg = SimulationCfg(
    physx=PhysxCfg(
        # 기본값 대비 2-4배 증가
        gpu_found_lost_pairs_capacity=2**22,           # 기본값: 2**21
        gpu_found_lost_aggregate_pairs_capacity=2**26, # 기본값: 2**25
        gpu_max_rigid_contact_count=2**24,             # 기본값: 2**23
        gpu_collision_stack_size=2**27,                # 기본값: 2**26
        gpu_heap_capacity=2**27,                       # 기본값: 2**26
        gpu_temp_buffer_capacity=2**25,                # 기본값: 2**24
    ),
)
```

주의: GPU 버퍼는 VRAM을 소비합니다. 불필요하게 크게 설정하면 `num_envs`를 늘릴 수 있는 여유 VRAM이 줄어듭니다. 에러 메시지가 나타날 때만 해당 버퍼를 증가시키는 것을 권장합니다.

## 5. Fabric 모드와 USD 동기화

Isaac Lab은 기본적으로 `use_fabric=True`입니다. Fabric 모드에서는 물리 데이터를 USD를 거치지 않고 GPU 버퍼에서 직접 읽어 성능이 크게 향상됩니다.

### Fabric 모드의 제약

- **GUI에서 물리 파라미터가 실시간 갱신되지 않습니다.** USD Inspector에서 joint position 등의 값이 고정되어 보일 수 있습니다.
- **USD API로 직접 prim attribute를 읽으면 stale 데이터**가 반환됩니다. 반드시 Isaac Lab의 `data` 프로퍼티를 통해 접근해야 합니다.

```python
# Fabric 모드에서 잘못된 접근
from pxr import UsdGeom
xform = UsdGeom.Xformable(prim)
pos = xform.GetLocalTransformation()  # stale! Fabric과 동기화되지 않음

# 올바른 접근
pos = env.scene["robot"].data.root_pos_w  # Fabric 버퍼에서 직접 읽음
```

디버깅 시 GUI에서 정확한 값을 확인해야 한다면 `use_fabric=False`로 전환하거나, `--disable_fabric` CLI 플래그를 사용합니다.

## 6. 첫 실행 시 셰이더 컴파일

Isaac Sim을 처음 실행하거나 GPU 드라이버를 업데이트한 후에는 셰이더 컴파일이 진행됩니다. 이 과정은 **20~60분**이 소요될 수 있으며, 이 동안 GUI가 멈춘 것처럼 보일 수 있습니다.

```bash
# 셰이더 캐시 위치
~/.cache/ov/
~/.nv/ComputeCache/

# 캐시를 지우면 재컴파일이 필요
rm -rf ~/.cache/ov  # 문제 발생 시에만 사용
```

셰이더 캐시가 정상적으로 생성되면, 이후 실행부터는 빠르게 시작됩니다. 첫 실행에서 crash가 발생한다고 오해하지 않도록 주의합니다.

## 7. Python 환경 혼동

Isaac Sim/Lab은 자체 Python 인터프리터를 사용합니다. 시스템 Python이나 다른 conda 환경의 Python으로 직접 import하면 동작하지 않습니다.

### 올바른 실행 방법

```bash
# Isaac Lab: isaaclab.sh 래퍼 사용
cd ~/workspace/IsaacLab
./isaaclab.sh -p my_script.py

# Isaac Sim standalone: IsaacSim 디렉토리의 python 사용
cd ~/workspace/IsaacSim
python standalone_examples/api/.../my_script.py
```

### 잘못된 실행 방법

```bash
# conda 환경의 python으로 직접 실행 → 모듈 import 실패
conda activate my_env
python my_isaac_lab_script.py  # ModuleNotFoundError: No module named 'isaacsim'
```

`env_isaaclab` conda 환경은 Isaac Lab 설치 시 Isaac Sim의 Python과 연동되도록 구성되어 있습니다. 이 환경을 사용할 경우 `conda activate env_isaaclab` 후 `isaaclab.sh`로 실행합니다.

## 8. registry.__del__ 경고 무시

시뮬레이션 종료 시 다음과 같은 에러가 출력될 수 있습니다:

```
Exception ignored in: <function Registry.__del__ at 0x...>
```

이 메시지는 Python 인터프리터 종료 과정에서 발생하는 정리(cleanup) 순서 문제이며, 시뮬레이션 결과에 영향을 주지 않습니다. 무시해도 됩니다.

## 9. Isaac Lab 외부 프로젝트 구조

Isaac Lab 저장소를 직접 fork하여 코드를 수정하는 것은 권장되지 않습니다. 대신 외부 프로젝트 템플릿을 사용합니다:

```bash
cd ~/workspace/IsaacLab
./isaaclab.sh --new
# 대화형으로 프로젝트 이름, 경로 등을 입력
```

이렇게 생성된 프로젝트는 Isaac Lab을 패키지로 참조하며, 독립적인 Git 저장소로 관리할 수 있습니다. 자체 task, 환경, reward 함수를 추가하려면 이 방식을 사용합니다.

## 10. num_envs 설정과 VRAM

`num_envs`가 클수록 학습 throughput이 증가하지만, VRAM 사용량도 비례하여 늘어납니다. VRAM이 부족하면 CUDA Out of Memory 에러가 발생합니다.

### VRAM별 권장 num_envs (대략적 참고)

| 태스크 | 8 GB VRAM | 12 GB VRAM | 24 GB VRAM |
|---|---|---|---|
| CartPole | 4096 | 8192 | 16384 |
| Ant | 2048 | 4096 | 8192 |
| Humanoid | 1024 | 2048 | 4096 |
| ANYmal locomotion | 2048 | 4096 | 8192 |
| Franka manipulation | 2048 | 4096 | 8192 |
| Shadow Hand | 512 | 1024 | 4096 |

실제 사용 가능한 값은 로봇의 관절 수, observation 차원, 센서 사용 여부에 따라 달라집니다. `nvidia-smi`로 VRAM 사용량을 모니터링하면서 점진적으로 늘리는 것을 권장합니다.

## 소스 코드 참조

| 파일 | 내용 |
|---|---|
| `IsaacLab/source/isaaclab/isaaclab/app/app_launcher.py` | SimulationApp 생성 순서, Extension 로딩 |
| `IsaacLab/source/isaaclab/isaaclab/sim/simulation_cfg.py` | PhysxCfg GPU 버퍼 설정 |
| `IsaacLab/source/isaaclab/docs/CHANGELOG.rst` | 모듈 리네임 마이그레이션 이력 |
| `IsaacSim/exts/isaacsim.core.api/config/extension.toml` | isaacsim.* 네임스페이스 구조 |
| `IsaacSim/exts/omni.isaac.core_archive/config/extension.toml` | 레거시 호환 extension |
