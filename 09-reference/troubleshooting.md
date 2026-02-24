# 트러블슈팅 가이드

## Isaac Sim 공통

### 1. Isaac Sim이 실행되지 않음

**증상**: `./isaac-sim.sh` 실행 후 멈추거나 crash

**원인 및 해결**:
```bash
# GPU 드라이버 확인 (최소 535.x 이상 필요)
nvidia-smi

# Vulkan 지원 확인
vulkaninfo | head -20

# 캐시 초기화 (손상된 shader 캐시)
rm -rf ~/.cache/ov
rm -rf ~/.nv/ComputeCache

# 충분한 VRAM 확인 (최소 8GB, 권장 12GB+)
nvidia-smi --query-gpu=memory.total,memory.used --format=csv
```

### 2. Extension이 로드되지 않음

**증상**: `--enable isaacsim.examples.interactive` 실행 시 메뉴에 데모가 표시되지 않음

**원인**: Extension 경로 문제 또는 의존성 누락

```bash
# Extension 경로 확인
ls ~/workspace/IsaacSim/exts/isaacsim.examples.interactive/

# 수동 Extension 활성화 (GUI)
# Window > Extensions > 검색 > isaacsim.examples.interactive > Enable
```

### 3. Nucleus Asset 로드 실패

**증상**: `Could not find Isaac Sim assets folder` 에러

**원인**: Nucleus 서버 연결 실패

```bash
# Nucleus 서버 상태 확인
# 로컬 Nucleus가 실행 중인지 확인

# 환경변수로 직접 지정
export OMNI_NUCLEUS_DEFAULT="omniverse://localhost/NVIDIA/Assets/Isaac/4.2"

# 또는 로컬 에셋 캐시 사용
# SimulationApp({"asset_path": "/path/to/local/assets"})
```

### 4. 물리 시뮬레이션이 불안정

**증상**: 로봇이 폭발하듯 날아가거나, 관통하거나, 떨림

**원인**: physics_dt가 너무 크거나, solver iteration이 부족

```python
# physics_dt 줄이기 (기본 1/60 → 1/120 이상)
sim_cfg = SimCfg(dt=1.0/120.0)

# solver iteration 늘리기
PhysxCfg(
    solver_type=1,              # TGS (0: PGS)
    min_position_iteration_count=4,
    min_velocity_iteration_count=0,
)
```

각 데모의 physics_dt 참고:
- CartPole: 1/120
- Franka manipulation: 1/400
- Spot quadruped: 1/500
- ANYmal standalone: 1/200

### 5. Standalone 스크립트에서 `No module named 'isaacsim'`

**증상**: Python 스크립트 실행 시 import 에러

**원인**: Isaac Sim의 Python 환경이 활성화되지 않음

```bash
# 올바른 실행 방법
cd ~/workspace/IsaacSim
python standalone_examples/api/.../script.py

# 또는 Isaac Lab을 통해 실행
cd ~/workspace/IsaacLab
./isaaclab.sh -p path/to/script.py
```

Isaac Sim은 자체 Python 인터프리터를 사용한다. conda 환경의 Python이 아닌, Isaac Sim 디렉토리의 `python` 또는 `isaac-sim.sh` 래퍼를 사용해야 한다.

## Isaac Lab 공통

### 6. `--task` 이름을 찾을 수 없음

**증상**: `Task 'Isaac-Xxx-v0' not found in registry`

**원인**: Task 등록 이름 오타, 또는 해당 task가 등록되지 않음

```bash
# 등록된 모든 task 목록 확인
./isaaclab.sh -p -c "
import isaaclab_tasks
from isaaclab.envs import ManagerBasedRLEnvCfg
import gymnasium as gym
for k in sorted(gym.registry.keys()):
    if 'Isaac' in k:
        print(k)
"
```

주요 task 이름 패턴:
- Direct: `Isaac-{Task}-Direct-v0` (예: `Isaac-Cartpole-Direct-v0`)
- Manager-Based: `Isaac-{Task}-v0` (예: `Isaac-Velocity-Flat-Anymal-C-v0`)

### 7. CUDA Out of Memory

**증상**: `RuntimeError: CUDA out of memory`

**해결**:
```bash
# num_envs 줄이기
./isaaclab.sh -p train.py --task Isaac-Ant-Direct-v0 --num_envs 1024

# 다른 GPU 프로세스 확인
nvidia-smi

# 환경별 권장 num_envs (8GB VRAM 기준):
# CartPole: 4096
# Ant: 2048
# Humanoid: 1024
# ANYmal locomotion: 2048
# Franka manipulation: 2048
# Shadow Hand: 512
```

### 8. 학습이 수렴하지 않음

**점검 사항**:

1. **Reward 스케일**: reward의 절대값이 너무 크거나 작으면 학습이 불안정. TensorBoard에서 reward range 확인.

2. **Observation normalization**: Isaac Lab은 기본적으로 observation normalization을 적용하지 않음. 필요 시 `ObsTerm`에 `normalize` 파라미터 추가.

3. **Action scale**: `action_scale`이 너무 크면 진동, 너무 작으면 무반응.
```python
# 각 환경의 action_scale 참고:
# CartPole: 100.0 (effort)
# Ant: 직접 확인 필요
# Locomotion velocity: 0.25~1.0 (position offset)
```

4. **Decimation**: `decimation` 값이 클수록 policy가 한 번 action을 내리면 여러 물리 스텝 동안 유지됨.

5. **학습률**: RSL-RL 기본값은 1e-3. 복잡한 환경에서는 3e-4 ~ 1e-4가 적합할 수 있음.

### 9. `isaaclab.sh` 실행 시 conda 환경 충돌

**증상**: Isaac Lab과 conda 환경의 패키지 버전 충돌

**해결**:
```bash
# conda 환경 비활성화 후 실행
conda deactivate
cd ~/workspace/IsaacLab
./isaaclab.sh -p train.py --task ...

# 또는 전용 conda 환경 사용
conda activate env_isaaclab
```

`isaaclab.sh`는 내부적으로 Isaac Sim의 Python 환경을 사용하므로, conda 환경과 충돌할 수 있다. `env_isaaclab` 환경이 이미 Isaac Lab용으로 구성되어 있다면 해당 환경을 활성화하고 사용한다.

### 10. Pretrained checkpoint 다운로드 실패

**증상**: `--use_pretrained_checkpoint` 사용 시 다운로드 에러

**해결**:
```bash
# 네트워크 연결 확인
ping omniverse-content-production.s3-us-west-2.amazonaws.com

# 수동 다운로드 후 경로 지정
./isaaclab.sh -p play.py --task Isaac-Ant-Direct-v0 \
    --checkpoint /path/to/downloaded/model.pt
```

## 센서 관련

### 11. Contact Sensor가 0만 반환

**원인**: 센서의 `radius`가 너무 작거나, body에 collision이 설정되지 않음

```python
# radius 확인 — body 크기에 비해 충분히 커야 함
ContactSensor(radius=0.1, ...)

# collision API 확인 — 접촉 대상에 UsdPhysics.CollisionAPI가 있어야 함
UsdPhysics.CollisionAPI.Apply(prim)
```

### 12. LiDAR가 데이터를 반환하지 않음

**PhysX LiDAR**: `UsdPhysics.Scene`이 정의되어 있는지 확인. PhysX LiDAR는 PhysX ray-cast를 사용하므로 physics scene이 필수.

**RTX LiDAR**: `SimulationApp({"enable_motion_bvh": True})`가 설정되어 있는지 확인. BVH 가속이 없으면 RTX ray-tracing이 동작하지 않음.

### 13. Camera 이미지가 검은색

**원인**: `camera.initialize()`가 `world.reset()` 이전에 호출됨

```python
# 올바른 순서
my_world.reset()          # 먼저 World 초기화
camera.initialize()       # 그 후 Camera 초기화
```

또는 첫 몇 프레임은 렌더링 파이프라인이 아직 준비되지 않아 검은색일 수 있다. 수 프레임 후 정상 이미지가 나오는지 확인.

## 성능 최적화

### 14. 학습이 느림

```bash
# headless 모드로 실행 (렌더링 비활성화)
./isaaclab.sh -p train.py --task ... --headless

# fabric 활성화 (USD overhead 제거)
./isaaclab.sh -p train.py --task ... --enable_cameras  # 카메라 사용 시만

# GPU profiling
nsys profile ./isaaclab.sh -p train.py --task ... --num_envs 4096
```

### 15. render=True vs render=False

```python
# Standalone에서 render 성능 차이
world.step(render=True)   # 렌더링 포함 — 느리지만 시각화 가능
world.step(render=False)  # 물리만 — 빠름, 데이터 수집에 적합
```

Camera/LiDAR 센서를 사용할 때는 `render=True`가 필요하다. 물리 데이터만 필요한 경우 `render=False`로 성능을 향상시킬 수 있다.
