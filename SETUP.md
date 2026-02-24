# 환경 구축 가이드

Isaac Sim과 Isaac Lab 데모를 실행하기 위한 환경 설정 및 검증 가이드.

## 사전 요구사항

- **OS**: Ubuntu 20.04 / 22.04
- **GPU**: NVIDIA RTX 30xx 이상 (VRAM 8GB+, 권장 16GB+)
- **Driver**: NVIDIA Driver 525.60+ (Isaac Sim 호환)
- **Python**: 3.10 (Isaac Sim/Lab 호환 버전)
- **Conda**: Anaconda 또는 Miniconda

## 1. conda 환경 확인

```bash
# env_isaaclab 환경이 존재하는지 확인
conda env list | grep env_isaaclab

# 환경 활성화
conda activate env_isaaclab

# Python 버전 확인 (3.10.x 예상)
python --version
```

## 2. GPU 상태 확인

```bash
# GPU 인식 및 드라이버 확인
nvidia-smi

# CUDA 버전 확인
nvcc --version

# PyTorch GPU 사용 가능 여부
python -c "import torch; print(f'CUDA: {torch.cuda.is_available()}, Device: {torch.cuda.get_device_name(0)}')"
```

**주의**: 다른 프로세스에서 학습이 진행 중이면 `nvidia-smi`에서 GPU 메모리 사용량을 확인한다. Isaac Sim GUI는 최소 4GB VRAM을 사용하므로 여유 메모리가 부족하면 학습 프로세스와 충돌할 수 있다.

## 3. Isaac Sim 설치 확인

```bash
cd ~/workspace/IsaacSim

# Isaac Sim 실행 가능 여부 (headless 모드)
./isaac-sim.sh --headless --help

# Isaac Sim Python 환경에서 import 테스트
./python.sh -c "from isaacsim import SimulationApp; print('Isaac Sim import OK')"
```

### Isaac Sim 디렉토리 구조

```
~/workspace/IsaacSim/
├── isaac-sim.sh           # GUI 실행 스크립트
├── python.sh              # Isaac Sim Python 인터프리터
├── exts/                  # Extension 패키지
│   ├── isaacsim.examples.interactive/  # Interactive 데모
│   ├── isaacsim.robot.policy.examples/ # ONNX 정책 배포 예제
│   └── isaacsim.sensors.*/             # 센서 관련 Extension
├── standalone_examples/   # GUI 없이 실행하는 스크립트
│   ├── api/               # API 활용 예제
│   ├── tutorials/         # 튜토리얼
│   └── benchmarks/        # 성능 벤치마크
└── data/                  # 에셋, 체크포인트 등
```

## 4. Isaac Lab 설치 확인

```bash
cd ~/workspace/IsaacLab

# isaaclab.sh wrapper 실행 확인
./isaaclab.sh -h

# Isaac Lab 모듈 import 테스트
./isaaclab.sh -p -c "import isaaclab; print(f'Isaac Lab {isaaclab.__version__}')"

# Isaac Lab 태스크 등록 확인
./isaaclab.sh -p -c "
import isaaclab_tasks  # 태스크 등록 트리거
from isaaclab.envs import DirectRLEnv, ManagerBasedRLEnv
print('Direct + Manager-Based env available')
"
```

### Isaac Lab 디렉토리 구조

```
~/workspace/IsaacLab/
├── isaaclab.sh                    # CLI wrapper (학습/평가 실행)
├── scripts/
│   ├── demos/                     # 빌트인 시각화 데모
│   ├── reinforcement_learning/    # RL 프레임워크별 train/play 스크립트
│   │   ├── rsl_rl/
│   │   ├── skrl/
│   │   ├── rl_games/
│   │   ├── sb3/
│   │   └── ray_rllib/
│   └── tutorials/                 # 튜토리얼 스크립트
├── source/
│   ├── isaaclab/                  # 코어 라이브러리
│   │   ├── isaaclab/envs/         # DirectRLEnv, ManagerBasedRLEnv 베이스 클래스
│   │   ├── isaaclab/managers/     # ObservationManager, RewardManager 등
│   │   └── isaaclab/assets/       # Articulation, RigidObject wrapper
│   └── isaaclab_tasks/            # 태스크 환경 정의
│       └── isaaclab_tasks/
│           ├── direct/            # Direct 방식 환경
│           └── manager_based/     # Manager-Based 방식 환경
└── logs/                          # 학습 로그 (TensorBoard)
```

## 5. 실행 모드별 검증

### Interactive Demo (GUI)

```bash
cd ~/workspace/IsaacSim

# Isaac Sim GUI 실행 + interactive examples 활성화
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI 로딩 후 메뉴: Isaac Examples > ... 에서 데모 선택
```

**체크포인트**: GUI가 정상 실행되고 메뉴에서 "Hello World" 데모를 로드할 수 있으면 성공.

### Standalone Script

```bash
cd ~/workspace/IsaacSim

# 가장 기본적인 standalone 실행 테스트
python standalone_examples/api/isaacsim.core/hello_world.py
```

### Isaac Lab 학습/평가

```bash
cd ~/workspace/IsaacLab

# CartPole 환경으로 빠른 테스트 (환경 수를 줄여서 빠르게 확인)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 16 \
    --max_iterations 10

# 사전학습된 체크포인트로 시각화 테스트
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Cartpole-Direct-v0 \
    --use_pretrained_checkpoint \
    --num_envs 4
```

**체크포인트**: 학습이 시작되고 iteration 로그가 출력되면 성공. 사전학습 체크포인트로 CartPole이 균형을 잡으면 전체 파이프라인 검증 완료.

## 6. 공통 문제 해결

### GPU 메모리 부족

```
RuntimeError: CUDA out of memory
```

- `--num_envs`를 줄인다 (4096 → 256 → 16)
- `nvidia-smi`로 다른 프로세스의 GPU 사용량 확인
- 다른 학습이 돌고 있으면 그것이 끝난 후 실행

### Isaac Sim GUI가 느리게 로딩됨

초기 실행 시 Nucleus에서 에셋을 다운로드하므로 첫 실행이 오래 걸릴 수 있다. 두 번째 실행부터는 캐시된 에셋을 사용하므로 빨라진다.

### Extension이 로딩되지 않음

```bash
# 특정 Extension을 명시적으로 활성화
./isaac-sim.sh --enable isaacsim.examples.interactive --enable isaacsim.robot.policy.examples
```

Extension Manager (Window > Extensions)에서 해당 Extension이 활성화되어 있는지 확인한다.

### Isaac Lab 태스크를 찾을 수 없음

```
gymnasium.error.NameNotFound: Environment Isaac-XXX-v0 doesn't exist
```

`isaaclab_tasks` 패키지가 정상 설치되었는지 확인:
```bash
./isaaclab.sh -p -c "import isaaclab_tasks; print(isaaclab_tasks.__path__)"
```

### DISPLAY 관련 오류 (Headless 서버)

```bash
# Headless 모드로 실행
./isaaclab.sh -p scripts/.../train.py --headless

# 또는 DISPLAY 환경변수 설정
export DISPLAY=:0
```

## 7. MCP 서버 설정 (선택)

이 레포지토리는 MCP(Model Context Protocol) 서버를 통해 AI 도구와 연동할 수 있다.

### Isaac Sim MCP
Isaac Sim이 실행 중일 때 `isaac.sim.mcp_extension` Extension을 활성화하면 localhost:8766에서 MCP 서버가 동작한다. Python 코드 실행, 씬 오브젝트 생성, 3D 모델 검색 등이 가능하다.

### ROS MCP
ROSBridge를 통해 ROS 토픽/서비스와 연동한다. `uvx ros-mcp --transport=stdio`로 실행.

### Notion MCP
코드 분석 결과를 Notion 페이지로 자동 발행할 때 사용. `OPENAPI_MCP_HEADERS` 환경변수에 Notion API 토큰 설정 필요.

### GitHub MCP
레포지토리 관리, 이슈 생성 등에 사용. `GITHUB_PERSONAL_ACCESS_TOKEN` 환경변수 설정 필요.

설정 파일: [`opencode.json`](./opencode.json)
