# Isaac Sim & Isaac Lab 소개 및 환경 설정

## 1. Isaac 생태계 개요

NVIDIA Isaac은 로봇 시뮬레이션과 AI 학습을 위한 통합 플랫폼입니다. 세 계층으로 구성되며, 각 계층이 하위 계층의 기능을 확장합니다.

```
┌──────────────────────────────────────────────────────────────┐
│                         Isaac Lab                            │
│  강화학습 프레임워크                                            │
│  Direct / Manager-Based 환경, RSL-RL / SKRL / SB3 통합        │
│  수천 개 환경 GPU 병렬 실행, 자동 Domain Randomization           │
├──────────────────────────────────────────────────────────────┤
│                         Isaac Sim                            │
│  물리 시뮬레이션 엔진                                           │
│  PhysX 5 GPU 가속, USD 씬 포맷, Articulation / Sensor API     │
│  Extension 시스템, Interactive 데모, Standalone 스크립트          │
├──────────────────────────────────────────────────────────────┤
│                       Omniverse Kit                          │
│  기반 플랫폼                                                   │
│  Hydra Renderer, USD Composer, Nucleus Asset Server           │
│  Extension Framework, OmniGraph                               │
└──────────────────────────────────────────────────────────────┘
```

### Isaac Sim

Isaac Sim은 NVIDIA Omniverse 위에 구축된 로봇 시뮬레이션 엔진입니다.

- **PhysX 5**: GPU 가속 rigid body, articulation, soft body 물리 연산을 제공합니다. 단일 GPU에서 수천 개의 rigid body를 실시간으로 시뮬레이션할 수 있습니다.
- **USD (Universal Scene Description)**: Pixar가 개발한 3D 씬 포맷으로, 계층적 prim 구조와 attribute 시스템을 통해 복잡한 로봇 씬을 관리합니다. Isaac Sim의 모든 에셋과 씬은 USD로 표현됩니다.
- **Extension 시스템**: 기능을 모듈 단위로 패키징합니다. 각 데모, 센서, 로봇 정책은 독립적인 Extension으로 구현되어 있어 필요한 기능만 선택적으로 로드할 수 있습니다.
- **Sensor API**: Contact sensor, IMU, LiDAR(PhysX/RTX), Camera 등 다양한 로봇 센서를 시뮬레이션합니다.

Isaac Sim은 두 가지 실행 모드를 지원합니다:
- **Interactive (GUI)**: GUI 앱에서 Extension 메뉴를 통해 데모를 실행합니다. `BaseSample`을 상속하는 형태입니다.
- **Standalone**: `SimulationApp`을 생성하고 Python 스크립트로 직접 시뮬레이션을 제어합니다. Headless 실행이 가능합니다.

### Isaac Lab

Isaac Lab은 Isaac Sim 위에 구축된 강화학습(RL) 프레임워크입니다. 이전의 IsaacGymEnvs, OmniIsaacGymEnvs, Orbit 프레임워크를 통합/대체합니다.

- **두 가지 환경 패러다임**:
  - **Direct**: 단일 클래스에서 observation, reward, reset을 직접 구현합니다. 자유도가 높고 `@torch.jit.script`로 JIT 컴파일이 가능합니다.
  - **Manager-Based**: 선언적 Config로 MDP 요소(observation, action, reward, termination, curriculum, event)를 각각의 Manager에 위임합니다. 코드 재사용성이 높습니다.

- **GPU 병렬 환경**: 단일 PhysX scene에서 수천 개의 환경을 동시 실행합니다. 모든 데이터는 GPU tensor로 관리되어 CPU-GPU 전송 병목이 없습니다.

- **RL 프레임워크 통합**: RSL-RL, SKRL, Stable Baselines3, RL Games, Ray RLlib을 지원합니다.

### Isaac Gym과의 관계

Isaac Gym은 GPU 가속 물리 시뮬레이션의 초기 구현체였으며, 현재는 deprecated 상태입니다. Isaac Gym의 핵심 기능(GPU 병렬 환경, 텐서 기반 API)은 Isaac Sim에 통합되었고, Isaac Lab이 이를 기반으로 학습 프레임워크를 제공합니다. 기존 Isaac Gym 코드를 Isaac Lab으로 마이그레이션할 것을 권장합니다.

## 2. 시스템 요구사항

### 하드웨어

| 항목 | 최소 사양 | 권장 사양 |
|---|---|---|
| GPU | NVIDIA RTX 2070 (8GB VRAM) | RTX 3090 / RTX 4090 (24GB VRAM) |
| CPU | Intel i7 / AMD Ryzen 7 | Intel i9 / AMD Ryzen 9 |
| RAM | 32 GB | 64 GB |
| 저장공간 | 50 GB SSD | 100 GB+ NVMe SSD |

VRAM은 병렬 환경 수(`num_envs`)에 직접 영향을 미칩니다. 8GB VRAM에서는 CartPole 4096개, ANYmal locomotion 2048개 정도가 한계이며, 24GB에서는 대부분의 환경을 8192개 이상 실행할 수 있습니다.

### 소프트웨어

| 항목 | 요구사항 |
|---|---|
| OS | Ubuntu 22.04 LTS (권장), Ubuntu 20.04 LTS |
| NVIDIA Driver | 580.65.06 이상 (Isaac Sim 5.x 기준) |
| CUDA | 12.x (Isaac Sim에 번들 포함) |
| Python | 3.11 (Isaac Sim 5.x), 3.10 (Isaac Sim 4.x) |
| Conda | Miniconda 또는 Anaconda |

> **참고**: Isaac Sim은 자체 Python 인터프리터와 CUDA 런타임을 번들로 포함합니다. 시스템에 별도의 CUDA toolkit을 설치할 필요는 없지만, NVIDIA 드라이버는 반드시 호환 버전이어야 합니다.

### 버전 호환성 매트릭스

Isaac Lab과 Isaac Sim의 버전은 반드시 호환되는 조합을 사용해야 합니다. 잘못된 조합은 import 오류나 런타임 크래시를 유발합니다.

| Isaac Lab | Isaac Sim | Python | PyTorch | NVIDIA Driver (Linux) |
|---|---|---|---|---|
| 2.3.x | 5.1.0 | 3.11 | 2.7.0 (CUDA 12.8) | 580.65.06+ |
| 2.1.x ~ 2.2.x | 4.5.0 | 3.10 | 2.4.x (CUDA 12.1) | 535.x+ |
| 1.x (Orbit) | 4.2.0 | 3.10 | 2.2.x (CUDA 12.1) | 535.x+ |

> **주의**: Isaac Lab의 `main` 브랜치는 최신 Isaac Sim을 타겟으로 합니다. 특정 Isaac Sim 버전에 맞는 브랜치(예: `v2.3.0`)를 사용하는 것이 안전합니다.

### 서버/클라우드 GPU 참고사항

A100, H100 등 데이터센터 GPU는 RT Core가 탑재되어 있지 않아 RTX 렌더링 기반 GUI가 동작하지 않습니다. 이러한 GPU에서는 반드시 headless 모드(`--headless`)로 실행해야 합니다. RTX LiDAR 등 RTX 기반 센서도 사용할 수 없으므로, PhysX LiDAR로 대체해야 합니다.

## 3. Isaac Sim 설치

Isaac Sim은 두 가지 방법으로 설치할 수 있습니다: Pre-built Binary와 pip 패키지. 연구용으로는 **Pre-built Binary** 방식을 권장합니다. Extension 개발, VS Code 연동, GUI 디버깅이 더 편리합니다.

### 방법 1: Pre-built Binary (권장)

**Step 1**: [Isaac Sim 다운로드 페이지](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html)에서 Linux용 zip 파일을 다운로드합니다.

**Step 2**: 원하는 경로에 압축을 풀어줍니다.

```bash
# 예시: 홈 디렉토리에 설치
cd ~
unzip isaac-sim-standalone-*.zip -d isaacsim

# 또는 workspace에 설치
mkdir -p ~/workspace/IsaacSim
unzip isaac-sim-standalone-*.zip -d ~/workspace/IsaacSim
```

**Step 3**: 환경변수를 설정합니다.

```bash
# ~/.bashrc 또는 ~/.zshrc에 추가
export ISAACSIM_PATH="${HOME}/workspace/IsaacSim"
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
```

**Step 4**: 설치를 검증합니다.

```bash
cd ${ISAACSIM_PATH}

# Isaac Sim Python 환경 테스트
./python.sh -c "from isaacsim import SimulationApp; print('Isaac Sim OK')"

# GUI 실행 테스트 (첫 실행 시 셰이더 컴파일로 20~60분 소요 가능)
./isaac-sim.sh
```

> **참고**: 첫 실행 시 Isaac Sim은 Extension 에셋과 셰이더를 다운로드/컴파일합니다. 네트워크 환경에 따라 10~30분 소요될 수 있으며, 이후 실행은 빠르게 로딩됩니다.

> **참고**: 첫 실행 시 EULA(최종 사용자 라이선스 동의) 팝업이 나타날 수 있습니다. CLI 환경에서는 `--accept-eula` 플래그를 추가하거나, `~/.nvidia-omniverse/config/privacy.toml` 파일에서 사전 동의를 설정할 수 있습니다.

### 방법 2: pip 패키지

GLIBC 2.35 이상이 필요합니다 (Ubuntu 22.04+). Ubuntu 20.04에서는 Binary 방식을 사용해야 합니다.

```bash
# Python 가상환경 생성 (Python 3.11 필요, Isaac Sim 5.x 기준)
conda create -n isaac python=3.11 -y
conda activate isaac

# Isaac Sim pip 패키지 설치
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com

# CUDA 호환 PyTorch 설치
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
```

> **주의**: pip 설치 방식은 편리하지만, VS Code 연동이나 Extension 소스 탐색에 제약이 있을 수 있습니다. 연구 목적으로 Isaac Sim 내부 코드를 자주 참조한다면 Binary 방식을 권장합니다.

### 방법 3: Docker (선택)

Docker를 사용하면 호스트 시스템의 드라이버 외에 별도 설정 없이 Isaac Sim을 실행할 수 있습니다. CI/CD 파이프라인이나 클라우드 환경에서 유용합니다.

```bash
# NVIDIA Container Toolkit 설치 (필수)
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

# Isaac Sim Docker 이미지 실행
docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all \
    -e "ACCEPT_EULA=Y" \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    nvcr.io/nvidia/isaac-sim:5.1.0
```

> Docker 환경에서는 GUI를 사용할 수 없으며, headless 모드 또는 WebRTC/Livestream을 통해 시각화해야 합니다. 자세한 내용은 [공식 Docker 설치 가이드](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install-container.html)를 참조합니다.

## 4. Isaac Lab 설치

### Step 1: 소스 클론

```bash
cd ~/workspace
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
```

### Step 2: conda 환경 생성

`isaaclab.sh`가 제공하는 자동 환경 생성 기능을 사용합니다:

```bash
# 기본 이름(env_isaaclab)으로 conda 환경 생성
./isaaclab.sh --conda

# 또는 커스텀 이름
./isaaclab.sh --conda my_env

# 환경 활성화
conda activate env_isaaclab
```

Isaac Sim 5.x를 사용하는 경우 Python 3.11, Isaac Sim 4.x를 사용하는 경우 Python 3.10 환경이 생성됩니다. `isaaclab.sh`가 Isaac Sim 경로를 자동 감지하여 symlink를 설정합니다.

### Step 3: Isaac Lab 빌드

```bash
# 시스템 의존성 설치 (Ubuntu)
sudo apt install cmake build-essential

# Isaac Lab 및 RL 프레임워크 설치 (editable 모드)
./isaaclab.sh --install
# 또는 특정 RL 프레임워크만 설치:
# ./isaaclab.sh --install rsl_rl
# ./isaaclab.sh --install skrl
# ./isaaclab.sh --install sb3
```

`--install` 명령은 `source/` 디렉토리의 모든 Isaac Lab extension을 `pip install -e`(editable) 모드로 설치합니다. 기본적으로 rl_games, rsl_rl, sb3, skrl, robomimic 프레임워크를 모두 설치합니다.

### Step 4: 설치 검증

```bash
# 빈 시뮬레이터 창 실행 테스트
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py

# Isaac Lab 모듈 import 테스트
./isaaclab.sh -p -c "import isaaclab; print(f'Isaac Lab {isaaclab.__version__}')"

# 등록된 RL 환경 목록 확인
./isaaclab.sh -p -c "
import isaaclab_tasks
import gymnasium as gym
tasks = [k for k in sorted(gym.registry.keys()) if 'Isaac' in k]
print(f'Registered tasks: {len(tasks)}')
for t in tasks[:10]:
    print(f'  {t}')
print('  ...')
"
```

빈 시뮬레이터 창이 열리고, Isaac Lab 버전과 등록된 태스크 목록이 출력되면 설치가 완료된 것입니다.

## 5. 첫 실행 테스트

설치 완료 후, 다음 단계로 전체 파이프라인이 정상 동작하는지 검증합니다.

### Isaac Sim GUI 데모

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI 로딩 후: Isaac Examples > Hello World 데모 선택
```

### Isaac Lab RL 학습 (빠른 테스트)

```bash
cd ~/workspace/IsaacLab

# CartPole 환경으로 10 iteration만 실행하여 파이프라인 검증
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 16 \
    --max_iterations 10

# 학습 iteration 로그가 출력되면 성공
```

### 사전학습 체크포인트 시각화

```bash
# 사전학습된 CartPole 정책을 GUI로 확인
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Cartpole-Direct-v0 \
    --use_pretrained_checkpoint \
    --num_envs 4
```

CartPole이 균형을 잡는 모습이 보이면 전체 환경이 정상입니다.

## 6. 디렉토리 구조 이해

### Isaac Sim

```
~/workspace/IsaacSim/
├── isaac-sim.sh                  # GUI 실행 래퍼
├── python.sh                     # Isaac Sim Python 인터프리터
├── exts/                         # Extension 패키지 (데모, 센서, 정책 등)
│   ├── isaacsim.examples.interactive/   # GUI 데모
│   ├── isaacsim.robot.policy.examples/  # ONNX 정책 배포
│   └── isaacsim.sensors.*/              # 센서 Extension
├── standalone_examples/          # Standalone 스크립트
│   ├── api/                      # API 예제
│   └── tutorials/                # 튜토리얼
└── data/                         # 에셋, 체크포인트
```

### Isaac Lab

```
~/workspace/IsaacLab/
├── isaaclab.sh                   # CLI 래퍼 (학습, 설치, 환경 관리)
├── scripts/
│   ├── demos/                    # 시각화 데모 (quadrupeds, arms 등)
│   ├── reinforcement_learning/   # RL 프레임워크별 train/play 스크립트
│   │   ├── rsl_rl/              # RSL-RL (locomotion에 주로 사용)
│   │   ├── skrl/                # SKRL (범용)
│   │   ├── sb3/                 # Stable Baselines3
│   │   └── rl_games/            # RL Games
│   └── tutorials/               # 단계별 학습 튜토리얼
├── source/
│   ├── isaaclab/                 # 코어 라이브러리
│   │   └── isaaclab/
│   │       ├── app/             # AppLauncher (SimulationApp 래퍼)
│   │       ├── envs/            # DirectRLEnv, ManagerBasedRLEnv 베이스
│   │       ├── managers/        # Observation, Reward, Action Manager
│   │       ├── sim/             # SimulationCfg, PhysxCfg
│   │       └── utils/           # 수학 유틸리티, 쿼터니언 변환
│   └── isaaclab_tasks/           # 환경 정의 (태스크별 Config)
│       └── isaaclab_tasks/
│           ├── direct/          # Direct 환경 (CartPole, Ant, Shadow Hand)
│           └── manager_based/   # Manager-Based 환경 (locomotion, manipulation)
└── logs/                         # 학습 로그 (TensorBoard 이벤트, 체크포인트)
```

## 7. 핵심 개념 미리보기

이 레포지토리의 데모를 학습하기 전에 알아두면 좋은 핵심 개념들입니다.

### 실행 방식 3가지

| 방식 | 사용 섹션 | 특징 |
|---|---|---|
| Isaac Sim GUI | 01~05 | `isaac-sim.sh`로 GUI 실행 후 메뉴에서 데모 선택 |
| Isaac Sim Standalone | 일부 03, 05 | Python 스크립트로 headless/GUI 실행 |
| Isaac Lab CLI | 06~08 | `isaaclab.sh -p train.py --task ...`로 학습/평가 |

### Direct vs Manager-Based

| 항목 | Direct | Manager-Based |
|---|---|---|
| 구현 방식 | 단일 클래스에서 모든 로직 직접 구현 | Config로 선언, Manager가 실행 |
| 적합한 경우 | 비표준 로직, 최대 성능, 빠른 프로토타이핑 | 컴포넌트 재사용, 실험 관리, 팀 협업 |
| 예시 | CartPole Direct, Shadow Hand | ANYmal-C Velocity, Franka Reach |

자세한 비교는 [09-reference/direct-vs-manager.md](../09-reference/direct-vs-manager.md)를 참조합니다.

### 물리 타이밍

Isaac Lab의 시뮬레이션 루프는 세 가지 시간 스케일로 구성됩니다:

- **physics_dt**: PhysX 물리 연산 주기 (기본 1/60초)
- **decimation**: policy step당 physics step 반복 횟수
- **control_dt**: RL policy의 action 출력 주기 = physics_dt × decimation

자세한 설명은 [09-reference/physics-timing.md](../09-reference/physics-timing.md)를 참조합니다.

## 8. 학습 로드맵

이 레포지토리는 다음 순서로 학습하는 것을 권장합니다:

```
00-introduction (현재 문서)
    ↓
01-isaacsim-basics          # USD 씬, PhysX, Articulation API, OmniGraph
    ↓
02-isaacsim-manipulation    # Franka IK, Cortex behavior tree, 경로 계획
03-isaacsim-locomotion      # Spot, H1, ANYmal 정책 배포
    ↓
04-isaacsim-multirobot      # 다중 로봇 협업
05-isaacsim-sensors         # Contact, IMU, LiDAR, Camera
    ↓
06-isaaclab-classic         # CartPole로 Direct vs Manager 비교, Ant, Humanoid
    ↓
07-isaaclab-locomotion      # ANYmal-C, H1, Go2 보행 학습
08-isaaclab-manipulation    # Franka Reach/Lift, Shadow Hand
    ↓
09-reference                # 아키텍처, CLI, 타이밍, 디버깅 (수시 참조)
```

01~05 섹션은 Isaac Sim의 물리 시뮬레이션과 로봇 제어 기초를 다루고, 06~08 섹션은 Isaac Lab의 RL 학습 파이프라인을 다룹니다. 09-reference는 특정 주제에 대한 깊은 참조 자료로, 필요할 때 수시로 참조합니다.

## 9. 유용한 링크

| 자료 | URL |
|---|---|
| Isaac Sim 공식 문서 | https://docs.isaacsim.omniverse.nvidia.com/ |
| Isaac Lab 공식 문서 | https://isaac-sim.github.io/IsaacLab/ |
| Isaac Lab GitHub | https://github.com/isaac-sim/IsaacLab |
| Isaac Sim 다운로드 | https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html |
| PhysX 5 문서 | https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/ |
| USD 공식 문서 | https://openusd.org/release/index.html |
| NVIDIA Developer Forums | https://forums.developer.nvidia.com/c/isaac-sim/ |
