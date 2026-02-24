# Isaac Sim & Isaac Lab Demo Playground

NVIDIA Isaac Sim과 Isaac Lab의 주요 데모를 분석하고 정리한 레포지토리.
Isaac 생태계의 물리 시뮬레이션, 로봇 제어, 강화학습 파이프라인을 체계적으로 학습할 수 있도록 구성했습니다.

## 레포지토리 구조

```
isaac_sim_demo_playground/
├── 01-isaacsim-basics/          # Isaac Sim 기초 — 씬 구성, USD 로딩, OmniGraph
│   ├── 01-hello-world/
│   ├── 02-getting-started/
│   ├── 03-robot-loading/
│   └── 04-omnigraph-keyboard/
├── 02-isaacsim-manipulation/    # Franka, UR10 매니퓰레이션 데모
│   ├── 01-franka-follow-target/
│   ├── 02-franka-pick-place/
│   ├── 03-path-planning/
│   ├── 04-franka-cortex/
│   ├── 05-simple-stack/
│   └── 06-ur10-palletizing/
├── 03-isaacsim-locomotion/      # Spot, H1, ANYmal 보행 정책 배포
│   ├── 01-quadruped-spot/
│   ├── 02-humanoid-h1/
│   └── 03-anymal-standalone/
├── 04-isaacsim-multirobot/      # 다중 로봇 협업 시나리오
│   ├── 01-robo-factory/
│   ├── 02-robo-party/
│   └── 03-bin-filling/
├── 05-isaacsim-sensors/         # Contact, IMU, LiDAR, Camera 센서
│   ├── 01-contact-sensor/
│   ├── 02-imu-sensor/
│   ├── 03-lidar/
│   └── 04-camera/
├── 06-isaaclab-classic/         # Isaac Lab 고전 제어 환경 (CartPole, Ant, Humanoid)
│   ├── 01-cartpole-direct/
│   ├── 02-cartpole-manager/
│   ├── 03-ant/
│   └── 04-humanoid/
├── 07-isaaclab-locomotion/      # Isaac Lab 보행 학습 환경
│   ├── 01-anymal-c-velocity/
│   ├── 02-h1-locomotion/
│   └── 03-go2-velocity/
├── 08-isaaclab-manipulation/    # Isaac Lab 매니퓰레이션 학습 환경
│   ├── 01-franka-reach/
│   ├── 02-franka-lift/
│   └── 03-shadow-hand/
├── 09-reference/                # 아키텍처, 프레임워크 비교, 트러블슈팅
├── SETUP.md                     # 환경 구축 가이드
└── README.md                    # 이 파일
```

## Isaac 생태계 아키텍처

Isaac 생태계는 세 계층으로 구성됩니다:

```
┌─────────────────────────────────────────────────────┐
│                    Isaac Lab                         │
│  RL 학습 프레임워크 (Direct / Manager-Based Env)      │
│  RSL-RL, SKRL, RL Games, SB3, Ray RLlib 지원        │
├─────────────────────────────────────────────────────┤
│                   Isaac Sim                          │
│  Omniverse 기반 물리 시뮬레이션 엔진                   │
│  PhysX 5, GPU-accelerated, USD 씬 포맷               │
├─────────────────────────────────────────────────────┤
│                  Omniverse Kit                       │
│  Extension 시스템, OmniGraph, Nucleus Asset Server    │
└─────────────────────────────────────────────────────┘
```

**Isaac Sim**은 PhysX 5 기반의 GPU 가속 물리 시뮬레이션을 제공합니다. USD(Universal Scene Description) 포맷으로 씬을 관리하며, Extension 시스템으로 기능을 모듈화합니다. GUI 모드에서 인터랙티브 데모를 실행하거나, Standalone 스크립트로 headless 실행이 가능합니다.

**Isaac Lab**은 Isaac Sim 위에 구축된 강화학습 프레임워크입니다. 환경을 정의하는 두 가지 패러다임을 제공합니다:
- **Direct**: 단일 클래스에서 observation, reward, reset을 명시적으로 구현. 로직을 세밀하게 제어할 때 사용.
- **Manager-Based**: 선언적 config로 MDP 요소(observation, action, reward, termination, curriculum)를 각각의 Manager에 위임. 코드 재사용성이 높음.

## 실행 방식

이 레포지토리의 데모는 세 가지 방식으로 실행됩니다:

### 1. Isaac Sim GUI (Interactive Demo)

GUI 앱을 실행한 후 메뉴에서 데모를 선택합니다. `01~05` 섹션의 대부분이 이 방식입니다.

```bash
# Isaac Sim GUI 실행 (interactive examples 활성화)
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI 내 메뉴: Isaac Examples > [카테고리] > [데모 이름]
```

내부적으로 각 데모는 `BaseSample`을 상속하며, Extension의 `on_startup()`에서 `get_browser_instance().register_example()`로 등록됩니다. `setup_scene()` → `setup_post_load()` → `_on_physics_step()` 순서로 초기화 및 시뮬레이션이 진행됩니다.

### 2. Isaac Sim Standalone Script

GUI 없이 Python 스크립트를 직접 실행합니다. Headless 모드로 대량 데이터 생성이나 자동화에 적합합니다.

```bash
cd ~/workspace/IsaacSim
python standalone_examples/api/isaacsim.robot.manipulators.examples/franka/follow_target.py
```

### 3. Isaac Lab CLI

Isaac Lab의 `isaaclab.sh` wrapper를 통해 학습/평가를 실행합니다. `06~08` 섹션이 이 방식입니다.

```bash
cd ~/workspace/IsaacLab

# 학습 (RSL-RL 프레임워크 예시)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 4096

# 사전학습된 체크포인트로 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Cartpole-Direct-v0 \
    --use_pretrained_checkpoint \
    --num_envs 32

# Isaac Lab 빌트인 데모 (GUI)
./isaaclab.sh -p scripts/demos/quadrupeds.py
```

## 환경 확인

```bash
# conda 환경 확인
conda activate env_isaaclab
python -c "import omni.isaac.core; print('Isaac Sim OK')"

# GPU 상태
nvidia-smi

# Isaac Lab 설치 확인
cd ~/workspace/IsaacLab
./isaaclab.sh -p -c "import isaaclab; print(isaaclab.__version__)"
```

자세한 환경 구축 및 트러블슈팅은 [SETUP.md](./SETUP.md)를 참고합니다.

## 학습 로드맵

### Phase 1: Isaac Sim 기초 이해
`01-isaacsim-basics/` → USD 씬 구조, PhysX 물리 설정, Articulation API, OmniGraph를 익힙니다.

### Phase 2: 로봇 제어 패턴
`02-isaacsim-manipulation/` + `03-isaacsim-locomotion/` → Franka IK 제어, Cortex behavior tree, ONNX 정책 배포, RRT 경로 계획 등 실제 로봇 제어 패턴을 분석합니다.

### Phase 3: 시스템 통합
`04-isaacsim-multirobot/` + `05-isaacsim-sensors/` → 다중 로봇 협업, 센서 데이터 파이프라인을 학습합니다.

### Phase 4: 강화학습 환경 설계
`06-isaaclab-classic/` → CartPole로 Direct vs Manager-Based 패러다임을 비교합니다. Ant, Humanoid로 복잡한 환경 구조를 분석합니다.

### Phase 5: 실전 RL 학습
`07-isaaclab-locomotion/` + `08-isaaclab-manipulation/` → 실제 로봇(ANYmal, H1, Go2, Franka, Shadow Hand) 환경에서 학습 파이프라인을 분석합니다.

### 부록
`09-reference/` → Isaac Sim vs Lab 아키텍처, Direct vs Manager 비교, RL 프레임워크(RSL-RL/SKRL/SB3) 비교, 공통 트러블슈팅.

## 소스 코드 위치

| 카테고리 | 소스 경로 |
|---|---|
| Isaac Sim Interactive | `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/` |
| Isaac Sim Standalone | `~/workspace/IsaacSim/standalone_examples/` |
| Isaac Sim Policy Examples | `~/workspace/IsaacSim/exts/isaacsim.robot.policy.examples/` |
| Isaac Sim Sensor Examples | `~/workspace/IsaacSim/exts/isaacsim.sensors.*.examples/` |
| Isaac Lab Tasks | `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/` |
| Isaac Lab RL Scripts | `~/workspace/IsaacLab/scripts/reinforcement_learning/` |
| Isaac Lab Demos | `~/workspace/IsaacLab/scripts/demos/` |

## 기여

각 데모 폴더의 README는 아래 구조를 따릅니다:
- **Overview**: 데모의 목적과 핵심 개념을 기술적으로 요약
- **Architecture**: 클래스 계층, 메서드 호출 흐름, 주요 컴포넌트 상호작용
- **Source Files**: 파일별 역할 테이블
- **Key Design Decisions**: observation/reward 설계 근거, 물리 파라미터 선택 이유
- **Run**: 실행 명령어 (학습, 평가, GUI 모드)
- **Comparison**: 해당될 경우 다른 접근법과의 비교
- **Further Reading**: 공식 문서, 베이스 클래스 소스 경로
