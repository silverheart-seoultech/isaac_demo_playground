# CLI 명령어 참조

## Isaac Lab 실행 래퍼: isaaclab.sh

Isaac Lab의 모든 스크립트는 `isaaclab.sh` 래퍼를 통해 실행합니다. 이 래퍼는 Isaac Sim의 Python 환경을 자동으로 설정하고, 필요한 환경 변수를 로드합니다.

```bash
cd ~/workspace/IsaacLab
./isaaclab.sh -p <스크립트 경로> [옵션들...]
```

`-p` 플래그는 Python 스크립트를 실행한다는 의미이며, 이후의 모든 인자는 해당 스크립트에 전달됩니다.

## 학습 (Training)

### 기본 실행

```bash
# RSL-RL 프레임워크로 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 4096

# SKRL 프레임워크로 학습
./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 4096

# Stable Baselines3로 학습
./isaaclab.sh -p scripts/reinforcement_learning/sb3/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 4096
```

### 학습 스크립트 공통 인자

| 인자 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `--task` | str | None | 환경 이름 (예: `Isaac-Cartpole-Direct-v0`) |
| `--num_envs` | int | None | 병렬 환경 수. None이면 환경 config 기본값 사용 |
| `--seed` | int | None | 난수 시드. None이면 미설정 |
| `--max_iterations` | int | None | 최대 학습 반복 횟수 |
| `--distributed` | flag | False | 멀티 GPU 분산 학습 활성화 |

### RSL-RL 전용 인자

| 인자 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `--experiment_name` | str | None | 실험 로그 폴더 이름 |
| `--run_name` | str | None | 개별 실행 이름 (로그 하위 디렉토리) |
| `--resume` | flag | False | 이전 체크포인트에서 학습 재개 |
| `--load_run` | str | None | 재개할 실행 폴더 이름 |
| `--checkpoint` | str | None | 재개할 체크포인트 파일 경로 |
| `--logger` | str | tensorboard | 로거 종류: `tensorboard`, `wandb`, `neptune` |
| `--log_project_name` | str | None | wandb/neptune 프로젝트 이름 |

### 학습 영상 기록

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 4096 \
    --video \
    --video_length 200 \
    --video_interval 2000
```

| 인자 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `--video` | flag | False | 학습 중 영상 기록 활성화 |
| `--video_length` | int | 200 | 기록할 영상 길이 (step 단위) |
| `--video_interval` | int | 2000 | 영상 기록 간격 (step 단위) |

## 평가 (Play/Evaluate)

### 기본 실행

```bash
# 사전학습 체크포인트로 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 32 \
    --use_pretrained_checkpoint

# 직접 학습한 체크포인트로 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 32 \
    --checkpoint /path/to/model.pt
```

### 평가 스크립트 전용 인자

| 인자 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `--checkpoint` | str | None | 모델 체크포인트 경로 |
| `--use_pretrained_checkpoint` | flag | False | Nucleus에서 사전학습 체크포인트 다운로드 |
| `--use_last_checkpoint` | flag | False | best 대신 마지막 저장된 체크포인트 사용 |
| `--real-time` | flag | False | 실시간 속도로 시뮬레이션 |
| `--disable_fabric` | flag | False | Fabric 비활성화 (USD I/O 직접 사용) |

## AppLauncher 공통 인자

다음 인자들은 `AppLauncher`가 처리하며, 학습/평가 스크립트 모두에서 사용 가능합니다:

| 인자 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `--headless` | flag | False | 렌더링 완전 비활성화. 학습 시 필수 |
| `--device` | str | `cuda:0` | 시뮬레이션 디바이스. `cpu`, `cuda`, `cuda:N` |
| `--enable_cameras` | flag | False | 카메라 센서 활성화 (렌더링 필요) |
| `--verbose` | flag | False | 상세 로깅 활성화 |
| `--livestream` | int | 0 | 라이브스트림 모드: 0=비활성, 1=공개, 2=로컬 |
| `--rendering_mode` | str | None | 렌더링 품질: `performance`, `balanced`, `quality` |

## Task 이름 규칙

Isaac Lab에 등록된 환경 이름은 다음 패턴을 따릅니다:

| 패턴 | 예시 | 설명 |
|---|---|---|
| `Isaac-{Task}-Direct-v0` | `Isaac-Cartpole-Direct-v0` | Direct workflow 환경 |
| `Isaac-{Task}-v0` | `Isaac-Velocity-Flat-Anymal-C-v0` | Manager-Based workflow 환경 |

### 등록된 환경 목록 확인

```bash
./isaaclab.sh -p -c "
import isaaclab_tasks
import gymnasium as gym
for k in sorted(gym.registry.keys()):
    if 'Isaac' in k:
        print(k)
"
```

### 주요 환경 이름

**Classic Control:**
```
Isaac-Cartpole-Direct-v0
Isaac-Cartpole-v0              (Manager-Based)
Isaac-Ant-Direct-v0
Isaac-Humanoid-Direct-v0
```

**Locomotion:**
```
Isaac-Velocity-Flat-Anymal-C-v0
Isaac-Velocity-Flat-Anymal-C-Direct-v0
Isaac-Velocity-Rough-Anymal-C-v0
Isaac-Velocity-Flat-H1-v0
Isaac-Velocity-Flat-Unitree-Go2-v0
```

**Manipulation:**
```
Isaac-Reach-Franka-v0
Isaac-Lift-Cube-Franka-v0
Isaac-Shadow-Hand-Direct-v0
```

## 자주 사용하는 명령어 조합

### 빠른 학습 실험

```bash
# headless + 대규모 병렬 + 로그 이름 지정
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0 \
    --num_envs 4096 \
    --headless \
    --experiment_name anymal_velocity \
    --max_iterations 1500
```

### 학습 재개

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0 \
    --num_envs 4096 \
    --headless \
    --resume \
    --load_run 2024-01-15_14-30-00
```

### GUI로 정책 시각화

```bash
# GUI 모드로 평가 (headless 없이 실행)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0 \
    --num_envs 16 \
    --checkpoint logs/rsl_rl/anymal_velocity/.../model_1500.pt
```

### TensorBoard 확인

```bash
tensorboard --logdir logs/rsl_rl/
# 브라우저에서 http://localhost:6006 접속
```

## Isaac Sim 데모 실행

Isaac Sim GUI 데모는 `isaaclab.sh`가 아닌 Isaac Sim 자체를 실행합니다:

```bash
# GUI 데모 실행
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# Standalone 스크립트 실행
cd ~/workspace/IsaacSim
python standalone_examples/api/isaacsim.robot.manipulators.examples/franka/follow_target.py
```

## isaaclab.sh 유틸리티 명령어

```bash
# Python 한 줄 실행
./isaaclab.sh -p -c "import isaaclab; print(isaaclab.__version__)"

# 새 외부 프로젝트 템플릿 생성
./isaaclab.sh --new

# Extension 설치
./isaaclab.sh -i <extension_name>

# 문서 빌드
./isaaclab.sh -d
```

## 소스 코드 참조

| 파일 | 내용 |
|---|---|
| `IsaacLab/source/isaaclab/isaaclab/app/app_launcher.py` | AppLauncher CLI 인자 정의 |
| `IsaacLab/scripts/reinforcement_learning/rsl_rl/train.py` | RSL-RL 학습 스크립트 인자 |
| `IsaacLab/scripts/reinforcement_learning/rsl_rl/play.py` | RSL-RL 평가 스크립트 인자 |
| `IsaacLab/scripts/reinforcement_learning/rsl_rl/cli_args.py` | RSL-RL 전용 CLI 인자 정의 |
