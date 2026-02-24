# RL 프레임워크 비교 — RSL-RL, SKRL, Stable Baselines3, RL Games

## 개요

Isaac Lab은 4개의 RL 라이브러리를 공식 지원한다. 각 프레임워크는 `scripts/reinforcement_learning/{framework}/train.py`와 `play.py`로 통합되어, 동일한 `--task` 인자로 어떤 프레임워크든 사용할 수 있다.

```bash
# 모든 프레임워크가 동일한 task를 지원
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Ant-Direct-v0
./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py --task Isaac-Ant-Direct-v0
./isaaclab.sh -p scripts/reinforcement_learning/sb3/train.py --task Isaac-Ant-Direct-v0
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py --task Isaac-Ant-Direct-v0
```

## 프레임워크 비교

| 항목 | RSL-RL | SKRL | Stable Baselines3 | RL Games |
|---|---|---|---|---|
| **개발** | RSL (ETH Zurich) | Toni-SM | DLR / Stable Baselines | NVIDIA |
| **알고리즘** | PPO | PPO, SAC, TD3, A2C, ... | PPO, SAC, A2C, DQN, ... | PPO |
| **GPU 학습** | 네 (native) | 네 (native) | 부분적 | 네 (native) |
| **Vectorized Env** | 네 (Isaac Lab native) | 네 (wrapper) | 네 (VecEnv wrapper) | 네 (native) |
| **주요 용도** | 보행 RL (ETH 연구실 표준) | 범용 RL 연구 | 범용 RL (교육/벤치마크) | NVIDIA 내부 |
| **설정 방식** | Python dict | Python dict / YAML | Python dict | YAML |
| **Asymmetric critic** | 지원 | 지원 | 미지원 | 지원 |
| **RNN policy** | LSTM 지원 | LSTM/GRU 지원 | 미지원 | LSTM 지원 |

## RSL-RL

ETH Zurich의 Robotic Systems Lab에서 개발한 PPO 구현. ANYmal, Go1 등 실제 로봇의 보행 정책 학습에 사용되어 검증된 프레임워크.

### 특징

- **PPO 전문**: PPO 알고리즘에 최적화된 간결한 구현
- **Asymmetric Actor-Critic**: Actor(policy)와 Critic(value)이 다른 observation을 받을 수 있음. 학습 시에만 접근 가능한 privileged information을 critic에 제공
- **체크포인트 호환성**: 학습된 모델을 ONNX로 변환하여 실제 로봇에 배포 가능

### 학습 커맨드

```bash
cd ~/workspace/IsaacLab

# 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0 \
    --num_envs 4096

# 평가 (사전학습 체크포인트)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0 \
    --use_pretrained_checkpoint \
    --num_envs 32
```

### Agent Config 예시

```python
@configclass
class CartpoleRslRlPpoCfg:
    seed = 42
    num_steps_per_env = 16          # rollout 길이
    max_iterations = 200            # 총 학습 반복
    policy = RslRlPpoActorCriticCfg(
        actor_hidden_dims = [32, 32],
        critic_hidden_dims = [32, 32],
        activation = "elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        learning_rate = 1e-3,
        discount_factor = 0.99,
        clip_param = 0.2,            # PPO clipping
        entropy_coef = 0.005,
        num_mini_batches = 4,
        num_learning_epochs = 8,
    )
```

## SKRL

범용 RL 라이브러리. 다양한 알고리즘(PPO, SAC, TD3, A2C 등)을 지원하여, PPO 외의 알고리즘 실험이 필요할 때 유용하다.

### 특징

- **다중 알고리즘**: PPO뿐 아니라 SAC, TD3 등 off-policy 알고리즘도 지원
- **Isaac Lab 네이티브 통합**: `SkrlVecEnvWrapper`로 Isaac Lab 환경을 직접 래핑
- **Custom model**: PyTorch 모델을 직접 정의하여 사용 가능

### 학습 커맨드

```bash
./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 4096
```

## Stable Baselines3 (SB3)

가장 널리 사용되는 RL 라이브러리. 교육 목적과 벤치마크에 적합하지만, Isaac Lab의 GPU 가속을 완전히 활용하지 못하는 제한이 있다.

### 특징

- **표준 API**: OpenAI Gym 호환 인터페이스
- **풍부한 문서**: 가장 많은 튜토리얼과 커뮤니티 지원
- **제한**: GPU tensorized 환경과의 데이터 전송 오버헤드, RNN policy 미지원

### 학습 커맨드

```bash
./isaaclab.sh -p scripts/reinforcement_learning/sb3/train.py \
    --task Isaac-Cartpole-Direct-v0 \
    --num_envs 4096
```

### 주의사항

SB3는 내부적으로 numpy 배열 기반으로 동작하므로, Isaac Lab의 GPU tensor를 CPU로 복사하는 오버헤드가 발생한다. 대규모 환경(4096+)에서는 RSL-RL이나 SKRL이 더 효율적이다.

## RL Games

NVIDIA에서 개발한 RL 라이브러리. NVIDIA 내부에서 IsaacGymEnvs와 함께 사용되어 왔다.

### 특징

- **NVIDIA 최적화**: GPU 메모리 관리, 텐서 연산에 최적화
- **YAML 기반 설정**: 학습 파라미터를 YAML 파일로 관리
- **Legacy 호환**: IsaacGymEnvs에서 사용하던 환경과 호환

### 학습 커맨드

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py \
    --task Isaac-Ant-Direct-v0 \
    --num_envs 4096
```

## 선택 가이드

### 보행 (Locomotion)

**RSL-RL (권장)**. ETH의 보행 연구에서 검증된 PPO 구현. Asymmetric critic으로 privileged information 활용 가능. 실제 로봇 배포 파이프라인(ONNX)이 잘 갖춰져 있다.

### 매니퓰레이션 (Manipulation)

**RSL-RL 또는 SKRL**. PPO가 충분한 경우 RSL-RL, SAC 같은 off-policy 알고리즘이 필요한 경우 SKRL.

### 교육/벤치마크

**Stable Baselines3**. 문서와 커뮤니티 지원이 풍부. 하지만 대규모 병렬 학습에서는 성능이 떨어진다.

### 커스텀 알고리즘 연구

**SKRL**. 다양한 알고리즘을 통일된 인터페이스로 비교 가능. 커스텀 모델 정의가 유연하다.

## 학습 결과 디렉토리 구조

```
logs/
├── rsl_rl/
│   └── Isaac-Velocity-Flat-Anymal-C-v0/
│       └── 2025-02-24_16-30-00/
│           ├── model_1500.pt          # 체크포인트
│           ├── params/                 # 환경 config 스냅샷
│           └── summaries/              # TensorBoard 로그
├── skrl/
│   └── ...
├── sb3/
│   └── ...
└── rl_games/
    └── ...
```

모든 프레임워크가 `logs/{framework}/{task}/` 하위에 체크포인트와 로그를 저장한다. TensorBoard로 학습 과정을 모니터링할 수 있다:

```bash
tensorboard --logdir logs/rsl_rl/Isaac-Velocity-Flat-Anymal-C-v0/
```

## 사전학습 체크포인트

Isaac Lab은 각 환경에 대해 사전학습된 체크포인트를 제공한다:

```bash
# 학습 없이 사전학습 정책 실행
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0 \
    --use_pretrained_checkpoint \
    --num_envs 32
```

`--use_pretrained_checkpoint` 플래그가 Nucleus 서버에서 미리 학습된 모델을 자동 다운로드하여 실행한다.
