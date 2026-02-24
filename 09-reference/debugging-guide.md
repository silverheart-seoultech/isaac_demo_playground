# 디버깅 가이드

## NaN 폭발 (NaN Explosion)

RL 학습 중 reward, observation, 또는 action에 NaN이 발생하면 policy가 급격히 발산하여 학습이 중단됩니다. Isaac Lab 환경에서 NaN의 주요 원인과 디버깅 방법을 설명합니다.

### 증상

- TensorBoard에서 reward가 갑자기 NaN으로 표시
- 로봇이 물리적으로 불가능한 속도로 날아감
- `RuntimeError: Function returned nan` 또는 유사한 에러

### 주요 원인

1. **과도한 힘/토크 적용**: action scale이 너무 크거나, PD gain이 물리적으로 불안정한 영역에 있는 경우
2. **0으로 나누기**: observation 정규화 시 표준편차가 0인 경우
3. **쿼터니언 정규화 누락**: 쿼터니언의 norm이 1에서 벗어나면 회전 연산이 발산
4. **physics_dt 과대**: 물리 스텝이 너무 커서 시뮬레이션 불안정

### 디버깅 절차

**Step 1: 단일 환경 + GUI로 재현**

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0 \
    --num_envs 1 \
    --checkpoint logs/rsl_rl/.../model_xxx.pt
```

환경을 1개로 줄이고 GUI를 켜면, NaN이 발생하기 직전의 로봇 상태를 시각적으로 확인할 수 있습니다.

**Step 2: NaN 발생 지점 추적**

환경 코드에 NaN 검사를 삽입합니다:

```python
import torch

def _get_observations(self):
    obs = self._compute_obs()
    
    # NaN 검사
    if torch.isnan(obs).any():
        nan_envs = torch.where(torch.isnan(obs).any(dim=-1))[0]
        print(f"NaN detected in obs at envs: {nan_envs}")
        print(f"Joint positions: {self.scene['robot'].data.joint_pos[nan_envs]}")
        print(f"Joint velocities: {self.scene['robot'].data.joint_vel[nan_envs]}")
        print(f"Root state: {self.scene['robot'].data.root_state_w[nan_envs]}")
        breakpoint()  # 디버거 진입
    
    return obs
```

**Step 3: Reward 항목별 분리 확인**

reward 함수에서 어떤 항목이 NaN을 유발하는지 개별 확인합니다:

```python
def _get_rewards(self):
    rewards = {}
    rewards["alive"] = self._reward_alive()
    rewards["velocity"] = self._reward_velocity()
    rewards["joint_torque"] = self._reward_joint_torque()
    
    for name, value in rewards.items():
        if torch.isnan(value).any():
            print(f"NaN in reward '{name}': {value}")
    
    return sum(rewards.values())
```

**Step 4: 쿼터니언 정규화 확인**

```python
root_quat = self.scene["robot"].data.root_quat_w
quat_norm = torch.norm(root_quat, dim=-1)
if (quat_norm - 1.0).abs().max() > 0.01:
    print(f"Quaternion denormalized: norm range [{quat_norm.min()}, {quat_norm.max()}]")
```

### 예방 방법

- Observation에 `torch.clamp()`을 적용하여 범위를 제한합니다
- Action을 적용하기 전에 `torch.nan_to_num()`으로 NaN을 제거합니다
- 쿼터니언은 사용 전에 `F.normalize(quat, dim=-1)`로 정규화합니다
- `physics_dt`를 줄이고 `decimation`을 늘려 물리 안정성을 확보합니다

## TensorBoard 모니터링

### 로그 디렉토리 구조

학습 로그는 기본적으로 `logs/` 디렉토리에 저장됩니다:

```
logs/
├── rsl_rl/
│   └── {experiment_name}/
│       └── {timestamp}/
│           ├── events.out.tfevents.*    # TensorBoard 이벤트
│           ├── model_*.pt               # 체크포인트
│           └── params/                  # 하이퍼파라미터
├── skrl/
│   └── ...
└── sb3/
    └── ...
```

### TensorBoard 실행

```bash
# 특정 실험 로그 확인
tensorboard --logdir logs/rsl_rl/anymal_velocity/

# 전체 로그 확인 (실험 간 비교)
tensorboard --logdir logs/

# 원격 서버에서 실행 시 포트 포워딩
tensorboard --logdir logs/ --bind_all --port 6006
# 로컬에서: ssh -L 6006:localhost:6006 user@server
```

### 주요 관찰 지표

| 지표 | 정상 패턴 | 이상 신호 |
|---|---|---|
| `Train/mean_reward` | 단조 증가 후 수렴 | 갑작스런 drop 또는 NaN |
| `Train/mean_episode_length` | 점진적 증가 (생존형 태스크) | 고정값 (학습 미진행) |
| `Loss/value_loss` | 초기 높다가 감소 | 지속적 증가 (발산) |
| `Loss/surrogate_loss` | 0 근처에서 진동 | 큰 진폭 (불안정) |
| `Policy/mean_std` | 점진적 감소 (exploitation 증가) | 0에 수렴 (premature collapse) |
| `Perf/total_fps` | 안정적 유지 | 급격한 하락 (메모리 이슈 의심) |

### 커스텀 메트릭 로깅

Isaac Lab의 RL wrapper는 `extras` 딕셔너리를 통해 커스텀 메트릭을 TensorBoard에 기록할 수 있습니다:

```python
def step(self, action):
    obs, rewards, dones, infos = super().step(action)
    
    # extras에 "log" 키로 추가하면 TensorBoard에 자동 기록
    infos["log"] = {
        "custom/mean_joint_velocity": self.scene["robot"].data.joint_vel.abs().mean(),
        "custom/contact_count": (self.scene["contact_sensor"].data.force_matrix.norm(dim=-1) > 0.1).sum(),
    }
    
    return obs, rewards, dones, infos
```

## PhysX 버퍼 오버플로우

### 증상과 에러 메시지

```
[Error] PhysX: GPU broad phase found/lost pair buffer overflow
[Error] PhysX: GPU rigid contact buffer overflow
```

이 에러는 PhysX의 GPU 버퍼 크기가 시뮬레이션의 접촉/충돌 데이터를 담기에 부족할 때 발생합니다.

### 버퍼별 역할과 조정

| 버퍼 | 기본값 | 관련 상황 | 조정 시점 |
|---|---|---|---|
| `gpu_found_lost_pairs_capacity` | 2^21 | broad phase에서 접촉 쌍 탐지 | "found/lost pair buffer overflow" |
| `gpu_found_lost_aggregate_pairs_capacity` | 2^25 | AABB aggregate 접촉 쌍 | "aggregate found/lost pair overflow" |
| `gpu_max_rigid_contact_count` | 2^23 | rigid body 접촉 스트림 | "rigid contact buffer overflow" |
| `gpu_collision_stack_size` | 2^26 | 충돌 감지 스택 | 복잡한 메시 충돌 시 |
| `gpu_heap_capacity` | 2^26 | GPU 메모리 힙 | 전반적 메모리 부족 |
| `gpu_temp_buffer_capacity` | 2^24 | 임시 연산 버퍼 | 대규모 환경 |

### 조정 방법

에러 메시지에 해당하는 버퍼를 2배씩 증가시킵니다:

```python
from isaaclab.sim import SimulationCfg, PhysxCfg

sim_cfg = SimulationCfg(
    physx=PhysxCfg(
        gpu_found_lost_pairs_capacity=2**22,     # 2배 증가
    ),
)
```

모든 버퍼를 무조건 크게 설정하면 VRAM이 낭비됩니다. 에러 메시지가 지목하는 버퍼만 선택적으로 증가시키는 것을 권장합니다.

## 시뮬레이션 불안정 디버깅

### 로봇이 폭발하듯 날아가는 경우

1. **physics_dt 확인**: 현재 dt가 태스크에 비해 너무 큰지 확인합니다. [physics-timing.md](./physics-timing.md)의 태스크별 참고값을 참조합니다.
2. **초기 상태 확인**: 로봇의 초기 pose가 지면과 겹치거나 자기 자신과 충돌하는지 확인합니다.
3. **action scale 확인**: 첫 action에 큰 값이 들어가는지 확인합니다. policy 출력에 `tanh`를 적용하고 적절한 scale을 곱합니다.

### 로봇이 지면을 관통하는 경우

```python
# solver iteration 증가
PhysxCfg(
    min_position_iteration_count=8,   # 기본값 1
    min_velocity_iteration_count=2,   # 기본값 0
    enable_ccd=True,                  # 연속 충돌 감지 활성화
)
```

CCD(Continuous Collision Detection)를 활성화하면 빠르게 이동하는 물체의 터널링을 방지할 수 있지만, 성능 비용이 있습니다.

### joint가 불안정하게 진동하는 경우

- PD controller의 gain이 물리적으로 적절한지 확인합니다. 과도한 stiffness는 고주파 진동을 유발합니다.
- `damping` 값을 높여 진동을 감쇠시킵니다.
- solver iteration을 늘립니다.

## 학습이 수렴하지 않을 때

### 체크리스트

1. **Reward 스케일**: 각 reward 항목의 절대값 범위가 합리적인지 확인합니다. 한 항목이 나머지를 압도하면 학습이 특정 행동에만 최적화됩니다. TensorBoard에서 개별 reward 항목을 비교합니다.

2. **Observation 범위**: observation 값의 범위가 [-10, 10] 수준을 크게 벗어나지 않도록 합니다. 필요 시 정규화를 적용합니다.

3. **Action space**: action이 실제로 로봇에 영향을 미치는지 확인합니다. `action_scale`이 너무 작으면 policy의 출력이 물리에 반영되지 않습니다.

4. **Episode length**: 에피소드가 너무 짧으면 policy가 의미 있는 행동을 학습할 시간이 부족합니다.

5. **Num_envs**: 환경 수가 너무 적으면 gradient 추정이 noisy합니다. PPO 계열 알고리즘에서는 최소 1024–2048개를 권장합니다.

6. **학습률**: RSL-RL 기본값은 1e-3입니다. 복잡한 환경에서는 3e-4 ~ 1e-4로 줄여볼 수 있습니다.

## 로깅 레벨 조정

디버깅 시 Isaac Sim/Lab의 로깅 레벨을 높여 상세한 정보를 확인할 수 있습니다:

```bash
# CLI에서 verbose 모드
./isaaclab.sh -p train.py --task ... --verbose

# 또는 코드에서 설정
sim_cfg = SimulationCfg(
    logging_level="DEBUG",    # "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"
    save_logs_to_file=True,
    log_dir="/tmp/isaac_debug_logs",
)
```

## 유용한 디버깅 패턴

### 환경 1개로 격리 테스트

대부분의 버그는 단일 환경에서도 재현됩니다. `--num_envs 1`로 설정하면 디버깅이 훨씬 쉬워집니다:

```bash
./isaaclab.sh -p play.py --task Isaac-Cartpole-Direct-v0 --num_envs 1
```

### breakpoint()를 활용한 인터랙티브 디버깅

Isaac Lab 환경 코드 내부에 `breakpoint()`를 삽입하면, PDB 디버거로 실시간 텐서 값을 확인할 수 있습니다:

```python
def _get_rewards(self):
    reward = self._compute_reward()
    if (reward > 100).any():
        print(f"Abnormal reward: {reward}")
        breakpoint()  # pdb 진입
    return reward
```

### GPU 텐서를 CPU로 이동 후 확인

```python
# GPU 텐서는 print 시 값이 보이지만, 복잡한 분석은 CPU에서
joint_pos = env.scene["robot"].data.joint_pos
print(joint_pos.cpu().numpy())  # numpy로 변환하여 상세 확인
```

## 소스 코드 참조

| 파일 | 내용 |
|---|---|
| `IsaacLab/source/isaaclab/isaaclab/sim/simulation_cfg.py` | PhysxCfg GPU 버퍼 설정, SimulationCfg 로깅 설정 |
| `IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/ray_caster_camera.py` | NaN 처리 패턴 예시 |
| `IsaacLab/source/isaaclab/isaaclab/sensors/camera/utils.py` | NaN/Inf 필터링 패턴 |
| `IsaacLab/source/isaaclab_tasks/test/benchmarking/env_benchmark_test_utils.py` | TensorBoard 이벤트 처리 |
