# Quadruped — Boston Dynamics Spot 보행 정책 배포
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

Isaac Lab에서 학습된 Spot 보행 정책(flat terrain)을 Isaac Sim GUI 환경에서 실시간 배포하는 데모. `SpotFlatTerrainPolicy` 클래스가 사전학습된 TorchScript 정책을 로드하고, 48차원 observation을 구성하여 12차원 관절 위치 action을 추론합니다. 키보드 입력으로 전후/좌우/회전 명령을 실시간 제어할 수 있습니다.

이 데모는 **학습(training)이 아닌 배포(deployment)** 예제입니다. 정책은 이미 학습되어 `.pt` 파일로 제공되며, 시뮬레이션에서는 observation 구성 → 정책 추론 → action 적용의 inference 루프만 실행합니다.

## Architecture

### 클래스 계층

```
BaseController (isaacsim.core.api)
    │
    └── PolicyController (controllers/policy_controller.py)
            │   ├── load_policy()         ← TorchScript 정책 + env YAML 로드
            │   ├── initialize()          ← Articulation 초기화, 게인/리밋 설정
            │   ├── _compute_action()     ← torch.jit 정책 추론 (obs → action)
            │   └── _set_articulation_props()  ← solver iteration, self-collision 등
            │
            └── SpotFlatTerrainPolicy (robots/spot.py)
                    │
                    ├── _compute_observation()  ← 48D observation 벡터 구성
                    └── forward()               ← decimation 체크 → action 적용

BaseSample (isaacsim.examples.interactive)
    │
    └── QuadrupedExample (quadruped_example.py)
            │
            ├── setup_scene()           ← Ground plane + Spot 생성
            ├── setup_post_load()       ← 키보드 리스너 등록, 물리 콜백 등록
            ├── on_physics_step()       ← 매 물리 스텝마다 forward() 호출
            └── _sub_keyboard_event()   ← 키 입력 → base_command 갱신
```

### Inference 루프

```
Isaac Sim GUI
    │
    ├── Play 버튼 클릭
    │   └── on_physics_step(step_size)     ← 500Hz (physics_dt = 1/500)
    │           │
    │           ├── [첫 프레임] initialize() + post_reset()
    │           │       └── Articulation 초기화, 관절 게인/리밋 설정
    │           │
    │           └── [이후] spot.forward(dt, base_command)
    │                   │
    │                   ├── if policy_counter % decimation == 0:
    │                   │       ├── _compute_observation(command)   ← 48D obs
    │                   │       ├── _compute_action(obs)            ← TorchScript 추론
    │                   │       └── _previous_action = action
    │                   │
    │                   └── robot.apply_action(default_pos + action × 0.2)
    │
    └── 키보드 입력
        └── _sub_keyboard_event()
                └── base_command += / -= key_mapping[key]
```

## Source Files

| 파일 | 역할 |
|---|---|
| `isaacsim.examples.interactive/.../quadruped/quadruped_example.py` | GUI 데모 메인 클래스 (BaseSample 상속, 122줄) |
| `isaacsim.examples.interactive/.../quadruped/quadruped_example_extension.py` | Extension 등록 (GUI 메뉴 "Policy > Quadruped") |
| `isaacsim.robot.policy.examples/robots/spot.py` | Spot 정책 래퍼 (observation 구성, forward 루프) |
| `isaacsim.robot.policy.examples/controllers/policy_controller.py` | 정책 로드/추론 베이스 클래스 |
| `Nucleus: Isaac/Samples/Policies/Spot_Policies/spot_policy.pt` | 사전학습된 TorchScript 정책 |
| `Nucleus: Isaac/Samples/Policies/Spot_Policies/spot_env.yaml` | 환경 설정 (decimation, dt, 관절 속성) |
| `Nucleus: Isaac/Robots/BostonDynamics/spot/spot.usd` | Spot USD 에셋 |

## 핵심 코드 분석

### Observation Space (48D)

```python
def _compute_observation(self, command):
    lin_vel_I = self.robot.get_linear_velocity()
    ang_vel_I = self.robot.get_angular_velocity()
    pos_IB, q_IB = self.robot.get_world_pose()

    R_IB = quat_to_rot_matrix(q_IB)
    R_BI = R_IB.transpose()
    lin_vel_b = np.matmul(R_BI, lin_vel_I)     # world → body frame 변환
    ang_vel_b = np.matmul(R_BI, ang_vel_I)
    gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

    obs = np.zeros(48)
    obs[:3]   = lin_vel_b                        # base 선속도 (body frame)
    obs[3:6]  = ang_vel_b                        # base 각속도 (body frame)
    obs[6:9]  = gravity_b                        # 중력 방향 (body frame)
    obs[9:12] = command                          # 사용자 명령 (v_x, v_y, w_z)
    obs[12:24] = current_joint_pos - self.default_pos  # 관절 위치 오프셋 (12 DOF)
    obs[24:36] = current_joint_vel               # 관절 속도 (12 DOF)
    obs[36:48] = self._previous_action           # 이전 action (12 DOF)
    return obs
```

| 인덱스 | 차원 | 내용 | 프레임 |
|---|---|---|---|
| 0-2 | 3 | base 선속도 | Body |
| 3-5 | 3 | base 각속도 | Body |
| 6-8 | 3 | 중력 벡터 투영 | Body |
| 9-11 | 3 | 명령 (v_x, v_y, w_z) | Body |
| 12-23 | 12 | 관절 위치 (default 대비 오프셋) | Joint |
| 24-35 | 12 | 관절 속도 | Joint |
| 36-47 | 12 | 이전 action | - |

**좌표 변환**: World frame의 속도/중력을 body frame으로 변환합니다(`R_BI = R_IB.T`). 정책이 body-centric observation으로 학습되었기 때문에, 배포 시에도 동일한 변환을 적용해야 합니다. 이는 로봇의 절대 위치/방향에 무관하게 상대적인 상태를 관찰하므로, 정책의 일반화 성능을 높입니다.

**관절 위치 오프셋**: `current_joint_pos - self.default_pos`로 기본 자세 대비 편차를 사용합니다. 이는 정책이 "현재 위치"가 아닌 "기본 자세로부터의 이탈"을 학습하므로, default standing pose를 기준으로 보정 action을 출력하게 됩니다.

**이전 action**: `_previous_action`을 observation에 포함하여 정책이 자신의 이전 출력을 참조할 수 있게 합니다. 이는 action의 연속성을 유지하고 급격한 관절 변화(jerk)를 줄이는 데 기여합니다.

### Forward (Action 적용)

```python
def forward(self, dt, command):
    if self._policy_counter % self._decimation == 0:
        obs = self._compute_observation(command)
        self.action = self._compute_action(obs)
        self._previous_action = self.action.copy()

    action = ArticulationAction(
        joint_positions=self.default_pos + (self.action * self._action_scale)
    )
    self.robot.apply_action(action)
    self._policy_counter += 1
```

**Decimation**: 물리 시뮬레이션은 500Hz로 실행되지만, 정책 추론은 `decimation` 스텝마다 1회만 수행합니다. env YAML에서 지정된 decimation 값에 따라 실제 제어 주파수가 결정됩니다. 예를 들어 decimation=4이면 제어 주파수는 125Hz가 됩니다.

**Action Scale**: `action_scale = 0.2`로, 정책 출력(보통 [-1, 1] 범위)에 0.2를 곱하여 실제 관절 위치 오프셋을 계산합니다. 최종 관절 위치는 `default_pos + action × 0.2`이 됩니다. 스케일이 작을수록 보수적인 움직임이 되므로, 안전한 보행을 보장합니다.

**Position Control**: Spot은 관절 위치 제어(position control) 방식을 사용합니다. 정책이 목표 관절 위치를 출력하면, PhysX의 PD 컨트롤러가 해당 위치로 관절을 구동합니다. 이는 ANYmal의 토크 제어(effort control) 방식과 대비됩니다.

### 키보드 제어

```python
self._input_keyboard_mapping = {
    "UP":       [2.0, 0.0, 0.0],    # 전진 v_x = +2.0 m/s
    "DOWN":     [-2.0, 0.0, 0.0],   # 후진 v_x = -2.0 m/s
    "RIGHT":    [0.0, -2.0, 0.0],   # 우측 v_y = -2.0 m/s
    "LEFT":     [0.0, 2.0, 0.0],    # 좌측 v_y = +2.0 m/s
    "N":        [0.0, 0.0, 2.0],    # 반시계 회전 w_z = +2.0 rad/s
    "M":        [0.0, 0.0, -2.0],   # 시계 회전 w_z = -2.0 rad/s
}
```

키보드 이벤트는 **additive** 방식으로 동작합니다. KEY_PRESS 시 해당 명령이 `_base_command`에 더해지고, KEY_RELEASE 시 빼진다. 따라서 여러 키를 동시에 누르면 명령이 합산되어 대각선 이동이나 회전하며 전진하는 복합 명령이 가능합니다.

명령 크기가 2.0 m/s로 상당히 크므로, 실제 시뮬레이션에서는 정책이 이 범위 전체를 추종하지 못할 수 있습니다. 학습 시 명령 범위와 일치하는지 확인이 필요합니다.

### 물리 설정

| 파라미터 | 값 | 의미 |
|---|---|---|
| `physics_dt` | 1/500 s | PhysX 시뮬레이션 500Hz |
| `rendering_dt` | 10/500 s = 1/50 s | 렌더링 50Hz |
| `action_scale` | 0.2 | 정책 출력 → 관절 위치 스케일 |
| Ground friction (static) | 0.2 | 지면 정적 마찰 |
| Ground friction (dynamic) | 0.2 | 지면 동적 마찰 |
| Ground restitution | 0.01 | 지면 반발 계수 (거의 없음) |

**physics_dt = 1/500**: Spot의 물리 시뮬레이션은 500Hz로 실행됩니다. 이는 12개 관절을 가진 사족보행 로봇의 빠른 다리 동역학을 정확히 시뮬레이션하기 위함입니다. CartPole(1/120)이나 Franka(1/400)보다 높은 주파수를 사용합니다.

**rendering_dt = 1/50**: 물리 대비 렌더링은 10배 낮은 주파수로 실행됩니다. 시각적 업데이트는 50Hz면 충분하며, GPU 리소스를 절약합니다.

### PolicyController 베이스 클래스

```python
class PolicyController(BaseController):
    def load_policy(self, policy_file_path, policy_env_path):
        file_content = omni.client.read_file(policy_file_path)[2]
        file = io.BytesIO(memoryview(file_content).tobytes())
        self.policy = torch.jit.load(file)              # TorchScript 모델 로드
        self.policy_env_params = parse_env_config(policy_env_path)  # env YAML 파싱
        self._decimation, self._dt, self.render_interval = get_physics_properties(...)

    def _compute_action(self, obs):
        with torch.no_grad():
            obs = torch.from_numpy(obs).view(1, -1).float()
            action = self.policy(obs).detach().view(-1).numpy()
        return action

    def initialize(self, ...):
        self.robot.initialize()
        # effort mode, control mode 설정
        # env YAML에서 stiffness, damping, max_effort, default_pos 로드
        # articulation root 속성 설정 (solver iteration, self-collision 등)
```

**정책 로딩 경로**: Nucleus 서버(`omni.client.read_file`)에서 `.pt` 파일을 읽어 `torch.jit.load`로 로드합니다. 로컬 파일이 아닌 Nucleus 경로를 사용하므로, 동일한 정책을 여러 머신에서 공유할 수 있습니다.

**env YAML**: 정책 학습 시 사용된 환경 설정(decimation, 관절 게인, 리밋 등)을 배포 시에도 동일하게 적용합니다. 학습-배포 간 설정 불일치는 정책 실패의 주요 원인입니다.

## 실행 방법

```bash
# Isaac Sim GUI 실행
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI 내 메뉴: Isaac Examples > Policy > Quadruped
# Load → Play → 키보드로 조작
```

### 키보드 조작

| 키 | 명령 |
|---|---|
| `↑` / `Numpad 8` | 전진 (+2.0 m/s) |
| `↓` / `Numpad 2` | 후진 (-2.0 m/s) |
| `←` / `Numpad 4` | 좌측 이동 (+2.0 m/s) |
| `→` / `Numpad 6` | 우측 이동 (-2.0 m/s) |
| `N` / `Numpad 7` | 반시계 회전 (+2.0 rad/s) |
| `M` / `Numpad 9` | 시계 회전 (-2.0 rad/s) |

## Comparison: Spot vs H1 vs ANYmal 정책 배포

| 항목 | Spot (이 문서) | H1 (`02-humanoid-h1/`) | ANYmal (`03-anymal-standalone/`) |
|---|---|---|---|
| 로봇 유형 | 사족보행 (12 DOF) | 이족보행 (19 DOF) | 사족보행 (12 DOF) |
| Observation 차원 | 48D | 69D | 48D |
| Action 차원 | 12 (관절 위치) | 19 (관절 위치) | 12 (토크) |
| Action Scale | 0.2 | 0.5 | 0.5 |
| 제어 방식 | Position Control | Position Control | Effort (Torque) Control |
| Actuator Network | 없음 | 없음 | LSTM SEA Network |
| physics_dt | 1/500 | 1/200 | 1/200 |
| 명령 범위 | ±2.0 m/s, ±2.0 rad/s | ±0.75 m/s, ±0.75 rad/s | ±1.0 m/s, ±1.0 rad/s |
| 실행 방식 | GUI (Interactive) | GUI (Interactive) | Standalone Script |

## Further Reading

- **PolicyController 베이스 클래스**: `~/workspace/IsaacSim/exts/isaacsim.robot.policy.examples/.../controllers/policy_controller.py`
- **SpotFlatTerrainPolicy**: `~/workspace/IsaacSim/exts/isaacsim.robot.policy.examples/.../robots/spot.py`
- **QuadrupedExample**: `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/.../quadruped/quadruped_example.py`
- **정책 배포 튜토리얼**: https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html
