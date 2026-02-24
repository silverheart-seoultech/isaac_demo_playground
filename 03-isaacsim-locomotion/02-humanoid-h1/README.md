# Humanoid — Unitree H1 보행 정책 배포
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

Isaac Lab에서 학습된 Unitree H1 이족보행 정책(flat terrain)을 Isaac Sim GUI 환경에서 실시간 배포하는 데모. `H1FlatTerrainPolicy` 클래스가 69차원 observation으로부터 19차원 관절 위치 action을 추론합니다. Spot(사족보행, 48D obs, 12D action)과 동일한 `PolicyController` 프레임워크를 사용하지만, DOF 수와 action scale, 물리 설정에서 차이가 있습니다.

이족보행은 사족보행보다 본질적으로 불안정(statically unstable)하므로, 명령 범위가 보수적(±0.75 m/s)이고 전진/회전만 지원합니다(횡방향 이동 없음).

## Architecture

### 클래스 계층

```
PolicyController (controllers/policy_controller.py)
    │
    └── H1FlatTerrainPolicy (robots/h1.py)
            │
            ├── _compute_observation()  ← 69D observation 벡터 구성
            ├── forward()               ← decimation 체크 → position action 적용
            └── initialize()            ← set_articulation_props=False 오버라이드

BaseSample (isaacsim.examples.interactive)
    │
    └── HumanoidExample (humanoid_example.py)
            │
            ├── setup_scene()           ← Ground plane + H1 USD 로드
            ├── setup_post_load()       ← 키보드 리스너, 물리 콜백 등록
            ├── on_physics_step()       ← 매 물리 스텝마다 forward() 호출
            └── _sub_keyboard_event()   ← 키 입력 → base_command 갱신
```

### Inference 루프

```
Isaac Sim GUI
    │
    └── on_physics_step(step_size)     ← 200Hz (physics_dt = 1/200)
            │
            ├── [첫 프레임] h1.initialize() + post_reset()
            │       └── set_articulation_props=False (USD 기본값 사용)
            │
            └── [이후] h1.forward(dt, base_command)
                    │
                    ├── if policy_counter % decimation == 0:
                    │       ├── _compute_observation(command)  ← 69D obs
                    │       └── _compute_action(obs)           ← TorchScript
                    │
                    └── robot.apply_action(default_pos + action × 0.5)
```

## Source Files

| 파일 | 역할 |
|---|---|
| `isaacsim.examples.interactive/.../humanoid/humanoid_example.py` | GUI 데모 메인 클래스 (115줄) |
| `isaacsim.examples.interactive/.../humanoid/humanoid_example_extension.py` | Extension 등록 (GUI 메뉴 "Policy > Humanoid") |
| `isaacsim.robot.policy.examples/robots/h1.py` | H1 정책 래퍼 (69D obs, 19D action) |
| `isaacsim.robot.policy.examples/controllers/policy_controller.py` | 정책 로드/추론 베이스 클래스 |
| `Nucleus: Isaac/Samples/Policies/H1_Policies/h1_policy.pt` | 사전학습된 TorchScript 정책 |
| `Nucleus: Isaac/Samples/Policies/H1_Policies/h1_env.yaml` | 환경 설정 (decimation, 관절 속성) |
| `Nucleus: Isaac/Robots/Unitree/H1/h1.usd` | H1 USD 에셋 |

## 핵심 코드 분석

### Observation Space (69D)

```python
def _compute_observation(self, command):
    # World → Body frame 변환 (Spot과 동일)
    R_BI = quat_to_rot_matrix(q_IB).transpose()
    lin_vel_b = np.matmul(R_BI, lin_vel_I)
    ang_vel_b = np.matmul(R_BI, ang_vel_I)
    gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

    obs = np.zeros(69)
    obs[:3]   = lin_vel_b                        # base 선속도 (3)
    obs[3:6]  = ang_vel_b                        # base 각속도 (3)
    obs[6:9]  = gravity_b                        # 중력 벡터 (3)
    obs[9:12] = command                          # 명령 (3)
    obs[12:31] = current_joint_pos - self.default_pos  # 관절 위치 오프셋 (19)
    obs[31:50] = current_joint_vel               # 관절 속도 (19)
    obs[50:69] = self._previous_action           # 이전 action (19)
    return obs
```

| 인덱스 | 차원 | 내용 |
|---|---|---|
| 0-2 | 3 | base 선속도 (body frame) |
| 3-5 | 3 | base 각속도 (body frame) |
| 6-8 | 3 | 중력 벡터 투영 (body frame) |
| 9-11 | 3 | 명령 (v_x, v_y, w_z) |
| 12-30 | 19 | 관절 위치 (default 대비 오프셋) |
| 31-49 | 19 | 관절 속도 |
| 50-68 | 19 | 이전 action |

**Spot(48D) vs H1(69D)**: 구조는 동일하지만 DOF 차이(12 vs 19)로 인해 관절 관련 차원이 증가합니다. H1의 19 DOF는 양 다리(각 6 DOF = 12) + 양 팔(각 3-4 DOF) + 허리(1 DOF) 등으로 구성됩니다. 추가된 상체 관절은 보행 중 균형 유지와 팔 스윙 제어에 사용됩니다.

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

**Action Scale = 0.5**: Spot(0.2)보다 2.5배 큰 스케일을 사용합니다. H1은 이족보행으로 더 큰 관절 각도 변화가 필요하며, 특히 무릎과 발목 관절에서 넓은 운동 범위가 보행에 필수적입니다.

### Initialize 오버라이드

```python
def initialize(self):
    return super().initialize(set_articulation_props=False)
```

H1은 `set_articulation_props=False`를 명시적으로 전달합니다. 이는 PolicyController의 기본 동작(env YAML에서 solver iteration, self-collision 등을 설정)을 비활성화하고, **USD 파일에 정의된 기본 articulation 속성을 그대로 사용**합니다는 의미입니다. Spot과 ANYmal은 이 오버라이드를 하지 않으므로 env YAML 설정이 적용됩니다.

이 차이는 H1 USD 에셋이 이미 적절한 solver 설정을 포함하고 있어 추가 조정이 불필요하거나, env YAML의 설정이 H1에 맞지 않기 때문일 수 있습니다.

### 키보드 제어

```python
self._input_keyboard_mapping = {
    "UP":    [0.75, 0.0, 0.0],     # 전진 v_x = +0.75 m/s
    "LEFT":  [0.0, 0.0, 0.75],     # 반시계 회전 w_z = +0.75 rad/s
    "RIGHT": [0.0, 0.0, -0.75],    # 시계 회전 w_z = -0.75 rad/s
}
```

**Spot과의 차이점**:
1. **후진/횡방향 이동 없음**: 이족보행 정책이 전진과 회전만 지원합니다. 후방/횡방향 보행은 추가 학습이 필요합니다.
2. **속도 범위 감소**: ±0.75 m/s (Spot은 ±2.0 m/s). 이족보행은 안정성 마진이 작아 보수적인 속도 제한이 필요합니다.
3. **키 매핑 축소**: 6개 방향 → 3개 방향. 지원하는 명령 공간이 축소되었습니다.

### 물리 설정

| 파라미터 | 값 | Spot 대비 |
|---|---|---|
| `physics_dt` | 1/200 s (200Hz) | Spot 1/500 (2.5배 낮음) |
| `rendering_dt` | 8/200 s = 1/25 s | Spot 1/50 (절반) |
| `action_scale` | 0.5 | Spot 0.2 (2.5배 큼) |
| `_previous_action` 크기 | 19 | Spot 12 |
| Ground friction | 0.2 / 0.2 | 동일 |

**physics_dt = 1/200**: H1의 물리 주파수(200Hz)가 Spot(500Hz)보다 낮은 이유는 두 가지로 추정됩니다:
1. H1의 관절 수(19)가 Spot(12)보다 많아 물리 계산 비용이 높다
2. 이족보행의 지면 접촉 패턴이 사족보행보다 단순(2점 vs 4점)하여 상대적으로 낮은 주파수로도 충분할 수 있습니다

**rendering_dt = 1/25**: Spot(1/50)보다 낮은 렌더링 주파수를 사용합니다. H1의 높은 DOF로 인한 렌더링 부하를 고려한 것입니다.

### H1 USD 에셋 로딩

```python
assets_root_path = get_assets_root_path()
self.h1 = H1FlatTerrainPolicy(
    prim_path="/World/H1",
    name="H1",
    usd_path=assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd",
    position=np.array([0, 0, 1.05]),
)
```

**초기 높이 = 1.05m**: Spot(0.8m)보다 높습니다. H1의 standing height가 약 1.0m이므로, 0.05m의 여유를 두고 스폰합니다. 초기 높이가 너무 낮으면 지면에 침투(penetration)하여 불안정한 시작 상태가 되고, 너무 높으면 착지 충격으로 정책이 실패할 수 있습니다.

## 실행 방법

```bash
# Isaac Sim GUI 실행
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI 내 메뉴: Isaac Examples > Policy > Humanoid
# Load → Play → 키보드로 조작
```

### 키보드 조작

| 키 | 명령 |
|---|---|
| `↑` / `Numpad 8` | 전진 (+0.75 m/s) |
| `←` / `Numpad 4` | 반시계 회전 (+0.75 rad/s) |
| `→` / `Numpad 6` | 시계 회전 (-0.75 rad/s) |

## Comparison: Spot vs H1

| 항목 | Spot (`01-quadruped-spot/`) | H1 (이 문서) |
|---|---|---|
| 보행 유형 | 사족보행 (statically stable) | 이족보행 (dynamically stable) |
| DOF | 12 | 19 |
| Observation 차원 | 48D | 69D |
| 명령 자유도 | 3 (v_x, v_y, w_z) | 2 (v_x, w_z) — 횡방향 없음 |
| 명령 범위 | ±2.0 m/s, ±2.0 rad/s | ±0.75 m/s, ±0.75 rad/s |
| Action Scale | 0.2 (보수적) | 0.5 (넓은 범위) |
| 물리 주파수 | 500Hz | 200Hz |
| Articulation Props | env YAML에서 설정 | USD 기본값 사용 |
| 안정성 | 높음 (4점 지지) | 낮음 (2점 지지, ZMP 균형 필요) |

## Further Reading

- **H1FlatTerrainPolicy**: `~/workspace/IsaacSim/exts/isaacsim.robot.policy.examples/.../robots/h1.py`
- **HumanoidExample**: `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/.../humanoid/humanoid_example.py`
- **PolicyController 베이스**: `~/workspace/IsaacSim/exts/isaacsim.robot.policy.examples/.../controllers/policy_controller.py`
- **Isaac Lab H1 학습 환경**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/h1/`
- **정책 배포 튜토리얼**: https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html
