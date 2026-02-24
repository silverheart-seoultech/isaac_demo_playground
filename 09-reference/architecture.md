# Isaac Sim vs Isaac Lab 아키텍처 비교

## 생태계 계층 구조

```
┌─────────────────────────────────────────────────────────────┐
│                       Isaac Lab                              │
│  RL 학습 프레임워크 — Direct/Manager-Based 환경 패러다임       │
│  RSL-RL, SKRL, RL Games, Stable Baselines3, Ray RLlib 지원   │
├─────────────────────────────────────────────────────────────┤
│                      Isaac Sim                               │
│  Omniverse 기반 물리 시뮬레이션 엔진                          │
│  PhysX 5 GPU, USD 씬, Articulation API, Sensor API          │
├─────────────────────────────────────────────────────────────┤
│                    Omniverse Kit                             │
│  Extension 시스템, OmniGraph, Nucleus Asset Server            │
│  Hydra Renderer, USD Composer, Replicator                    │
└─────────────────────────────────────────────────────────────┘
```

## Isaac Sim — 물리 시뮬레이션 엔진

### 핵심 구성 요소

| 구성 요소 | 역할 | 해당 예제 |
|---|---|---|
| **PhysX 5** | GPU 가속 rigid body, articulation, soft body 물리 | 전체 |
| **USD (Universal Scene Description)** | 씬 포맷, prim 계층구조, attribute 시스템 | `01-basics/` |
| **Articulation API** | 다관절 로봇 제어 (joint position/velocity/effort) | `02-manipulation/`, `03-locomotion/` |
| **Extension System** | 모듈식 기능 확장 (`omni.ext.IExt` 상속) | GUI 데모 전체 |
| **Sensor API** | Contact, IMU, LiDAR, Camera 센서 | `05-sensors/` |
| **Controller API** | RmpFlow IK, DifferentialController, HolonomicController 등 | `02-manipulation/`, `04-multirobot/` |

### 실행 모드

**Interactive (GUI Extension)**:
```
isaac-sim.sh → Kit App → Extension System → BaseSample
    └── setup_scene() → setup_post_load() → _on_physics_step()
```

**Standalone Script**:
```
SimulationApp(config) → World → scene.add() → world.reset()
    └── while running: world.step(render=True)
```

### BaseSample 라이프사이클

모든 Interactive 데모는 `BaseSample`을 상속한다:

```python
class MyDemo(BaseSample):
    def setup_scene(self):
        """씬 구성: USD 로딩, 로봇/오브젝트 추가"""
        pass

    async def setup_post_load(self):
        """씬 로드 후 초기화: Controller 생성, 콜백 등록"""
        pass

    def _on_physics_step(self, step_size):
        """매 물리 스텝마다 실행: observation → action → apply"""
        pass

    async def setup_pre_reset(self):
        """리셋 전 정리: 콜백 제거, Controller 리셋"""
        pass

    def world_cleanup(self):
        """완전한 정리: 모든 참조 해제"""
        pass
```

### Task 시스템

Isaac Sim의 Task는 **씬 구성 + observation 제공**을 캡슐화한다:

```python
class Stacking(BaseTask):
    def set_up_scene(self, scene):
        """로봇 + 큐브 + ground plane 추가"""

    def get_observations(self):
        """로봇/큐브의 position, orientation, joint_positions 반환"""

    def get_params(self):
        """robot_name, cube_names 등 Task 파라미터"""
```

이 패턴은 동일한 Task를 여러 번 인스턴스화하여 병렬 실행할 수 있게 한다 (예: RoboFactory의 4x Stacking).

## Isaac Lab — RL 학습 프레임워크

### 핵심 구성 요소

| 구성 요소 | 역할 | 해당 예제 |
|---|---|---|
| **DirectRLEnv** | 단일 클래스에서 MDP 전체를 명시적 구현 | `06-classic/01-cartpole-direct/` |
| **ManagerBasedRLEnv** | 선언적 config로 MDP 요소를 Manager에 위임 | `06-classic/02-cartpole-manager/` |
| **InteractiveScene** | 환경의 3D 씬 관리 (로봇, 센서, 지형) | 전체 |
| **ActionManager** | action space 정의 및 물리 명령 변환 | Manager-Based 전체 |
| **ObservationManager** | observation space 구성 (policy/critic 그룹) | Manager-Based 전체 |
| **RewardManager** | reward 함수 합산 (가중치 기반) | Manager-Based 전체 |
| **TerminationManager** | 에피소드 종료 조건 | Manager-Based 전체 |
| **CurriculumManager** | 학습 진행에 따른 난이도 조절 | `08-manipulation/` |
| **EventManager** | 리셋 시 랜덤화, 도메인 랜덤화 | 전체 |

### 환경 실행 흐름

```
isaaclab.sh -p train.py --task Isaac-Cartpole-Direct-v0 --num_envs 4096

    RL Framework (RSL-RL)
        │
        ├── env.reset() → _reset_idx()
        │
        └── for iteration in range(max_iterations):
                observations = env.get_observations()    # GPU tensor
                actions = policy(observations)            # neural network
                rewards, dones, infos = env.step(actions) # 물리 + reward 계산
```

### 병렬 환경 (Vectorized)

Isaac Lab의 환경은 단일 PhysX scene에서 수천 개의 환경을 동시 실행한다:

```
PhysX Scene
├── /World/envs/env_0/Robot, /World/envs/env_0/Object, ...
├── /World/envs/env_1/Robot, /World/envs/env_1/Object, ...
├── ...
└── /World/envs/env_4095/Robot, /World/envs/env_4095/Object, ...
```

모든 환경의 joint position, velocity, force는 **단일 GPU tensor**로 관리된다. Isaac Sim의 Task 기반 병렬화(RoboFactory)가 CPU loop으로 4개를 처리하는 것과 달리, Isaac Lab은 GPU에서 4096+ 환경을 배치 연산으로 처리한다.

## Isaac Sim vs Isaac Lab 비교 요약

| 항목 | Isaac Sim | Isaac Lab |
|---|---|---|
| **용도** | 시뮬레이션 엔진, 데모, 검증 | RL 학습, 대규모 병렬 환경 |
| **실행 단위** | 단일 씬 (수 대 로봇) | 수천 환경 동시 실행 |
| **제어 방식** | Controller API (IK, PD 등) | RL policy (neural network) |
| **데이터 형식** | numpy array | torch GPU tensor |
| **병렬화** | CPU loop (순차) | GPU batch (동시) |
| **씬 구성** | Python code / USD | InteractiveSceneCfg (선언적) |
| **센서** | 개별 API 호출 | Cfg 기반 자동 관리 |
| **physics_dt** | SimulationContext 설정 | EnvCfg.sim.dt |
| **학습** | 직접 구현 필요 | RSL-RL/SKRL/SB3 통합 |

## 코드 패턴 비교

### 로봇 추가

**Isaac Sim**:
```python
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
robot = world.scene.add(Robot(prim_path="/World/Franka", name="franka"))
```

**Isaac Lab**:
```python
class SceneCfg(InteractiveSceneCfg):
    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=UsdFileCfg(usd_path=FRANKA_USD_PATH),
        actuators={"arm": ImplicitActuatorCfg(...)},
    )
```

### 물리 스텝

**Isaac Sim**:
```python
world.step(render=True)
joint_positions = robot.get_joint_positions()
robot.set_joint_positions(target_positions)
```

**Isaac Lab**:
```python
obs, rewards, dones, infos = env.step(actions)  # actions: (4096, action_dim) GPU tensor
# 내부에서: decimation 횟수만큼 물리 스텝 반복 + reward 합산
```

### Reward 정의

**Isaac Sim**: reward 개념 없음 (Controller가 직접 제어)

**Isaac Lab Direct**:
```python
@torch.jit.script
def compute_rewards(pole_angle, cart_vel, ...):
    reward = 1.0 - pole_angle.abs() / math.pi - 0.01 * cart_vel.abs()
    return reward
```

**Isaac Lab Manager-Based**:
```python
class RewardsCfg:
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    joint_vel = RewTerm(func=mdp.joint_vel_l1, weight=-0.005, params={"asset_cfg": ...})
```
