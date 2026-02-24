# Franka Lift — Manager-Based Manipulation

## Overview

Franka Panda 로봇으로 큐브를 집어 목표 위치로 들어올리는 환경. Reach 환경의 확장으로, **gripper 제어 + 물체 상호작용**이 추가됩니다. Observation에 물체 위치와 목표 위치가 포함되고, reward는 접근 → 잡기 → 들기 → 목표 추종의 단계적 구조를 가집니다.

3가지 action space variant(joint_pos, ik_abs, ik_rel)를 제공합니다. Reach와 달리 gripper action이 Binary(열기/닫기)로 추가됩니다.

## Architecture

### 상속 구조

```
ManagerBasedRLEnvCfg
    │
    └── LiftEnvCfg (lift_env_cfg.py)
            │   ├── ObjectTableSceneCfg ← robot(MISSING) + object(MISSING) + ee_frame(MISSING) + table
            │   ├── CommandsCfg         ← object_pose (목표 물체 위치)
            │   ├── ObservationsCfg     ← 5개 term (Reach + object_pos + target_pos)
            │   ├── RewardsCfg          ← 단계적 reward (reach → lift → track)
            │   ├── EventCfg            ← reset_all + reset_object_position
            │   ├── CurriculumCfg       ← action_rate/joint_vel weight 증가
            │   └── TerminationsCfg     ← time_out + object_dropping
            │
            └── FrankaCubeLiftEnvCfg (joint_pos_env_cfg.py)
                    ├── robot = FRANKA_PANDA_CFG
                    ├── arm_action = JointPositionActionCfg
                    ├── gripper_action = BinaryJointPositionActionCfg
                    ├── object = DexCube (0.8x scale)
                    └── ee_frame = FrameTransformerCfg → panda_hand
```

### Reach vs Lift 구조 비교

| | Reach | Lift |
|---|---|---|
| Scene 추가 요소 | — | object (RigidObject), ee_frame (FrameTransformer) |
| Observation | joint_pos/vel, pose_cmd, actions | + object_position, target_object_position |
| Action | arm_action only | arm_action + **gripper_action** |
| Reward | ee tracking | reach → lift → goal tracking (단계적) |
| Termination | time_out | + **object_dropping** (물체 낙하) |
| Command | ee_pose (로봇 ee 목표) | object_pose (물체 목표) |

## Source Files

| 파일 | 역할 |
|---|---|
| `lift_env_cfg.py` | 공통 베이스 config |
| `config/franka/joint_pos_env_cfg.py` | Franka + Cube + JointPos action |
| `config/franka/ik_abs_env_cfg.py` | Franka + IK absolute |
| `config/franka/ik_rel_env_cfg.py` | Franka + IK relative |
| `config/franka/__init__.py` | Gymnasium 등록 |
| `config/franka/agents/` | RSL-RL / SKRL PPO configs |
| `mdp/` | 물체 관련 MDP terms (object_ee_distance, object_is_lifted 등) |

## 핵심 코드 분석

### Observation Space (5 terms)

```python
class PolicyCfg(ObsGroup):
    joint_pos = ObsTerm(func=mdp.joint_pos_rel)
    joint_vel = ObsTerm(func=mdp.joint_vel_rel)
    object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame)
    target_object_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "object_pose"})
    actions = ObsTerm(func=mdp.last_action)
```

Reach 대비 `object_position`(물체의 현재 위치, 로봇 base frame 기준)과 `target_object_position`(물체 목표 위치)이 추가됩니다. 정책은 물체가 어디에 있고 어디로 가야 하는지를 직접 관찰합니다.

`object_position_in_robot_root_frame`는 물체 위치를 world frame이 아닌 로봇 root frame으로 변환하여 반환합니다. 이는 병렬 환경에서 각 환경의 절대 좌표가 다르므로, 로봇 기준 상대 좌표를 사용하는 것이 학습에 유리하기 때문입니다.

### Reward Decomposition — 단계적 구조

```python
class RewardsCfg:
    reaching_object                  weight=1.0    std=0.1
    lifting_object                   weight=15.0   min_height=0.04
    object_goal_tracking             weight=16.0   std=0.3, min_height=0.04
    object_goal_tracking_fine        weight=5.0    std=0.05, min_height=0.04
    action_rate                      weight=-1e-4
    joint_vel                        weight=-1e-4
```

| Reward Term | Weight | 조건 | 설계 의도 |
|---|---|---|---|
| `reaching_object` | 1.0 | 항상 | ee와 물체 간 거리 줄이기 (tanh, std=0.1) |
| `lifting_object` | **15.0** | 물체 높이 > 0.04m | 물체를 테이블에서 들어올리면 큰 보상 |
| `object_goal_tracking` | **16.0** | 높이 > 0.04m | 들어올린 후 목표로 이동 (coarse, std=0.3) |
| `object_goal_tracking_fine` | 5.0 | 높이 > 0.04m | 목표 근처 정밀 추종 (fine, std=0.05) |
| `action_rate` | -1e-4 | 항상 | action jitter 억제 |
| `joint_vel` | -1e-4 | 항상 | 관절 속도 억제 |

**단계적 Reward 설계**: 이 reward 구조는 정책이 자연스럽게 아래 순서로 학습하게 만듭니다:

```
1단계: reaching_object (w=1.0)
   → ee를 물체에 접근

2단계: lifting_object (w=15.0)
   → 물체를 잡아 0.04m 이상 들어올림 (min_height gating)

3단계: object_goal_tracking (w=16.0)
   → 들어올린 물체를 목표 위치로 이동

4단계: object_goal_tracking_fine (w=5.0)
   → 목표 근처에서 정밀 위치 조정
```

`min_height=0.04` 게이트는 핵심 설계입니다. 물체를 들어올리기 전에는 goal tracking reward가 0이므로, 정책은 먼저 접근과 들기에 집중합니다. 들어올린 후에야 goal tracking이 활성화되어 이동을 학습합니다.

### Gripper Action — Binary Control

```python
self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
    asset_name="robot",
    joint_names=["panda_finger.*"],
    open_command_expr={"panda_finger_.*": 0.04},   # 열기: 4cm
    close_command_expr={"panda_finger_.*": 0.0},    # 닫기: 0cm
)
```

Gripper를 연속 제어 대신 Binary(열기/닫기)로 단순화합니다. 정책이 gripper 제어에 대한 학습 부담을 줄이고 물체 접근/이동에 집중할 수 있습니다. 실제 pick-and-place에서도 gripper는 대부분 완전 열기/닫기로 사용됩니다.

### FrameTransformer — End-Effector 좌표

```python
self.scene.ee_frame = FrameTransformerCfg(
    prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
    target_frames=[
        FrameTransformerCfg.FrameCfg(
            prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
            name="end_effector",
            offset=OffsetCfg(pos=[0.0, 0.0, 0.1034]),  # finger tip offset
        ),
    ],
)
```

`panda_link0`(base)에서 `panda_hand`까지의 변환을 추적하며, 추가로 z방향 0.1034m offset을 적용하여 실제 finger tip 위치를 반환합니다. 이 변환은 물체와 ee 간 거리 계산에 사용됩니다.

### Object Reset Randomization

```python
reset_object_position = EventTerm(
    func=mdp.reset_root_state_uniform,
    mode="reset",
    params={
        "pose_range": {"x": (-0.1, 0.1), "y": (-0.25, 0.25), "z": (0.0, 0.0)},
        "asset_cfg": SceneEntityCfg("object", body_names="Object"),
    },
)
```

물체 위치를 매 에피소드 x±0.1m, y±0.25m 범위에서 랜덤화합니다. 높이(z)는 테이블 위에 고정. 정책이 다양한 물체 위치에서 잡기를 학습하게 합니다.

### 물리 설정

| 파라미터 | 값 | Reach 대비 |
|---|---|---|
| `dt` | 0.01 s (100Hz) | 60Hz → 100Hz (접촉 시뮬레이션에 더 높은 주파수 필요) |
| `decimation` | 2 | 동일 |
| 제어 주파수 | 50 Hz | 30Hz → 50Hz |
| `episode_length_s` | 5.0 s | 12.0 → 5.0 (lift는 빠르게 완료해야) |
| `num_envs` | 4096 | 동일 |

접촉이 포함되므로 Reach(60Hz)보다 높은 100Hz 물리를 사용합니다. 접촉 해석의 안정성을 위해 `friction_correlation_distance=0.00625`, `solver_position_iteration_count=16`을 설정하여 물체 파지의 물리적 안정성을 확보합니다.

## 실행 방법

```bash
cd ~/workspace/IsaacLab

# Joint Position variant 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Lift-Cube-Franka-v0 \
    --num_envs 4096

# IK Absolute variant 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Lift-Cube-Franka-IK-Abs-v0

# IK Relative variant 학습
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Lift-Cube-Franka-IK-Rel-v0

# 사전학습 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Lift-Cube-Franka-Play-v0 \
    --use_pretrained_checkpoint
```

### Curriculum 주의사항

학습 시 `action_rate`와 `joint_vel`의 penalty weight가 10000 스텝에 걸쳐 -1e-4에서 -1e-1로 **1000배** 증가합니다 (Reach는 50배). Lift는 물체 조작의 정밀도가 중요하므로 후반부에 매우 강한 smoothness 제약을 겁니다.

## Comparison: Reach → Lift 확장 패턴

Lift는 Reach에서 아래 요소를 추가하여 구현됩니다:

```
Reach                           Lift (추가 요소)
────────────────────────────────────────────────────
Scene:  robot + table           + object (RigidObject)
                                + ee_frame (FrameTransformer)
Obs:    joint, ee_cmd, actions  + object_pos, target_obj_pos
Action: arm only                + gripper (Binary)
Reward: ee tracking             → reach + lift + goal tracking (단계적)
Event:  joint reset             + object position reset
Term:   time_out                + object_dropping
```

이 패턴은 Isaac Lab에서 새로운 manipulation 태스크를 설계할 때 참고할 수 있는 확장 가이드라인입니다.

## Further Reading

- **Lift 베이스 config**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift/lift_env_cfg.py`
- **Franka Cube Lift**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift/config/franka/joint_pos_env_cfg.py`
- **Lift MDP terms**: `~/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift/mdp/`
- **FrameTransformer**: `isaaclab.sensors.FrameTransformer`
- **BinaryJointPositionAction**: `isaaclab.envs.mdp.BinaryJointPositionActionCfg`
