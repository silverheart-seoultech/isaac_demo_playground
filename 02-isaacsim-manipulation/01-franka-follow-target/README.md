# Franka Follow Target — RMPflow 기반 End-Effector 추종

## Overview

Franka Emika Panda 로봇이 GUI에서 드래그 가능한 타겟 마커를 실시간으로 추종하는 데모. 핵심은 **RMPflow (Riemannian Motion Policies)** 컨트롤러로, task-space의 end-effector 목표 위치/방향을 joint-space action으로 변환한다. 장애물 추가/제거, 데이터 로깅 기능도 포함된다.

이 데모는 Isaac Sim의 **Task 기반 아키텍처**를 보여준다. `FollowTargetTask`가 씬 구성(로봇, 타겟)과 observation 제공을 담당하고, `RMPFlowController`가 모션 생성을 담당하는 분리된 구조다.

## Architecture

### 클래스 구성

```
BaseSample
    │
    └── FollowTarget (follow_target.py)
            │
            ├── setup_scene()
            │       └── world.add_task(FollowTargetTask())
            │
            ├── setup_post_load()
            │       ├── FollowTargetTask에서 Franka 참조 획득
            │       └── RMPFlowController(robot_articulation=franka) 생성
            │
            └── _on_follow_target_simulation_step()
                    ├── observations = world.get_observations()
                    │       └── target의 position, orientation
                    ├── actions = controller.forward(target_pos, target_orient)
                    │       └── RMPflow: task-space → joint-space
                    └── articulation_controller.apply_action(actions)
```

### 시뮬레이션 루프

```
GUI에서 타겟 마커 드래그
    │
    └── 매 물리 스텝:
            ├── world.get_observations()
            │       └── target의 현재 world pose (position + quaternion)
            │
            ├── RMPFlowController.forward(target_pos, target_orient)
            │       └── Riemannian Motion Policy로 충돌 회피 + 목표 추종
            │       └── → ArticulationAction (joint positions)
            │
            └── articulation_controller.apply_action(actions)
                    └── Franka 관절에 위치 명령 적용
```

## Source Files

| 파일 | 역할 |
|---|---|
| `isaacsim.examples.interactive/.../follow_target/follow_target.py` | 데모 메인 클래스 (112줄) |
| `isaacsim.examples.interactive/.../follow_target/follow_target_extension.py` | Extension 등록 |
| `isaacsim.robot.manipulators.examples/franka/tasks/follow_target.py` | FollowTargetTask (씬 구성, observation) |
| `isaacsim.robot.manipulators.examples/franka/controllers/rmpflow_controller.py` | RMPFlowController |

## 핵심 코드 분석

### Task-Controller 분리 패턴

```python
class FollowTarget(BaseSample):
    def setup_scene(self):
        world.add_task(FollowTargetTask())    # Task가 씬 구성

    async def setup_post_load(self):
        self._franka_task = list(self._world.get_current_tasks().values())[0]
        self._task_params = self._franka_task.get_params()
        my_franka = self._world.scene.get_object(self._task_params["robot_name"]["value"])
        self._controller = RMPFlowController(
            name="target_follower_controller",
            robot_articulation=my_franka
        )
```

**Task**: `FollowTargetTask`가 Franka 로봇과 드래그 가능한 타겟 마커를 씬에 배치하고, 매 스텝 타겟의 position/orientation을 observation으로 제공한다. Task는 **무엇을 해야 하는가**(목표 정의)를 담당한다.

**Controller**: `RMPFlowController`가 observation(타겟 위치)을 받아 관절 명령을 생성한다. Controller는 **어떻게 해야 하는가**(모션 생성)를 담당한다.

이 분리는 동일한 Task에 다른 Controller(예: IK, 강화학습 정책)를 교체할 수 있게 한다.

### RMPflow Controller

```python
def _on_follow_target_simulation_step(self, step_size):
    observations = self._world.get_observations()
    actions = self._controller.forward(
        target_end_effector_position=observations[target_name]["position"],
        target_end_effector_orientation=observations[target_name]["orientation"],
    )
    self._articulation_controller.apply_action(actions)
```

**RMPflow (Riemannian Motion Policies)**: 전통적인 IK와 달리, RMPflow는 여러 목표(end-effector 추종, 충돌 회피, 관절 리밋 유지, 자기충돌 방지)를 Riemannian 공간에서 통합하여 하나의 가속도 명령을 생성한다. 각 목표가 독립적인 "policy"로 작동하며, 이들이 기하학적으로 결합된다.

### 장애물 동적 추가/제거

```python
def _on_add_obstacle_event(self):
    cube = current_task.add_obstacle()       # Task가 씬에 장애물 추가
    self._controller.add_obstacle(cube)      # Controller에 장애물 등록

def _on_remove_obstacle_event(self):
    obstacle_to_delete = current_task.get_obstacle_to_delete()
    self._controller.remove_obstacle(obstacle_to_delete)  # Controller에서 제거
    current_task.remove_obstacle()                         # 씬에서 제거
```

RMPflow는 런타임에 장애물을 동적으로 추가/제거할 수 있다. 장애물이 추가되면 Controller 내부에 새로운 충돌 회피 policy가 생성되어, 기존 목표(타겟 추종)와 자동으로 통합된다.

### Data Logger

```python
def frame_logging_func(tasks, scene):
    return {
        "joint_positions": scene.get_object(robot_name).get_joint_positions().tolist(),
        "applied_joint_positions": scene.get_object(robot_name).get_applied_action().joint_positions.tolist(),
        "target_position": scene.get_object(target_name).get_world_pose()[0].tolist(),
    }
data_logger.add_data_frame_logging_func(frame_logging_func)
```

매 프레임 관절 위치, 적용된 명령, 타겟 위치를 기록한다. `data_logger.save(log_path)`로 JSON 파일로 저장할 수 있으며, 이후 경로 분석이나 디버깅에 활용된다.

## 실행 방법

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI: Isaac Examples > Manipulation > Follow Target
# 1. Load
# 2. Follow Target 버튼 클릭
# 3. Viewport에서 타겟 마커 드래그
# 4. Add Obstacle / Remove Obstacle로 장애물 관리
```

## Further Reading

- **RMPFlowController**: `~/workspace/IsaacSim/exts/isaacsim.robot.manipulators.examples/.../franka/controllers/rmpflow_controller.py`
- **FollowTargetTask**: `~/workspace/IsaacSim/exts/isaacsim.robot.manipulators.examples/.../franka/tasks/follow_target.py`
- **RMPflow 논문**: Cheng et al., "RMPflow: A Computational Graph for Automatic Motion Policy Generation" (WAFR 2018)
