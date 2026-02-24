# Franka Path Planning — RRT 기반 경로 계획

## Overview

Franka Emika Panda의 end-effector를 타겟 위치로 이동시키되, **RRT (Rapidly-exploring Random Tree)** 알고리즘으로 장애물을 회피하는 경로를 계획하는 데모. Follow Target(RMPflow)이 reactive하게 매 스텝 모션을 생성하는 것과 달리, RRT는 **plan-then-execute** 패턴으로 전체 경로를 먼저 계산한 후 순차적으로 실행합니다.

## Architecture

```
PathPlannerController (path_planning_controller.py)
    │
    └── FrankaRrtController
            │
            ├── RRT (Lula 기반)              ← C-space 경로 탐색
            ├── LulaCSpaceTrajectoryGenerator ← 웨이포인트 → 스플라인 궤적
            ├── PathPlannerVisualizer          ← 경로 시각화 + 보간
            └── ArticulationTrajectory         ← 궤적 → action sequence

BaseSample
    │
    └── PathPlanning (path_planning.py)
            ├── setup_scene() → FrankaPathPlanningTask (벽 장애물 포함)
            └── _on_follow_target_simulation_step()
                    ├── controller.forward(target_pos) → action_sequence.pop(0)
                    └── custom gains 적용
```

### Plan-Then-Execute 흐름

```
1. 타겟 위치 설정
    │
2. RRT.compute_path(start_pos, target) ← max_iterations=5000
    │   └── C-space에서 충돌-없는 경로 탐색
    │
3. PathPlannerVisualizer.interpolate_path(plan, max_dist=0.01)
    │   └── 웨이포인트 간 최대 L2 거리 0.01로 보간
    │
4. LulaCSpaceTrajectoryGenerator.compute_c_space_trajectory()
    │   └── 보간된 웨이포인트 → 스플라인 기반 연속 궤적
    │
5. ArticulationTrajectory.get_action_sequence()
    │   └── 궤적 → 물리 타임스텝별 ArticulationAction 리스트
    │
6. 매 물리 스텝: action_sequence.pop(0)
    └── 미리 계산된 action을 순서대로 적용
```

## Source Files

| 파일 | 역할 |
|---|---|
| `path_planning/path_planning.py` | 데모 메인 (117줄) |
| `path_planning/path_planning_controller.py` | PathPlannerController + FrankaRrtController (169줄) |
| `path_planning/path_planning_task.py` | FrankaPathPlanningTask (씬 + 벽 장애물) |
| `path_planning/path_planning_example_assets/` | 보수적 충돌 구(inflated collision spheres) 설정 |

## 핵심 코드 분석

### RRT 경로 탐색

```python
rrt_config = interface_config_loader.load_supported_path_planner_config("Franka", "RRT")

# 보수적 충돌 구 사용 (기본보다 큰 반경)
rrt_config["robot_description_path"] = os.path.join(
    examples_extension_path, ..., "franka_conservative_spheres_robot_description.yaml"
)
rrt = RRT(**rrt_config)
```

**Conservative Collision Spheres**: 기본 Franka 충돌 모델 대신 반경이 확대된 충돌 구를 사용합니다. 이는 RRT가 찾은 경로가 실제 실행 시 장애물과 충분한 안전 마진을 확보하도록 보장합니다. 스플라인 보간 과정에서 경로가 약간 벗어날 수 있으므로, 이 보수적 접근이 필수적입니다.

### 웨이포인트 → 궤적 변환

```python
def _convert_rrt_plan_to_trajectory(self, rrt_plan):
    interpolated_path = self._path_planner_visualizer.interpolate_path(
        rrt_plan, self._rrt_interpolation_max_dist    # max_dist = 0.01
    )
    trajectory = self._cspace_trajectory_generator.compute_c_space_trajectory(interpolated_path)
    art_trajectory = ArticulationTrajectory(self._robot, trajectory, self._physics_dt)
    return art_trajectory.get_action_sequence()
```

RRT는 이론적으로 웨이포인트 간 **선형 보간만 보장**합니다. 하지만 `LulaCSpaceTrajectoryGenerator`는 스플라인 기반 보간을 사용하므로, 웨이포인트가 충분히 조밀하지 않으면 스플라인이 장애물을 통과할 수 있습니다. `max_dist=0.01` 보간으로 이 문제를 완화합니다.

### Forward (Pre-computed Execution)

```python
def forward(self, target_end_effector_position, target_end_effector_orientation=None):
    if self._action_sequence is None:
        self._make_new_plan(target_end_effector_position, ...)

    if len(self._action_sequence) == 0:
        return ArticulationAction()     # 완료 → 정지

    return self._action_sequence.pop(0) # 다음 action 꺼내서 반환
```

Follow Target의 `forward()`가 매 스텝 새로운 action을 계산하는 것과 달리, 여기서는 사전 계산된 action 시퀀스를 순서대로 반환합니다.

### Custom PD Gains

```python
def _on_follow_target_simulation_step(self, step_size):
    actions = self._controller.forward(target_pos, target_orient)
    kps, kds = self._franka_task.get_custom_gains()
    self._articulation_controller.set_gains(kps, kds)
    self._articulation_controller.apply_action(actions)
```

경로 추종 정밀도를 위해 Task에서 제공하는 custom PD gains(stiffness, damping)를 설정합니다. Jerk/Acceleration 리밋이 robot description에 포함되어 있어 궤적이 실제 관절 역학 내에서 추종 가능합니다.

## 실행 방법

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI: Isaac Examples > Manipulation > Path Planning
# 1. Load
# 2. Add Wall로 장애물 배치
# 3. Plan To Target 버튼 클릭
# 4. 경로 계산 후 자동 실행
```

## Comparison: RMPflow vs RRT

| 항목 | RMPflow (Follow Target) | RRT (이 문서) |
|---|---|---|
| 패턴 | Reactive (매 스텝 계산) | Plan-then-Execute |
| 글로벌 최적성 | 없음 (로컬 최적) | 점근적 최적 (충분한 iteration) |
| 계산 시간 | O(1) per step | O(N) upfront (5000 iter) |
| 동적 장애물 | 실시간 대응 | 재계획 필요 |
| 좁은 통로 | 로컬 미니마에 빠질 수 있음 | 통과 가능 (랜덤 탐색) |
| 궤적 품질 | 부드러움 (연속) | 스플라인 보간 필요 |

## Further Reading

- **PathPlannerController**: `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/.../path_planning/path_planning_controller.py`
- **Lula RRT**: `~/workspace/IsaacSim/exts/isaacsim.robot_motion.motion_generation/.../lula/`
- **RRT 알고리즘**: LaValle, "Rapidly-Exploring Random Trees: A New Tool for Path Planning" (1998)
