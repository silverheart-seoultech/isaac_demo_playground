# Franka Open Drawer — RL 정책 기반 서랍 열기
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

Franka Emika Panda가 학습된 RL 정책으로 캐비닛 서랍을 여는 데모. `FrankaOpenDrawerPolicy`는 보행 정책 배포와 동일한 `PolicyController` 프레임워크를 사용하며, 9차원 action(7 관절 + 2 그리퍼)으로 서랍 핸들을 잡고 당기는 동작을 수행합니다.

10초마다 시뮬레이션이 자동 리셋되어 반복 실행됩니다. Follow Target(RMPflow)과 달리 이 데모는 학습된 정책이 모션을 생성하므로, 명시적인 task-space 목표가 아닌 관절 수준의 직접 제어입니다.

## Architecture

```
PolicyController (policy_controller.py)
    │
    └── FrankaOpenDrawerPolicy (robots/franka.py)
            │
            ├── _compute_observation()  ← 로봇 + 캐비닛 상태
            └── forward()               ← TorchScript 정책 → 관절 위치 명령

BaseSample
    │
    └── FrankaExample (franka_example.py)
            ├── setup_scene()  ← Ground + Cabinet USD + Franka 생성
            └── on_physics_step()  ← 매 400Hz 스텝마다 forward() 호출
                                     10초 경과 시 자동 리셋
```

## Source Files

| 파일 | 역할 |
|---|---|
| `isaacsim.examples.interactive/.../franka/franka_example.py` | 데모 메인 (104줄) |
| `isaacsim.robot.policy.examples/robots/franka.py` | Franka 서랍 열기 정책 래퍼 |
| `isaacsim.robot.policy.examples/controllers/policy_controller.py` | 정책 로드/추론 베이스 |
| `Nucleus: Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd` | 캐비닛 에셋 |

## 핵심 코드 분석

### 씬 구성

```python
self._world_settings["physics_dt"] = 1.0 / 400.0
self._world_settings["rendering_dt"] = 1.0 / 60.0

cabinet_usd_path = get_assets_root_path() + "/Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd"
stage_utils.add_reference_to_stage(cabinet_usd_path, cabinet_prim_path)
self.cabinet = SingleArticulation(prim_path=cabinet_prim_path, position=np.array([0.8, 0.0, 0.4]))

self.franka = FrankaOpenDrawerPolicy(
    prim_path="/World/franka", name="franka",
    position=np.array([0, 0, 0]),
    cabinet=self.cabinet               # 캐비닛 참조 전달
)
```

**physics_dt = 1/400**: 매니퓰레이션은 보행(1/500, 1/200)과는 다른 물리 주파수를 사용합니다. 서랍 핸들과의 접촉 역학이 중요하므로 높은 주파수가 필요하지만, 보행만큼 빠른 다리 동역학은 없습니다.

**캐비닛 = Articulation**: 서랍은 단순 rigid body가 아닌 `SingleArticulation`으로 로드됩니다. 서랍의 슬라이드 관절이 articulation DOF로 표현되어, 정책이 서랍의 상태(열림 정도)를 관찰하고 제어할 수 있습니다.

### 자동 리셋

```python
def on_physics_step(self, step_size):
    if self.get_world().current_time >= 10.0:
        self.get_world().reset()
        return
    self.franka.forward(step_size)
```

10초마다 자동 리셋하여 정책의 성공/실패를 반복 관찰할 수 있습니다.

### Action 구조 (9D)

Franka Panda는 7 DOF 관절 + 2 그리퍼 핑거로 총 9 DOF를 가집니다. 정책은 `_previous_action = np.zeros(9)`에서 볼 수 있듯이 9차원 action을 출력합니다. 그리퍼 제어(열기/닫기)가 정책 출력에 포함되어, 핸들을 잡는 타이밍도 학습된 결과입니다.

## 실행 방법

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive

# GUI: Isaac Examples > Policy > Franka
# Load → Play → 10초 주기로 자동 반복
```

## Comparison: Follow Target vs Open Drawer

| 항목 | Follow Target (`01`) | Open Drawer (이 문서) |
|---|---|---|
| 제어 방식 | RMPflow (모션 플래닝) | 학습된 RL 정책 |
| 목표 입력 | Task-space (EE position) | 없음 (정책이 자율 결정) |
| 환경 상호작용 | 없음 (추종만) | 서랍 핸들 파지 + 당기기 |
| 물리 주파수 | Default (1/60) | 400Hz |
| 리셋 | 수동 | 10초 자동 |

## Further Reading

- **FrankaOpenDrawerPolicy**: `~/workspace/IsaacSim/exts/isaacsim.robot.policy.examples/.../robots/franka.py`
- **FrankaExample**: `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/.../franka/franka_example.py`
