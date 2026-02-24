# Hello World
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->

## Overview

Isaac Sim의 가장 기본적인 진입점. 두 가지 실행 모드(Standalone / Interactive)에서의 애플리케이션 라이프사이클을 보여줍니다. Standalone 모드에서는 `SimulationApp`의 생성-업데이트-종료 사이클을, Interactive 모드에서는 `BaseSample` 추상 클래스의 템플릿 메서드 패턴과 Extension 등록 메커니즘을 이해할 수 있습니다.

## Architecture

### Standalone 실행 모델

```
SimulationApp(config)
    │
    ├── kit.update()  × N  ← 매 프레임 렌더링 + 물리 스텝
    │
    └── kit.close()        ← 리소스 해제
```

Standalone 스크립트는 `SimulationApp`을 직접 인스턴스화합니다. `config` 딕셔너리로 headless 여부, 해상도 등을 지정하며, `update()` 호출마다 한 프레임이 진행됩니다. 물리 시뮬레이션 없이 순수하게 Kit 앱 라이프사이클만 동작합니다.

### Interactive 실행 모델

```
Extension.on_startup()
    │
    └── get_browser_instance().register_example()
            │
            ├── BaseSampleUITemplate(sample=HelloWorld())
            │       │
            │       └── Load → HelloWorld.load_world_async()
            │                      │
            │                      ├── World(physics_dt=1/60, rendering_dt=1/60)
            │                      ├── setup_scene()        ← 씬 오브젝트 추가
            │                      ├── world.reset_async()
            │                      └── setup_post_load()    ← 콜백/컨트롤러 초기화
            │
            └── Reset → HelloWorld.reset_async()
                            │
                            ├── setup_pre_reset()
                            ├── world.reset_async()
                            └── setup_post_reset()
```

Interactive 모드에서는 Extension 시스템이 예제를 Isaac Sim GUI의 브라우저에 등록합니다. 사용자가 메뉴에서 예제를 선택하면 `BaseSample`의 라이프사이클이 시작됩니다.

## Source Files

| 파일 | 역할 |
|---|---|
| `standalone_examples/api/isaacsim.simulation_app/hello_world.py` | Standalone 최소 예제 (28줄). SimulationApp 라이프사이클만 시연 |
| `exts/isaacsim.examples.interactive/.../hello_world/hello_world.py` | Interactive 예제 (42줄). BaseSample 상속, setup_scene()에서 GroundPlane 추가 |
| `exts/isaacsim.examples.interactive/.../hello_world/hello_world_extension.py` | Extension 등록 (53줄). Examples 브라우저에 UI 템플릿으로 등록 |
| `exts/isaacsim.examples.interactive/.../base_sample/base_sample.py` | 모든 Interactive 예제의 추상 베이스 클래스 (134줄) |

## 핵심 코드 분석

### Standalone: SimulationApp 라이프사이클

```python
from isaacsim import SimulationApp
kit = SimulationApp({"headless": False})

for i in range(100):
    kit.update()       # 1 프레임 = 렌더링 + 물리(있으면)
    print("Hello World!")

kit.close()            # GPU/메모리 리소스 해제
```

`SimulationApp`은 Omniverse Kit 런타임의 Python wrapper다. `config` 딕셔너리의 `"headless": True`로 설정하면 GUI 없이 실행되어 서버 환경에서의 데이터 생성에 활용됩니다. `update()`는 blocking 호출이며, 반환 후 다음 프레임이 렌더링 가능한 상태가 됩니다.

### Interactive: BaseSample 템플릿 메서드

```python
class HelloWorld(BaseSample):
    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
```

`BaseSample`은 Template Method 패턴을 구현합니다. 서브클래스는 `setup_scene()`, `setup_post_load()`, `setup_pre_reset()`, `setup_post_reset()` 중 필요한 메서드만 오버라이드합니다. `load_world_async()`가 전체 초기화 시퀀스를 조율하며, 이 메서드들을 정해진 순서로 호출합니다.

`World` 객체의 기본 설정:
- `physics_dt = 1/60` — 물리 시뮬레이션 타임스텝 (60Hz)
- `rendering_dt = 1/60` — 렌더링 프레임레이트 (60Hz)
- `stage_units_in_meters = 1.0` — USD 스테이지 단위를 미터로 설정

### Extension 등록 패턴

```python
class HelloWorldExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        ui_handle = BaseSampleUITemplate(
            sample=HelloWorld(),
            title="Hello World",
            ...
        )
        get_browser_instance().register_example(
            name="Hello World",
            execute_entrypoint=ui_handle.build_window,
            category="Isaac Examples",
        )
```

Extension의 `on_startup()`은 Isaac Sim이 해당 Extension을 로드할 때 호출됩니다. `get_browser_instance()`는 GUI의 Examples 브라우저 싱글턴을 반환하며, 여기에 등록된 예제는 메뉴에서 선택할 수 있게 됩니다.

## 실행 방법

```bash
# Standalone (headless 가능)
cd ~/workspace/IsaacSim
python standalone_examples/api/isaacsim.simulation_app/hello_world.py

# Interactive (GUI 필요)
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive
# GUI 메뉴: Isaac Examples > Hello World > Load
```

## 핵심 개념

### Standalone vs Interactive

| 항목 | Standalone | Interactive |
|---|---|---|
| 진입점 | `SimulationApp` 직접 생성 | Extension `on_startup()` |
| 물리 제어 | `world.step()` 직접 호출 | `BaseSample`이 관리 |
| UI | 없음 (headless 가능) | Isaac Sim GUI 내장 |
| 용도 | 자동화, 대규모 데이터 생성 | 인터랙티브 시각화, 프로토타이핑 |
| 라이프사이클 | 명시적 (for loop) | 이벤트 기반 (async) |

### USD Stage 단위

Isaac Sim은 USD(Universal Scene Description)를 씬 포맷으로 사용합니다. `stage_units_in_meters=1.0`은 USD 스테이지의 1 단위가 1미터임을 의미합니다. NVIDIA의 일부 에셋은 cm 단위(0.01)로 제작되었으므로, 에셋 로딩 시 단위 불일치에 주의해야 합니다.

## Further Reading

- **BaseSample 소스**: `~/workspace/IsaacSim/exts/isaacsim.examples.interactive/isaacsim/examples/interactive/base_sample/base_sample.py`
- **SimulationApp API**: Isaac Sim Python API 문서
- **Extension 시스템**: [Omniverse Extension 문서](https://docs.omniverse.nvidia.com/extensions/)
