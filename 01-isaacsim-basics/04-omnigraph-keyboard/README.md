# OmniGraph Keyboard Control

## Overview

OmniGraph를 사용한 선언적 데이터플로우 프로그래밍 패턴을 시연한다. Python 스크립트 대신 노드 그래프로 키보드 입력 → 수학 연산 → Prim 속성 변경 파이프라인을 구성한다. A키로 큐브를 확대하고 D키로 축소하는 간단한 상호작용을 OmniGraph 노드만으로 구현한다.

## Architecture

### OmniGraph 데이터플로우

```
┌─────────────────┐     ┌──────────┐     ┌──────────┐
│ ReadKeyboardState│────▶│ ToDouble │────▶│ Multiply │──┐
│   (A key)       │     │          │     │ (×0.1)   │  │
└─────────────────┘     └──────────┘     └──────────┘  │
                                                        │   ┌─────────┐
                                                        ├──▶│  Add    │──▶ WritePrimAttribute
                                                        │   │         │    (cube xScale)
┌─────────────────┐     ┌──────────┐     ┌──────────┐  │   └─────────┘
│ ReadKeyboardState│────▶│ ToDouble │────▶│ Multiply │──┘        ▲
│   (D key)       │     │          │     │ (×-0.1)  │           │
└─────────────────┘     └──────────┘     └──────────┘    ReadPrimAttribute
                                                         (현재 cube xScale)
```

A키 입력 → `true` → `1.0` → `×0.1` → `+0.1` → 현재 스케일에 가산
D키 입력 → `true` → `1.0` → `×-0.1` → `-0.1` → 현재 스케일에서 감산

## Source Files

| 파일 | 역할 |
|---|---|
| `exts/.../omnigraph_keyboard/omnigraph_keyboard.py` | OmniGraph 구성 (93줄). BaseSample 상속, 노드 생성/연결 |
| `exts/.../omnigraph_keyboard/omnigraph_keyboard_extension.py` | Extension 등록 (61줄). 키보드 바인딩 문서화 |

## 핵심 코드 분석

### OmniGraph 노드 생성

```python
import omni.graph.core as og

keys = og.Controller.Keys

og.Controller.edit(
    {"graph_path": "/World/PushGraph", "evaluator_name": "push"},
    {
        keys.CREATE_NODES: [
            ("ReadKeyA", "omni.graph.nodes.ReadKeyboardState"),
            ("ReadKeyD", "omni.graph.nodes.ReadKeyboardState"),
            ("ToDoubleA", "omni.graph.nodes.ToDouble"),
            ("ToDoubleD", "omni.graph.nodes.ToDouble"),
            ("MultiplyA", "omni.graph.nodes.Multiply"),
            ("MultiplyD", "omni.graph.nodes.Multiply"),
            ("Add", "omni.graph.nodes.Add"),
            ("ReadScale", "omni.graph.nodes.ReadPrimAttribute"),
            ("WriteScale", "omni.graph.nodes.WritePrimAttribute"),
        ],
        keys.SET_VALUES: [
            ("ReadKeyA.inputs:key", "A"),
            ("ReadKeyD.inputs:key", "D"),
            ("MultiplyA.inputs:b", 0.1),
            ("MultiplyD.inputs:b", -0.1),
            ("ReadScale.inputs:primPath", "/World/Cube"),
            ("ReadScale.inputs:name", "xformOp:scale"),
            ("WriteScale.inputs:primPath", "/World/Cube"),
            ("WriteScale.inputs:name", "xformOp:scale"),
        ],
        keys.CONNECT: [
            ("ReadKeyA.outputs:isPressed", "ToDoubleA.inputs:value"),
            ("ToDoubleA.outputs:double", "MultiplyA.inputs:a"),
            ("ReadKeyD.outputs:isPressed", "ToDoubleD.inputs:value"),
            ("ToDoubleD.outputs:double", "MultiplyD.inputs:a"),
            ("MultiplyA.outputs:product", "Add.inputs:a"),
            ("MultiplyD.outputs:product", "Add.inputs:b"),
            ("ReadScale.outputs:value", "Add.inputs:a"),  # 누적
            ("Add.outputs:sum", "WriteScale.inputs:value"),
        ],
    }
)
```

`og.Controller.edit()`은 선언적으로 그래프를 구성한다:
1. `CREATE_NODES`: 노드 인스턴스 생성 (이름, 타입)
2. `SET_VALUES`: 노드의 입력 파라미터 설정
3. `CONNECT`: 노드 간 데이터 연결 (output → input)

`evaluator_name="push"` 는 push evaluation 모드로, 입력이 변경될 때만 downstream 노드가 재평가된다.

### Python 스크립트 vs OmniGraph

| 항목 | Python 스크립트 | OmniGraph |
|---|---|---|
| 제어 흐름 | 명령형 (순차 실행) | 선언적 (데이터플로우) |
| 실행 시점 | 물리 콜백에서 매 스텝 | 그래프 evaluator가 관리 |
| 디버깅 | print / breakpoint | GUI에서 노드별 값 확인 |
| 재사용 | 코드 복사 | 그래프 에셋(.json)으로 저장/로드 |
| 성능 | Python GIL 제약 | C++ 백엔드, 병렬 평가 가능 |

OmniGraph는 반복적인 데이터 파이프라인(센서 처리, 로봇 행동 트리)에 적합하다. 복잡한 분기 로직이 필요한 경우에는 Python 스크립트가 더 직관적이다.

## 실행 방법

```bash
cd ~/workspace/IsaacSim
./isaac-sim.sh --enable isaacsim.examples.interactive
# 메뉴: Isaac Examples > OmniGraph Keyboard
# A 키: 큐브 확대 / D 키: 큐브 축소
```

## 핵심 개념

### OmniGraph Evaluator 모드

- **Push**: 입력 변경 시 downstream으로 전파. 이벤트 기반에 적합.
- **Dirty Push**: Push와 유사하나, 변경되지 않은 노드는 건너뜀. 대규모 그래프에서 성능 최적화.
- **Execution**: 명시적 실행 순서 제어. Action Graph에서 사용.

이 예제는 Push evaluator를 사용하여 키보드 입력이 감지될 때만 그래프가 평가된다.

### Prim Attribute 접근

`ReadPrimAttribute`/`WritePrimAttribute` 노드는 USD Prim의 속성에 직접 접근한다. `xformOp:scale`은 USD의 Transform Operation으로, Prim의 스케일링을 제어한다. 이 방식으로 OmniGraph에서 씬의 모든 속성을 동적으로 읽고 쓸 수 있다.

## Further Reading

- **OmniGraph 문서**: [Omniverse OmniGraph](https://docs.omniverse.nvidia.com/extensions/latest/ext_omnigraph.html)
- **노드 레퍼런스**: Isaac Sim 내 Window > Visual Scripting > Action Graph에서 노드 목록 확인
- **Action Graph**: OmniGraph의 실행 기반 변형으로, 로봇 제어 파이프라인에서 활용
