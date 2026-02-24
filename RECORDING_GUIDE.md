# Demo Recording Guide

각 데모 폴더의 README에는 `assets/demo.gif` 경로로 GIF 플레이스홀더가 포함되어 있습니다.
이 가이드는 각 데모의 동작 화면을 녹화하고 GIF로 변환하는 방법을 설명합니다.

## 녹화 도구

### 방법 1: OBS Studio (권장)

```bash
# 설치
sudo apt install obs-studio

# 설정
# - Sources > Window Capture > Isaac Sim 창 선택
# - Settings > Output > Recording Format: mp4
# - Settings > Video > Output Resolution: 1280x720 (README 임베딩에 적합)
```

### 방법 2: ffmpeg + x11grab (Headless 환경)

```bash
# 특정 창 녹화 (window ID 확인 후)
xdotool search --name "Isaac Sim" | head -1
ffmpeg -video_size 1280x720 -framerate 30 -f x11grab -i :0.0+0,0 -t 10 output.mp4
```

### 방법 3: Isaac Sim 내장 녹화

Isaac Sim GUI 메뉴에서 `Window > Movie Capture`를 사용하면 렌더링 품질이 가장 높습니다.

## MP4 → GIF 변환

```bash
# 기본 변환 (10초, 15fps, 너비 640px)
ffmpeg -i input.mp4 -vf "fps=15,scale=640:-1:flags=lanczos" -t 10 output.gif

# 파일 크기 최적화 (palette 사용, GitHub 권장 용량 10MB 이하)
ffmpeg -i input.mp4 -vf "fps=12,scale=480:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -t 8 output.gif

# 특정 구간 추출 (시작 5초부터 8초간)
ffmpeg -ss 5 -i input.mp4 -vf "fps=15,scale=640:-1:flags=lanczos" -t 8 output.gif
```

## 데모별 녹화 가이드

각 데모에서 어떤 장면을 녹화해야 하는지 정리합니다.
녹화된 GIF는 해당 데모 폴더의 `assets/demo.gif`로 저장합니다.

### 01-isaacsim-basics

| 데모 | 녹화 포인트 | 권장 길이 |
|---|---|---|
| 01-hello-world | 큐브가 추가되고 물리 시뮬레이션이 시작되는 장면 | 5초 |
| 02-getting-started | 큐브/구에 물리가 적용되어 낙하하는 장면 | 5초 |
| 03-robot-loading | Franka 로봇이 로드되고 관절이 움직이는 장면 | 8초 |
| 04-omnigraph-keyboard | 키보드로 큐브를 제어하는 장면 | 8초 |

### 02-isaacsim-manipulation

| 데모 | 녹화 포인트 | 권장 길이 |
|---|---|---|
| 01-franka-follow-target | 타겟 마커를 드래그하고 Franka가 추종하는 장면 | 10초 |
| 02-franka-pick-place | 서랍을 여는 전체 동작 (접근 → 잡기 → 당기기) | 10초 |
| 03-path-planning | RRT 경로가 계산되고 장애물을 회피하며 이동하는 장면 | 10초 |
| 04-franka-cortex | Cortex behavior tree로 블록을 쌓는 장면 | 10초 |
| 05-simple-stack | 4대 Franka가 동시에 블록을 쌓는 장면 | 10초 |
| 06-ur10-palletizing | UR10이 빈에서 나사를 옮기는 장면 + 동적 생성 | 10초 |

### 03-isaacsim-locomotion

| 데모 | 녹화 포인트 | 권장 길이 |
|---|---|---|
| 01-quadruped-spot | Spot이 키보드 명령으로 보행하는 장면 | 10초 |
| 02-humanoid-h1 | H1이 전진 보행하는 장면 | 10초 |
| 03-anymal-standalone | ANYmal이 warehouse에서 보행하는 장면 | 10초 |

### 04-isaacsim-multirobot

| 데모 | 녹화 포인트 | 권장 길이 |
|---|---|---|
| 01-robo-factory | 4대 Franka가 동시에 작업하는 전체 뷰 | 10초 |
| 02-robo-party | 여러 종류 로봇이 함께 움직이는 장면 | 10초 |
| 03-bin-filling | UR10이 나사를 옮기고 새 나사가 동적 생성되는 장면 | 10초 |

### 05-isaacsim-sensors

| 데모 | 녹화 포인트 | 권장 길이 |
|---|---|---|
| 01-contact-sensor | 접촉 센서 값이 실시간 표시되는 장면 | 8초 |
| 02-imu-sensor | IMU 데이터가 출력되며 로봇이 이동하는 장면 | 8초 |
| 03-lidar | LiDAR 포인트 클라우드가 렌더링되는 장면 | 8초 |
| 04-camera | 카메라 뷰와 depth/semantic 출력을 나란히 보여주는 장면 | 8초 |

### 06-isaaclab-classic

| 데모 | 녹화 포인트 | 권장 길이 |
|---|---|---|
| 01-cartpole-direct | 학습 후 CartPole 균형 유지 장면 (play 모드) | 8초 |
| 02-cartpole-manager | 학습 후 CartPole 균형 유지 장면 (play 모드) | 8초 |
| 03-ant | 학습 후 Ant가 전진하는 장면 | 10초 |
| 04-humanoid | 학습 후 Humanoid가 보행하는 장면 | 10초 |

### 07-isaaclab-locomotion

| 데모 | 녹화 포인트 | 권장 길이 |
|---|---|---|
| 01-anymal-c-velocity | ANYmal-C가 rough terrain에서 보행하는 장면 | 10초 |
| 02-h1-locomotion | H1이 보행하는 장면 (flat terrain) | 10초 |
| 03-go2-velocity | Go2가 보행하는 장면 | 10초 |

### 08-isaaclab-manipulation

| 데모 | 녹화 포인트 | 권장 길이 |
|---|---|---|
| 01-franka-reach | Franka가 타겟 위치로 이동하는 장면 | 8초 |
| 02-franka-lift | Franka가 물체를 들어올리는 장면 | 10초 |
| 03-shadow-hand | Shadow Hand가 물체를 회전하는 장면 | 10초 |

## 파일 배치

녹화한 GIF를 각 데모 폴더의 `assets/` 디렉토리에 저장합니다:

```
01-isaacsim-basics/01-hello-world/assets/demo.gif
02-isaacsim-manipulation/01-franka-follow-target/assets/demo.gif
...
```

각 README의 상단에 이미 GIF 참조가 포함되어 있습니다:

```markdown
<!-- TODO: 데모 GIF 추가 — 녹화 가이드는 RECORDING_GUIDE.md 참조 -->
<!-- ![Demo](assets/demo.gif) -->
```

GIF가 준비되면 주석을 해제합니다:

```markdown
![Demo](assets/demo.gif)
```

## 참고

- GitHub에서 README에 임베딩된 GIF는 **10MB 이하**가 권장됩니다. 초과 시 로딩이 느려집니다.
- `scale=480:-1`로 너비를 480px로 제한하면 대부분 10MB 이내로 유지됩니다.
- Isaac Lab 환경은 `--num_envs 1`로 실행해야 단일 로봇이 잘 보입니다. (기본 4096은 화면이 혼잡합니다)
