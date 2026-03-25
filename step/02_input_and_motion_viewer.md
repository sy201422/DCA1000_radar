# 02. 실시간 입력과 Motion Viewer

## 목표

먼저 `IWR6843ISK-ODS + DCA1000` 기준으로 raw ADC를 안정적으로 받아서,  
`Moving Range-Doppler`와 `Moving Range-Angle`를 실시간으로 표시하는 것이 1차 목표였습니다.

## 주요 변경 파일

- `tools/radar_runtime.py`
- `tools/real_time_process.py`
- `tools/radar_config.py`
- `real-time/live_motion_viewer.py`
- `config/profile_3d.cfg`

## 핵심 변경점

### 1. cfg 파싱 일반화

`profile_3d.cfg`를 읽어서 아래 값을 자동으로 계산하도록 바꿨습니다.

- ADC sample 수
- chirp loop 수
- TX / RX 개수
- range / doppler / angle FFT 크기

즉, 예전처럼 코드에 값이 박혀 있지 않고 cfg 기반으로 runtime 설정을 만들게 했습니다.

### 2. 3TX / 4RX 입력 처리

`3TX / 4RX` 설정에 맞춰 `12 virtual antenna` radar cube를 만들도록 바꿨습니다.

이 단계가 중요한 이유:
- 기존 `1TX / 4RX` 처리 흐름을 그대로 쓰면 `IWR6843ISK-ODS` 데이터가 올바르게 재배열되지 않음
- 이후 angle 처리와 추적 결과도 모두 여기 영향을 받음

### 3. static clutter suppression

정적인 벽/가구보다 움직이는 사람을 더 잘 보이게 하려고 평균 성분 제거 기반 static clutter suppression을 넣었습니다.

### 4. ROI 설정

기본 관심 영역은 아래처럼 두었습니다.

- 좌우: `1.5m`
- 전방: `3.0m`
- 최소 전방 컷: `0.25m`

근거리 leakage와 센서 바로 앞 clutter를 줄이는 목적이 큽니다.

## 실행 확인 기준

정상 실행 시 보였던 핵심 로그:

- `Received first UDP packet`
- `Received first complete radar frame`
- `Generated first processed RDI/RAI frame`
- `Displaying first processed frame`

즉, 이 단계에서는 “레이더 입력이 실제로 들어오고, moving target 중심의 RDI/RAI가 실시간으로 뜬다”까지를 확인한 것입니다.
