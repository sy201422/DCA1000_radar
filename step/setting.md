# 설정 가이드

이 문서는 다른 사람이 프로젝트를 처음 받아 실행할 때 필요한 설치 항목과 환경 설정을 정리한 문서입니다.

기준 장비:

- `IWR6843ISK-ODS`
- `DCA1000`
- Windows PC
- 실행 파일: `real-time/live_motion_viewer.py`

## 1. 권장 실행 환경

권장 환경:

- 운영체제: Windows 10 / 11
- Python: `3.12`
- 실행 방식: 별도 `venv` 가상환경

참고:

- `PyQt5`, `pyqtgraph`, `PyOpenGL` 조합을 사용합니다.
- 상단 3D Spatial View까지 안정적으로 보려면 `Python 3.12` 가상환경이 가장 무난합니다.

## 2. Python 패키지 설치

프로젝트 루트에서 아래 순서로 설치하면 됩니다.

### PowerShell

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install numpy pyqtgraph PyQt5 PyQt5-sip pyserial PyOpenGL PyOpenGL_accelerate
```

### Bash

```bash
py -3.12 -m venv .venv
source .venv/Scripts/activate
python -m pip install --upgrade pip
pip install numpy pyqtgraph PyQt5 PyQt5-sip pyserial PyOpenGL PyOpenGL_accelerate
```

설명:

- `serial`은 별도 패키지가 아니라 `pyserial` 설치 후 `import serial`로 사용합니다.
- `PyOpenGL`, `PyOpenGL_accelerate`는 3D view용입니다.

## 3. 하드웨어 / 네트워크 준비

필수 연결:

- 레이더 USB
- DCA1000 USB
- DCA1000 <-> PC 이더넷

기본 네트워크 값:

- Host PC IP: `192.168.33.30`
- DCA1000 IP: `192.168.33.180`
- DCA config port: `4096`
- ADC data port: `4098`

확인할 것:

- Windows 방화벽이 UDP 통신을 막지 않는지
- 유선 NIC IP가 `192.168.33.30`인지
- DCA1000과 PC가 같은 대역인지

## 4. 먼저 수정할 설정 파일

현재는 [live_motion_settings.json](C:\Users\sy201\U\4-1\C\real-time-radar-master\config\live_motion_settings.json)에서 주요 실행 설정을 관리합니다.

먼저 확인할 항목:

- `config_path`
- `cli_port`
- `cli_baudrate`
- `network.host_ip`
- `network.fpga_ip`
- `network.data_port`
- `network.config_port`
- `dca.packet_delay_us`

현재 기본 예시:

- cfg: `config/profile_3d.cfg`
- CLI 포트: `COM5`
- Host IP: `192.168.33.30`
- FPGA IP: `192.168.33.180`
- DCA packet delay: `100 us`

즉 다른 PC에서 실행할 때는 코드보다 먼저 JSON 설정 파일을 보는 편이 좋습니다.

## 5. tracking 관련 주요 설정

최근 버전에서는 추적 안정화와 다인원 유지력을 위해 tracking 설정도 JSON으로 조정합니다.

대표 항목:

- `confirm_hits`
- `max_misses`
- `association_gate`
- `range_measurement_scale`
- `confidence_measurement_scale`
- `doppler_zero_guard_bins`
- `doppler_gate_bins`
- `doppler_cost_weight`

설명:

- `doppler_zero_guard_bins`
  0속도 근처에서는 속도 부호 변동에 너무 민감하지 않게 하기 위한 완충 구간
- `doppler_gate_bins`
  기존 track와 detection의 Doppler bin 차이를 어디까지 허용할지 정하는 값
- `doppler_cost_weight`
  Doppler 불일치를 association 비용에 얼마나 반영할지 정하는 값

추가로 시각화 관련 설정:

- `visualization.show_tentative_tracks`
- `visualization.tentative_min_confidence`
- `visualization.tentative_min_hits`

의미:

- `show_tentative_tracks`
  confirmed가 되기 전의 강한 tentative track를 보조 표시할지 결정
- `tentative_min_confidence`
  어느 정도 강한 tentative만 화면에 보일지 정하는 기준
- `tentative_min_hits`
  몇 번 이상 누적된 tentative만 보일지 정하는 기준

## 6. 현재 cfg 기준

현재 실시간 뷰어는 [profile_3d.cfg](C:\Users\sy201\U\4-1\C\real-time-radar-master\config\profile_3d.cfg)를 사용합니다.

현재 전제:

- `3TX / 4RX`
- `12 virtual antennas`
- `lvdsStreamCfg -1 0 1 0`

주의:

- cfg 주석보다 실제 명령 줄을 기준으로 봐야 합니다.
- cfg가 바뀌면 sample 수, chirp 수, frame 길이, FFT 크기가 같이 바뀝니다.

## 7. 실행 방법

프로젝트 루트에서:

```bash
python -u real-time/live_motion_viewer.py
```

정상 시작 시 기대 로그:

```text
Runtime config: ...
Create socket successfully
Now start data streaming
```

GUI가 뜬 뒤:

1. `Send Radar Conf` 버튼 클릭
2. cfg가 레이더에 전송
3. UDP 수신이 시작되면 RDI / RAI / 3D view가 갱신

## 8. 정상 동작 확인 포인트

체크리스트:

- 창이 바로 꺼지지 않는지
- `Received first UDP packet ...`가 뜨는지
- `Received first complete radar frame`가 뜨는지
- `Generated first processed RDI/RAI frame`가 뜨는지
- 상단 3D view와 하단 RDI / RAI가 표시되는지

## 9. 로그 저장 위치

세션 로그는 아래 위치에 저장됩니다.

- `logs/live_motion_viewer/<session_timestamp>/`

주요 파일:

- `runtime_config.json`
- `status_log.jsonl`

최근 버전에서는 아래 항목이 함께 기록됩니다.

- `capture_ts`
- `processed_ts`
- `render_ts`
- `capture_to_process_ms`
- `capture_to_render_ms`
- `udp_gap_count`
- `byte_mismatch_count`
- `out_of_sequence_count`
- `invalid`

## 10. 자주 막히는 문제

### 1) `WinError 10048`

원인:

- 이전 실행이 UDP 포트를 이미 사용 중인 상태

대응:

- 이전 실행 창 종료
- 필요하면 PID를 종료 후 다시 실행

### 2) 창은 뜨는데 화면이 검정

확인할 것:

- `Send Radar Conf` 버튼을 눌렀는지
- NIC IP가 맞는지
- COM 포트가 맞는지
- `lvdsStreamCfg`가 맞는지

### 3) 3D view가 안 뜸

원인:

- OpenGL 패키지 미설치
- Python 환경 불일치

권장 해결:

- `Python 3.12 venv`를 새로 만들고 다시 설치

### 4) 장소를 옮기면 노이즈가 증가

가능한 원인:

- 벽 / 유리 / 금속 반사
- 바닥 반사
- 센서 앞 0.3~0.5m clutter
- 센서 높이 / 기울기 변화

## 11. 공유할 때 같이 전달하면 좋은 정보

다른 사람이 재현하기 쉽게 하려면 아래 정보를 함께 전달하는 것이 좋습니다.

- 사용 레이더 모델
- 사용하는 cfg 파일
- CLI 포트 번호
- PC NIC IP
- 센서 설치 높이
- 센서 pitch 각도
- 테스트 공간 설명
- 정상 실행 로그 세션 1개
