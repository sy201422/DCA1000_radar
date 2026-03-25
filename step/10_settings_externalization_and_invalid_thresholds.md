# 10. 설정 외부화와 Invalid Threshold 세분화

## 왜 필요한가

이전까지는 COM 포트, IP, ROI, tracking 파라미터, packet delay 같은 값이 코드 상수에 많이 들어 있었습니다.

이 구조의 문제는 다음과 같았습니다.

- 장소가 바뀔 때마다 코드를 직접 수정해야 한다.
- 다른 사람이 프로젝트를 받아 실행할 때 세팅 진입 장벽이 높다.
- 근거리, 원거리, 다인원 상황마다 파라미터를 바꾸기 번거롭다.

또한 invalid frame 처리도 한 가지 방식으로만 묶여 있어서, 경미한 packet 이상과 심각한 frame 훼손을 구분하기 어려웠습니다.

## 이번 단계에서 한 일

### 1. JSON 설정 파일 추가

추가한 파일:

- `config/live_motion_settings.json`

이 파일에 아래 항목을 모았습니다.

- cfg 경로
- CLI 포트와 baudrate
- Host / FPGA IP와 port
- buffer size
- static clutter 설정
- ROI
- detection 관련 값
- tracking 관련 값
- pipeline queue size
- DCA packet delay
- invalid frame 처리 threshold
- 3D view 관련 값

이제 실행 환경이 바뀌어도 코드보다 설정 파일을 먼저 조정하는 방식으로 운용할 수 있습니다.

### 2. runtime_settings 로더 추가

추가한 파일:

- `tools/runtime_settings.py`

역할:

- 기본 설정 제공
- `live_motion_settings.json`이 있으면 override
- 상대 경로를 프로젝트 루트 기준으로 해석

### 3. invalid frame 정책 세분화

이제 invalid frame을 아래 세 단계로 나눠서 처리합니다.

- `full`
  정상 처리
- `no_birth`
  기존 track update는 허용하지만 새 track birth는 막음
- `drop`
  tracking 입력 자체에서 제외

즉, 조금 깨진 프레임은 continuity 유지에 최대한 활용하고, 심하게 깨진 프레임만 강하게 차단하는 방향입니다.

## 현재 기본 threshold

현재 `config/live_motion_settings.json`의 기본값은 다음과 같습니다.

- birth 차단:
  - `gap >= 16`
  - `out_of_sequence >= 2`
  - `byte_mismatch >= 2`
- tracker 입력 drop:
  - `gap >= 140`
  - `out_of_sequence >= 6`
  - `byte_mismatch >= 6`

이 값은 이전보다 더 완화된 기준입니다.

의도는 다음과 같습니다.

- 1.5m 이상 원거리에서 약한 타깃이 잠깐 끊겨도 너무 빨리 새 birth를 막지 않기
- 다인원 상황에서 secondary track가 invalid frame 때문에 너무 쉽게 사라지지 않게 하기
- 구조 안정성은 유지하되, 지나치게 보수적인 억제를 줄이기

## 화면에서 보이는 변화

status bar에서는 invalid frame일 때 다음과 같은 문구가 보일 수 있습니다.

- `births=off`
- `tracker=drop`

의미:

- `births=off`
  continuity는 유지하되, 그 프레임에서 새 타깃은 만들지 않음
- `tracker=drop`
  해당 프레임은 tracking 입력으로 사용하지 않음

## 기대 효과

이번 단계 이후에는 다음 변화를 기대할 수 있습니다.

- invalid 비율이 조금 남아 있어도 track가 너무 빨리 사라지지 않음
- 2명 이상 상황에서 동시에 살아남는 track 수가 조금 더 늘어날 수 있음
- 장소가 바뀌어도 코드 수정 없이 JSON 설정만 조정해 실험 가능

## 다음 확인 포인트

이 설정을 적용한 뒤에는 아래 항목을 로그로 다시 확인합니다.

- `invalid_rate`
- `display_track_count`
- `lead_switches`
- `display >= 2` 프레임 비율
- `births=off`, `tracker=drop` 비율

즉, 이번 단계의 목표는 "완전히 더 공격적으로 검출"이 아니라, "지나치게 보수적인 invalid 정책을 완화해서 실제 타깃 유지력을 높이는 것"입니다.
