# 09. Tracking 파이프라인 이동과 Invalid Frame 정책

## 문제

이전 구조에서는 tracker가 UI 갱신 주기에 영향을 많이 받았습니다.

이 경우 생기는 문제:

- UI가 느려지면 tracking 타이밍도 같이 흔들린다.
- render 지연이 Kalman filter의 시간축에 섞일 수 있다.
- invalid frame이 들어왔을 때 continuity와 ghost birth 사이의 균형을 잡기 어렵다.

## 이번 단계에서 한 일

### 1. tracking을 UI에서 processing 단계로 이동

이제 tracking은 `DataProcessor` 안에서 수행합니다.

구조:

- `UdpListener -> raw_frame_queue`
- `DataProcessor -> detection + tracking`
- `processed_frame_queue -> UI render`

즉 UI는 추적 결과를 계산하는 곳이 아니라, 이미 계산된 결과를 받아 그리는 역할만 합니다.

### 2. FramePacket에 tracking 결과 포함

이제 프레임마다 아래 정보가 함께 전달됩니다.

- `tracker_input_count`
- `confirmed_tracks`
- `tentative_tracks`
- `track_birth_blocked`
- `tracker_policy`

그래서 UI와 로그가 같은 프레임 기준으로 동작할 수 있습니다.

### 3. invalid frame 정책 추가

invalid frame이 들어와도 무조건 버리지 않고, 상태에 따라 다르게 처리합니다.

- `full`
  정상 처리
- `no_birth`
  기존 track update는 허용하지만 새 track birth는 막음
- `drop`
  tracking 입력 자체에서 제외

이 정책의 목적은 다음과 같습니다.

- continuity는 최대한 유지
- packet 이상이 심한 프레임에서 ghost birth는 줄이기

## 왜 이렇게 했는가

invalid frame을 전부 버리면 packet 상태가 조금만 나빠져도 tracker가 쉽게 끊깁니다.

반대로 invalid frame을 전부 정상 프레임처럼 쓰면, 순간적인 packet 이상이 ghost track로 이어질 수 있습니다.

그래서 기존 track 유지와 새로운 track 생성의 기준을 분리했습니다.

## 사용자에게 보이는 변화

status bar에서 아래 문구가 보일 수 있습니다.

- `births=off`
- `tracker=drop`

의미:

- `births=off`
  기존 track은 유지하되, 그 프레임에서는 새 target을 만들지 않음
- `tracker=drop`
  해당 프레임은 tracking 입력에서 제외

## 기대 효과

이번 단계 이후에는 다음 효과를 기대할 수 있습니다.

- UI 지연이 tracking jitter를 만드는 문제가 줄어든다.
- invalid frame에서 ghost birth가 줄어든다.
- continuity와 false birth 사이 균형을 좀 더 안정적으로 맞출 수 있다.
