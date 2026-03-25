# 07. Frame 무결성과 Timebase 정리

## 왜 이 단계가 필요했는가

기존 구조에서는 UDP data의 앞 10바이트를 그냥 버리고 payload만 이어붙이고 있었습니다.  
이 상태에서는 packet loss, out-of-sequence, byte mismatch가 있어도 알 방법이 없었습니다.

또 tracker는 한동안 UI 갱신 시각에 가까운 타이밍을 사용하고 있어서,  
실제 capture timing과 tracking timing이 섞일 위험이 있었습니다.

## 이 단계에서 바꾼 것

### 1. DCA1000 header 파싱

이제 packet header를 읽어서 frame 단위로 아래 메타데이터를 기록합니다.

- `sequence_start`, `sequence_end`
- `byte_count_start`, `byte_count_end`
- `udp_gap_count`
- `byte_mismatch_count`
- `out_of_sequence_count`
- `invalid`
- `invalid_reason`

### 2. FramePacket 구조 도입

이전에는 queue가 분리되어 있어서 RDI, RAI, detection이 서로 느슨하게 연결되어 있었습니다.

이제는 하나의 `FramePacket` 안에:
- 원본 IQ
- capture 시각
- packet health 정보
- 처리 결과

가 같이 들어가도록 바꿨습니다.

### 3. capture timebase 사용

tracker에 넘기는 시간축을 `frame.capture_ts`로 정리하기 시작했습니다.

즉, UI가 느리게 그려도 frame 기준 시간은 따로 유지되는 방향으로 구조를 바꾸었습니다.

### 4. 최신 프레임 위주 렌더링

UI는 이제 queue에 쌓인 오래된 프레임을 모두 그리기보다, 최신 프레임 위주로 가져와서 렌더링합니다.

## 로그에 추가된 항목

이 단계부터 로그에는 아래 정보도 남습니다.

- `capture_ts`
- `assembled_ts`
- `processed_ts`
- `render_ts`
- `capture_to_process_ms`
- `process_to_render_ms`
- `capture_to_render_ms`
- `udp_gap_count`
- `byte_mismatch_count`
- `out_of_sequence_count`
- `invalid`
- `skipped_render_frames`

## 의미

이 단계 이후에는 단순히 “화면이 튄다”가 아니라,

- packet이 깨진 것인지
- processing이 늦은 것인지
- render가 밀린 것인지

를 로그로 분리해서 볼 수 있게 되었습니다.
