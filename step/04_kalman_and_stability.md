# 04. Kalman And Stability

문제:
- DBSCAN 뒤에도 원거리 false candidate와 track 끊김이 남았다.
- 단순 nearest-neighbor tracker만으로는 여러 사람과 일시적 미검출에 약했다.

주요 변경 파일:
- `tools/tracking.py`
- `real-time/live_motion_viewer.py`

적용 사항:
- 기존 단순 tracker를 `Kalman + Hungarian assignment` 기반 tracker로 교체
- 상태 관리 추가
  `TENTATIVE`, `CONFIRMED`, `LOST`
- prediction/update 구조 추가
  잠깐 detection이 비어도 track를 이어가도록 함
- distance만 보지 않고 measurement 신뢰도에 따라 필터 반응 조정

현재 주요 tracking 파라미터:
- `track_confirm_hits = 3`
- `track_max_misses = 4`
- `track_process_var = 1.0`
- `track_measurement_var = 0.43`
- `track_range_measurement_scale = 0.50`
- `track_confidence_measurement_scale = 0.35`
- `track_association_gate = 5.99`
- `track_report_miss_tolerance = 1`

range/confidence measurement weighting 의미:
- 멀수록 측정 오차가 크다고 보고 덜 믿음
- score/confidence가 높을수록 더 빨리 따라감
- 약하고 먼 후보는 더 보수적으로 반영

로그 분석으로 본 현재 끊김 원인:
- detection이 아예 없는 프레임보다
  detection은 있는데 display track이 잠깐 0이 되는 경우가 더 많았다
- 즉 끊김의 큰 원인은
  `필터가 없음`보다 `association / state transition / display cutoff`였다

실제 확인된 패턴:
- `display_track_count == 0` 프레임이 존재하지만 그 대부분에서 candidate는 계속 있었다
- tentative인데 hits가 많이 쌓인 track도 여러 번 보였다
- 이는 track가 완전히 사라진 게 아니라, 상태 전환과 표시 조건이 너무 엄격하다는 뜻이다

현재 완화한 부분:
- 화면 표시 조건을 `misses == 0`에서 `misses <= report_miss_tolerance`로 완화

현재 평가:
- 거리 추정 자체는 초반보다 좋아졌다
- 아직 남은 문제는 track continuity와 false tentative track 관리다
