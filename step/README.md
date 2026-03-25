# Step Notes

이 폴더는 `IWR6843ISK-ODS + DCA1000` 기반 실시간 추적 개발 과정을 단계별로 정리한 문서 모음이다.

현재 판단:
- 거리 측정 안정성은 초반보다 확실히 좋아졌다.
- `DBSCAN + Kalman` 이후 한 사람을 계속 붙잡는 능력은 개선됐다.
- 아직 남은 이슈는 `track continuity`와 `유령 후보/일시적 숨김` 쪽이다.

문서 목록:
- `01_baseline_to_target.md`
  원본 프로젝트의 성격과 목표 시스템 차이
- `02_input_and_motion_viewer.md`
  `3TX / 4RX / 12 virtual antenna` 실시간 파이프라인 구축
- `03_detection_and_dbscan.md`
  후보 검출과 adaptive DBSCAN 적용
- `04_kalman_and_stability.md`
  Kalman tracker, 튜닝, 현재 끊김 원인
- `05_logging_and_visualization.md`
  상태 로그 저장과 3D spatial view 추가
- `06_current_status_and_next_steps.md`
  현재 상태 평가와 다음 권장 작업

핵심 실행 파일:
- `real-time/live_motion_viewer.py`

핵심 설정 파일:
- `config/profile_3d.cfg`

로그 저장 위치:
- `logs/live_motion_viewer/<session_timestamp>/`

실행 코드:
- `python -u real-time/live_motion_viewer.py`