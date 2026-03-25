# Step 문서 모음

이 폴더는 `IWR6843ISK-ODS + DCA1000` 기반 실시간 추적 프로젝트를 단계별로 정리한 문서 모음입니다.

현재 프로젝트 요약:

- 초기의 단순 `RDI / RAI` 시각화 데모를 실시간 추적 PoC로 확장했습니다.
- `3TX / 4RX / 12 virtual antenna` 기반 입력 파이프라인이 동작합니다.
- 후보 검출, adaptive DBSCAN, Kalman/Hungarian 추적, 세션 로그가 연결되어 있습니다.
- 최근에는 packet/frame 무결성, timebase, invalid frame 정책, 설정 외부화 중심으로 안정화 작업을 진행하고 있습니다.

문서 목록:

- `01_baseline_to_target.md`
  기존 데모 코드와 현재 목표의 차이
- `02_input_and_motion_viewer.md`
  실시간 입력 파이프라인과 motion viewer 구성
- `03_detection_and_dbscan.md`
  후보 검출과 adaptive DBSCAN 적용
- `04_kalman_and_stability.md`
  Kalman tracker와 안정성 튜닝
- `05_logging_and_visualization.md`
  세션 로그와 3D 시각화 추가
- `06_current_status_and_next_steps.md`
  중간 평가와 다음 작업 방향
- `07_frame_integrity_and_timebase.md`
  packet header 파싱, frame 메타데이터, capture timebase 정리
- `08_capture_control_and_packet_health.md`
  DCA/CLI 응답 검증, packet delay 조정, fallback 축소
- `09_tracking_pipeline_and_invalid_policy.md`
  tracking을 processing 파이프라인으로 옮기고 invalid frame 정책 추가
- `10_settings_externalization_and_invalid_thresholds.md`
  설정을 JSON으로 분리하고 invalid frame 정책을 경미/심각 단계로 나눔
- `11_doppler_consistency_and_association.md`
  Doppler 일관성을 association 비용에 반영해 다인원과 원거리 안정성 개선
- `12_multitarget_display_and_tentative_overlay.md`
  강한 tentative track를 보조 표시해 다인원 상황을 더 빨리 확인하도록 개선
- `setting.md`
  다른 사람이 실행할 때 필요한 설치와 환경 설정 가이드

주 실행 파일:

- `real-time/live_motion_viewer.py`

기본 cfg 파일:

- `config/profile_3d.cfg`

로그 저장 위치:

- `logs/live_motion_viewer/<session_timestamp>/`

실행 명령:

```bash
python -u real-time/live_motion_viewer.py
```
