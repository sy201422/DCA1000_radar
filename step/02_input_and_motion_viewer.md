# 02. Input And Motion Viewer

주요 목표:
- `profile_3d.cfg` 기준으로 `3TX / 4RX` raw ADC를 실시간으로 받아 `12 virtual antenna` radar cube로 변환
- 정적 clutter를 줄인 뒤 `Moving RDI / Moving RAI`를 표시

주요 변경 파일:
- `tools/radar_runtime.py`
- `tools/real_time_process.py`
- `tools/radar_config.py`
- `real-time/live_motion_viewer.py`
- `config/profile_3d.cfg`

적용 사항:
- cfg 파서 추가
  `adc sample`, `chirp loops`, `tx/rx`, FFT 크기 자동 파싱
- `frame -> radar cube` 변환 일반화
  `3TX / 4RX` 입력을 `12 virtual antenna` 구조로 재배열
- static clutter suppression 추가
  moving target 위주로 보기 쉽게 평균 성분 제거
- `profile_3d.cfg`를 실시간 뷰어 기본 cfg로 연결
- `lvdsStreamCfg -1 0 1 0`로 수정
  실제 UDP raw ADC 스트리밍이 들어오도록 조정
- `%` 주석과 빈 줄은 레이더로 보내지 않게 수정

실행 검증:
- `Received first UDP packet`
- `Received first complete radar frame`
- `Generated first processed RDI/RAI frame`

ROI 기본 설정:
- 좌우: `1.5m`
- 전방: `3.0m`
- 최소 전방 거리 컷: `0.25m`

이 단계 결과:
- 하드웨어 입력과 실시간 시각화는 목표 보드 기준으로 동작
- 사람 움직임이 에너지 맵 수준에서 확인 가능
