# 05. Logging And Visualization

주요 변경 파일:
- `real-time/live_motion_viewer.py`

상태 로그 저장:
- 실행마다 세션 폴더를 자동 생성
- 위치:
  `logs/live_motion_viewer/<timestamp>/`

세션 폴더 구성:
- `runtime_config.json`
  실행 당시 cfg와 추적 파라미터 기록
- `status_log.jsonl`
  프레임별 상태 기록

`status_log.jsonl`에 저장되는 것:
- 상태바 문자열
- candidate 수
- display track 수
- tentative track 수
- detection 상세 목록
- display track 상세 목록
- tentative track 상세 목록

로그 목적:
- candidate는 있는데 display가 0이 되는 구간 분석
- lead id 유지 시간 분석
- false track 생성 빈도 분석
- 파라미터 튜닝 전후 비교

3D spatial view 추가:
- 기존 `RDI / RAI` 위쪽에 spatial view 영역 추가
- 현재는 true elevation이 아니라 `x / y track + confidence 기반 z` 시각화
- 즉 `실제 3D 복원`이 아니라 `공간적으로 보기 쉬운 3D scene`이다

OpenGL 관련 메모:
- `pyqtgraph.opengl` 사용을 위해 `PyOpenGL`, `PyOpenGL_accelerate` 설치
- bash/anaconda 환경과 system Python 환경이 달라서 user-site 경로를 자동 탐색하도록 수정
- OpenGL import가 실패해도 앱이 완전히 죽지 않고 2D는 뜨도록 fallback 추가
