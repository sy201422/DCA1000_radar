# 05. 세션 로그와 3D 시각화

## 로그를 넣은 이유

처음에는 화면만 보고 “좋아졌다 / 안 좋아졌다”를 판단했는데, 이 방식으로는 왜 끊기는지 원인을 구분하기 어려웠습니다.

그래서 세션 단위 로그를 저장하도록 바꿨습니다.

## 로그 저장 구조

실행할 때마다 아래 폴더가 생성됩니다.

- `logs/live_motion_viewer/<session_timestamp>/`

주요 파일:
- `runtime_config.json`
- `status_log.jsonl`

기록되는 정보 예시:
- frame index
- 상태바 문자열
- candidate 수
- display track 수
- tentative track 수
- detection 목록
- display / tentative track 상세

이 로그 덕분에 이후에는:
- continuity가 어디서 끊기는지
- ghost 후보가 얼마나 자주 뜨는지
- invalid frame과 display drop이 어떻게 연결되는지

를 정량적으로 볼 수 있게 되었습니다.

## 3D Spatial View 추가

상단에 `3D Spatial View`를 추가해서, 현재 track를 x-y 공간 위에 보기 쉽게 표시했습니다.

주의:
- 이것은 “진짜 elevation 기반 3D 추적”이 아닙니다
- 현재 z축은 confidence를 보기 좋게 띄운 pseudo-height입니다

즉, 이 뷰는 사용자에게 공간감을 주는 시각화 도구이고, 실제 추정 결과는 여전히 x-y 중심입니다.

## 이 단계의 의미

이 단계부터는 단순 demo가 아니라,

- “어떤 세션에서”
- “어떤 파라미터로”
- “어떤 증상이”
- “얼마나 자주 나왔는지”

를 분석 가능한 구조로 넘어갔다고 볼 수 있습니다.
