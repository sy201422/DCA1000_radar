# 03. Detection And DBSCAN

문제:
- RDI/RAI만 보면 사람 하나가 여러 bright spot로 퍼져 보였다.
- 근거리 반사, multipath, angle spread 때문에 후보가 너무 많이 잡혔다.

주요 변경 파일:
- `tools/detection.py`
- `tools/dbscan_cluster.py`
- `real-time/live_motion_viewer.py`

적용 사항:
- `RDI` 기반 후보 검출 추가
  local maxima + CFAR-like threshold + Doppler guard
- `RAI`에서 angle peak를 붙여 `(range, angle, x, y)` 후보 생성
- angle contrast 조건 추가
  너무 평평한 angle response는 후보에서 제외
- 한 프레임 안의 후보를 adaptive DBSCAN으로 군집화
  같은 사람 주변의 중복 후보를 사람 단위로 합침

adaptive DBSCAN band:
- `0.25m ~ 1.0m`: `eps = 0.34`
- `1.0m ~ 2.0m`: `eps = 0.44`
- `2.0m ~ 3.5m`: `eps = 0.56`

왜 adaptive로 했는가:
- 가까운 거리에서는 점들이 조밀하게 몰려서 작은 `eps`가 유리
- 먼 거리에서는 각도 오차와 spread가 커져서 더 큰 `eps`가 필요

추가 메모:
- 현재 DBSCAN은 dense point cloud용 대형 구현이 아니라, sparse detection 후보 수에 맞춘 경량 버전이다.
- `scikit-learn` 없이도 동작하도록 pure Python으로 넣었다.

이 단계 결과:
- 사람이 하나일 때 주변 빨간 점이 이전보다 확실히 줄었다.
- 근거리와 중거리에서 같은 사람 주변 후보가 더 잘 합쳐졌다.
