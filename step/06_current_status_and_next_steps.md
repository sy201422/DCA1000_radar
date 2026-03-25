# 06. 당시 상태 평가와 다음 방향

## 당시 중간 평가

여기까지 진행했을 때 얻은 결론은 아래와 같았습니다.

좋아진 점:
- `IWR6843ISK-ODS + DCA1000` 실시간 입력이 실제로 동작
- `3TX / 4RX / 12 virtual antenna` 처리 연결
- RDI / RAI 기반 후보 검출 동작
- DBSCAN으로 같은 사람 주변 후보를 어느 정도 줄임
- Kalman 추적으로 단순 표시보다 continuity가 개선

아직 남아 있던 문제:
- track가 갑자기 사라지거나 숨는 현상
- tentative track가 길게 남는 현상
- 장소가 바뀌면 multipath와 clutter가 크게 달라지는 문제
- 다중 인원에서는 후보는 나오지만 display track가 적게 유지되는 문제

## 왜 이후 방향이 바뀌었는가

초기에는 detection / DBSCAN / Kalman 파라미터를 더 만지는 쪽으로 가려고 했지만,  
로그를 보기 시작하면서 “문제의 핵심이 알고리즘보다 파이프라인 무결성에 있다”는 점이 더 분명해졌습니다.

대표 징후:
- candidate는 있는데 display가 0인 프레임이 많음
- UI timing이 tracker에 섞임
- UDP packet 상태를 전혀 모른 채 추적만 튜닝하고 있었음

그래서 이후 단계에서는:
- packet header 파싱
- frame metadata 추가
- capture timebase 사용
- invalid frame 정책

같은 구조 쪽 작업을 우선하기로 했습니다.
