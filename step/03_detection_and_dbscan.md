# 03. 후보 검출과 Adaptive DBSCAN

## 왜 필요한가

RDI/RAI만 보면 사람 주변에 에너지가 퍼져 보이고, 근거리 clutter나 multipath도 같이 보입니다.  
즉 “그림은 보이지만, 사람 좌표를 바로 쓰기엔 너무 거친 상태”였습니다.

그래서 이 단계에서는 사람 후보를 점 형태로 줄이기 위해 검출과 clustering을 추가했습니다.

## 주요 파일

- `tools/detection.py`
- `tools/dbscan_cluster.py`

## 검출 흐름

현재 검출은 대략 아래 순서입니다.

1. `RDI`에서 zero-Doppler 주변 제거
2. row median 기반 배경 억제
3. power map 생성
4. 2D CFAR-like threshold 계산
5. local maxima 추출
6. `RAI`에서 angle profile을 읽어 각도 후보 선택
7. ROI 밖 angle 제거

이 단계 결과는 `DetectionCandidate` 목록입니다.

## Adaptive DBSCAN을 넣은 이유

한 사람 주변에 가까운 peak가 여러 개 생기다 보니, raw detection만 쓰면 같은 사람이 여러 후보로 뜨는 문제가 컸습니다.

그래서 DBSCAN 계열 병합을 추가했습니다.

기본 아이디어:
- 가까운 거리에서는 작은 `eps`
- 먼 거리에서는 조금 큰 `eps`
- 같은 사람 주변 점은 묶고, 멀리 떨어진 사람은 분리

현재는 거리 band에 따라 `eps`를 다르게 쓰는 adaptive 방식입니다.

## 이 단계의 효과

이 단계 이후에는:
- 한 사람 주변의 중복 후보가 줄어듦
- 빨간 점이 예전보다 사람 단위에 가깝게 정리됨
- 이후 Kalman tracker가 덜 흔들리게 됨

## 남아 있던 한계

이 단계만으로는 해결되지 않는 것:
- 각도 peak가 흔들리면 후보 좌표도 흔들림
- multipath가 클 경우 여전히 다른 위치에 점이 생길 수 있음
- 같은 프레임에서는 정리돼도, 프레임 간 ID 유지 문제는 따로 남음

즉 DBSCAN은 “공간적으로 같은 사람 후보를 줄이는 역할”이고, 시간축 안정화는 다음 단계의 tracker가 담당합니다.
