# 04. Kalman 추적기와 안정성 조정

## 왜 Kalman이 필요했는가

DBSCAN까지 넣어도 후보는 여전히 프레임마다 조금씩 튑니다.  
이 상태로는 사람이 움직일 때 좌표가 들쭉날쭉하고, 잠깐 detection이 약해질 때 바로 사라지는 문제가 생깁니다.

그래서 `Kalman + Hungarian` 기반 다중 추적기를 추가했습니다.

## 주요 파일

- `tools/tracking.py`

## 현재 tracker의 특징

현재 tracker는 단순 nearest-neighbor가 아니라 아래 요소를 갖고 있습니다.

- Kalman filter
- Hungarian assignment
- `TENTATIVE / CONFIRMED / LOST` 상태
- Mahalanobis gating
- range / confidence 기반 measurement covariance 조절

이 덕분에:
- 좌표가 조금 더 부드럽게 이어지고
- 잠깐 detection이 끊겨도 바로 track이 죽지 않으며
- 다중 후보 중 같은 객체를 이어붙일 수 있게 되었습니다

## 튜닝하면서 본 trade-off

### `confirm hits`

너무 낮으면:
- noise가 쉽게 confirmed track가 됨

너무 높으면:
- 실제 사람도 늦게 붙거나 놓칠 수 있음

### measurement 신뢰도

멀수록 angle 오차가 커지고 좌표 흔들림이 커집니다.  
그래서 range와 confidence에 따라 measurement를 덜 믿도록 조정했습니다.

### 근거리 / 원거리 분리 조정

DBSCAN과 tracker를 같이 조정하면서:
- 근거리 후보 중복은 줄이고
- 원거리 candidate는 너무 쉽게 하나로 합쳐지지 않게 조정했습니다

## 당시 남아 있던 문제

이 단계에서 보였던 대표 문제:
- track가 잠깐 숨는 현상
- multipath로 생긴 후보가 tentative로 오래 남는 현상
- 장소가 바뀌면 tuning이 흔들리는 현상

즉 tracker 자체는 붙었지만, 이후에는 detection보다 파이프라인 무결성과 timebase 문제가 더 중요해졌습니다.
