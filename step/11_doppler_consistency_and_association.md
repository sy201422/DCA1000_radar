# 11. Doppler 일관성과 Association 안정화

## 왜 이 단계가 필요한가

이전까지 tracker는 주로 `x / y` 위치 기반으로만 detection과 기존 track를 연결했습니다.

이 방식은 1인 환경에서는 어느 정도 버티지만, 아래 상황에서 약점이 드러납니다.

- 2명 이상이 동시에 움직일 때
- 서로 비슷한 위치를 지나갈 때
- 원거리에서 각도 오차가 커질 때

즉, 히트맵에서는 여러 사람이 보이는데 최종 track는 1~2개로 줄어드는 병목이 생깁니다.

## 이번 단계에서 한 일

### 1. Doppler 일관성을 association 비용에 반영

이제 tracker는 단순히 위치 Mahalanobis 거리만 보지 않고, detection의 `doppler_bin`과 기존 track의 `doppler_bin` 차이도 함께 봅니다.

핵심 아이디어:

- 위치는 가까워도 속도 방향이 너무 다르면 같은 객체일 가능성을 낮게 본다.
- 0속도 근처는 부호가 쉽게 흔들릴 수 있으므로 완충 구간을 둔다.
- Doppler 차이가 너무 크면 매칭 자체를 거부할 수 있다.

### 2. 새 설정 파라미터 추가

`config/live_motion_settings.json`과 `tools/runtime_settings.py`에 아래 값을 추가했습니다.

- `doppler_zero_guard_bins`
- `doppler_gate_bins`
- `doppler_cost_weight`

의미:

- `doppler_zero_guard_bins`
  0속도 근처에서는 부호 변화에 둔감하게 하기 위한 guard 구간
- `doppler_gate_bins`
  track와 detection 사이 Doppler 차이를 어디까지 허용할지 정하는 기준
- `doppler_cost_weight`
  Doppler 불일치를 association 비용에 얼마나 반영할지 정하는 값

## 현재 기본값

현재 기본값:

- `doppler_zero_guard_bins = 3`
- `doppler_gate_bins = 18`
- `doppler_cost_weight = 0.65`

이 값은 너무 공격적으로 자르지 않으면서, 명백히 다른 속도 패턴은 분리하려는 목적의 보수적 초기값입니다.

## 기대 효과

이번 단계 이후 기대하는 변화:

- 다인원에서 서로 다른 사람을 같은 track로 잘못 합치는 경우 감소
- 원거리에서 detection이 조금 흔들려도 기존 track와 더 일관되게 연결
- candidate는 여러 개 보이는데 display track가 1개로 수축되는 현상 일부 완화

## 주의할 점

이 단계는 detection을 늘리는 작업이 아닙니다.

즉, "더 많은 노란 점을 만들기"보다는
"이미 보이는 후보를 기존 track와 더 레이더답게 연결하기"에 가깝습니다.

또한 Doppler를 너무 강하게 믿으면:

- 아주 느리게 움직이는 사람
- 정지와 이동 사이를 오가는 사람
- 0속도 근처에서 부호가 흔들리는 경우

에서 오히려 매칭이 끊길 수 있습니다.

그래서 현재 값은 완만한 초기 세팅으로 두고, 로그를 보며 조정하는 방식이 적합합니다.

## 다음 확인 포인트

이 버전 이후 로그에서 특히 봐야 할 항목:

- `display_track_count`
- `display >= 2`, `display >= 3` 비율
- `lead_switches`
- `no_display_with_candidates`
- 다인원에서 `candidate >= N`인데 `display < N`인 프레임 비율

즉 이번 단계의 목적은 다인원과 원거리에서 "검출은 보이는데 추적으로 유지되지 않는" 병목을 association 단계에서 줄이는 것입니다.
