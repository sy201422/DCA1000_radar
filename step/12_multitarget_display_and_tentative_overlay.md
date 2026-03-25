# 12. 다인원 표시 개선과 Tentative Overlay

## 왜 필요한가

최근 로그를 보면 다인원 환경에서 아래 현상이 반복됐습니다.

- `candidate >= 2`, `candidate >= 3`은 자주 발생
- 하지만 최종 `display_track_count`는 대부분 1개에 머무름

즉, 히트맵과 후보 단계에서는 여러 사람이 보이는데, 화면에서는 confirmed track만 표시하다 보니 다인원 상황이 과하게 축소되어 보이는 문제가 있었습니다.

## 이번 단계에서 한 일

### 1. confirmed와 tentative를 분리 표시

이제 화면에는 두 종류의 track가 표시됩니다.

- confirmed track
  - 기존처럼 빨간색 원
- strong tentative track
  - 노란색 원

즉, 아직 confirmed는 아니지만 충분히 강하고 몇 프레임 이상 유지된 tentative track를 보조적으로 볼 수 있게 했습니다.

### 2. tentative 표시 조건 추가

아래 조건을 만족하는 tentative만 보조 표시합니다.

- `misses == 0`
- `hits >= visualization.tentative_min_hits`
- `confidence >= visualization.tentative_min_confidence`

이렇게 해서 잠깐 튄 잡음까지 전부 보여주지 않고, 다인원 후보를 비교적 안정적으로 확인할 수 있게 했습니다.

### 3. 3D view에도 반영

상단 3D Spatial View에도 tentative를 별도 색으로 함께 표시합니다.

- confirmed: 주황/빨강 계열
- tentative: 노랑 계열

## 새 설정 항목

`config/live_motion_settings.json`

- `visualization.show_tentative_tracks`
- `visualization.tentative_min_confidence`
- `visualization.tentative_min_hits`

기본값:

- `show_tentative_tracks = true`
- `tentative_min_confidence = 0.30`
- `tentative_min_hits = 2`

## 기대 효과

이번 단계 이후 기대하는 변화:

- 1명에서 2명으로 바뀔 때 두 번째 사람이 더 빨리 보임
- 2명 이상 상황에서 "보이긴 하는데 화면에는 1명만 남는" 체감이 줄어듦
- tracking을 공격적으로 느슨하게 만들지 않고도 다인원 상황을 더 잘 해석할 수 있음

## 주의할 점

이 단계는 confirmed 조건을 크게 완화한 것이 아닙니다.

즉, ghost를 줄이기 위한 현재 tracking 정책은 유지하면서,
"confirmed 직전의 강한 후보를 보조로 보이게 한 것"에 가깝습니다.

그래서 표시가 조금 늘어나더라도, 그것이 곧 최종 객체 확정이 늘었다는 뜻은 아닙니다.

## 다음 확인 포인트

로그에서 아래 항목을 같이 확인합니다.

- `display_track_count`
- `tentative_display_track_count`
- `candidate >= 2`인데 `display < 2`인 프레임 비율
- 1명에서 2명으로 바뀌는 시점의 후반부 track 수 변화

즉 이번 단계의 목표는 다인원에서 "보이는 정보량"을 늘려 해석성을 높이는 것입니다.
