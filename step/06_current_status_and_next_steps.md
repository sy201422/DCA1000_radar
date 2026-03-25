# 06. Current Status And Next Steps

현재 상태 평가:
- `IWR6843ISK-ODS + DCA1000` 실시간 입력은 동작한다.
- `3TX / 4RX / 12 virtual antenna` 처리도 동작한다.
- `RDI / RAI` 기반 후보 검출은 동작한다.
- `adaptive DBSCAN`으로 같은 사람 주변 후보를 줄이는 효과가 있다.
- `Kalman + Hungarian` 이후 lead target 유지도 초반보다 좋아졌다.
- 거리 측정 안정성은 이전보다 더 좋아졌다고 볼 수 있다.

아직 남아 있는 문제:
- track가 완전히 없어지는 것보다 `잠깐 숨는` 경우
- 낮은 confidence의 tentative track가 오래 남는 경우
- 원거리에서 생기는 multipath성 후보

왜 이런 현상이 남는가:
- 실내 환경의 clutter와 multipath는 완전히 사라지지 않는다.
- ODS의 넓은 시야각 때문에 angle spread와 근거리 leakage가 잘 보인다.
- DBSCAN과 Kalman이 많이 줄여주지만, `false candidate 0개`는 현실적으로 어렵다.

현재 권장 다음 단계:
- `association_gate` 미세 조정
  재매칭 실패를 조금 줄이기
- `confirmed -> LOST` 전환 지연
  한 프레임 mismatch에 덜 민감하게 만들기
- tentative track pruning 강화
  낮은 confidence에서 오래 남는 tentative track 정리
- 로그 분석 스크립트 추가
  세션별 `lead continuity`, `drop rate`, `false tentative rate` 자동 요약

중장기 다음 단계:
- 진짜 point cloud 기반 multi-person tracking
- elevation 또는 pseudo-height가 아닌 실제 3D 추정
- 사람 단위 ID 유지 최적화
- 오프라인 재생 데이터셋 기반 파라미터 튜닝 자동화
