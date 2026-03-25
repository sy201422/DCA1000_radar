# 08. 장비 제어 검증과 Packet Health 개선

## 왜 이 단계가 필요했는가

Frame 메타데이터를 넣고 보니 실제 세션에서 invalid frame 비율이 꽤 높게 드러났습니다.  
즉, 이제는 “정말 detection 문제인가?”보다 “capture/control이 제대로 되는가?”를 먼저 봐야 하는 단계가 되었습니다.

## 이 단계에서 한 일

### 1. DCA1000 응답 검증

이전에는 DCA command를 보내고 응답을 거의 확인하지 않았습니다.  
이제는 아래를 확인합니다.

- response header / footer
- command code 일치 여부
- status 값

즉, DCA 설정이 실패했는데도 조용히 넘어가는 상황을 줄였습니다.

### 2. Radar CLI 응답 확인

Radar CLI 쪽도 명령을 보낸 뒤 응답을 읽고, 명백한 error/fail 문자열이 있으면 예외로 처리하도록 바꿨습니다.

### 3. packet delay 조정

packet loss를 줄이기 위해 DCA packet delay를 코드에서 조절 가능하게 하고, 기본값을 높였습니다.

추가한 값:
- `DCA_PACKET_SIZE_BYTES`
- `DCA_PACKET_DELAY_US`

### 4. strongest fallback 축소

기존에는 유효한 peak가 하나도 없어도 strongest cell을 억지로 후보로 만드는 흐름이 있었습니다.  
이건 ghost target를 키우는 방향이라 기본적으로 끄고, 필요한 경우 demo 용도로만 켜도록 바꿨습니다.

## 이 단계의 효과

이후 로그에서는:
- invalid frame 비율이 실제로 얼마나 나오는지
- packet delay를 높였을 때 개선되는지
- control 단계에서 조용히 실패하는 문제가 없는지

를 확인할 수 있게 되었습니다.

즉, 이 단계는 “검출기 튜닝”보다 “입력과 제어가 정말 믿을 만한가”를 보강한 단계입니다.
