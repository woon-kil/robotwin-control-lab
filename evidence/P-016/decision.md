# P-016 decision

조회일: 2026-10-25

**CONDITIONAL GO**

## GO 열 항목

| # | 조건 | 결과 |
|---|---|---|
| 1 | Official Power PMAC Simulator | 만족. 1.2.1, 2026-01-19 |
| 2 | Virtual motor | 만족. release note와 공지 |
| 3 | Motion/PLC software | 만족. release note |
| 4 | RoboTwin이 쓸 external communication path | 불만족. 프로토콜 이름만 있고 세션 측정 없음 |
| 5 | command write | 불만족. 미측정 |
| 6 | state/variable read | 불만족. 미측정 |
| 7 | virtual motor position read | 불만족. `ActPos`는 후보 |
| 8 | `IRobotController` 아래 adapter 설계 | 부분 만족. Protocol은 유지 가능. `step(dt)`와 `reset`은 직접 대응 아님 |
| 9 | Controller-side state/fault logic | 부분 만족. Script/PLC/motion program은 문서상 가능. C는 Partial. 구현 안 함 |
| 10 | 자동 측정 방법 | 부분 만족. 채널이 열리면 round-trip과 status read로 측정 가능. 현재 없음 |

열 개가 모두 만족하지 않으므로 GO가 아니다.

## CONDITIONAL GO인 이유

- Windows 호스트가 필요하다.
- 설치 전 라이선스 동의가 있다. 조건은 미확인.
- SSH, OPC UA, MQTT, DART, HTTP, SFTP가 Simulator 1.2.0 기능으로 적혀 있으나 client 계약은 문서만으로 닫히지 않는다.
- 핵심 구조는 가능하다. virtual CPU, virtual motor, PLC, motion program, 그리고 이름이 있는 외부 프로토콜.

NO-GO로 보지 않은 이유: 외부 client가 불가능하다고 공식 문서가 말하지 않는다. IDE GUI만 가능하다는 제한도 없다. release note가 SSH와 OPC UA를 Simulator 기능으로 적는다.

## P-017

열리더라도 Environment / Communication Spike만 한다.

확인할 것:

1. Windows에서 Simulator 1.2.1이 뜨는지
2. IDE 4.6.5.3이 그 Simulator에 연결되는지
3. Ubuntu 또는 같은 Windows의 외부 client가 SSH 또는 OPC UA로 변수 하나를 쓰고 `Motor[x].ActPos`를 읽는지
4. 그 값이 virtual motor feedback이며 encoder가 아닌지를 기록하는지

adapter 구현, core 수정, 500-cycle 재측정은 P-017 범위가 아니다.

P-017은 이 문서의 REVIEW 전에 시작하지 않는다.
