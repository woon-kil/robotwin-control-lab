# P-016 architecture feasibility

조회일: 2026-10-25. 구현하지 않는다.

```text
RoboTwin
   │
RobotCommand
   │
PreExecutionValidator
   │
ExecutionGate
   │
IRobotController
   │
   ├── VirtualPmacController          유지. 수정 없음
   │
   └── PowerPmacControllerAdapter     미작성
                 │
           Communication Layer        SSH 또는 OPC UA 후보. 미측정
                 │
         Power PMAC Simulator 1.2.1
                 │
          PMAC Script / PLC / Motion Program
                 │
           Virtual Motors
                 │
           Controller State
                 │
       PowerPmacControllerAdapter
                 │
            RobotState
                 │
        DigitalTwinMonitor
```

기존 Protocol을 유지한 채 두 번째 구현을 옆에 둘 수 있다. `ExecutionGate`는 계속 `SAFE_TO_ATTEMPT`일 때만 submit한다. Simulator가 validator를 대신하지 않는다.

Digital Twin 경로는 read가 될 때 기존 P-009 구조를 재사용할 수 있다.

```text
VP-PMAC virtual motor
   ↓
adapter가 actual_q로 환산
   ↓
RobotState.actual_q
   ↓
DigitalTwinMonitor
   ↓
FK
   ↓
Target TCP / Actual TCP
   ↓
HMI
```

이 값은 virtual motor feedback이다. physical encoder state가 아니다.

## Cross-controller

| 항목 | 판정 | 이유 |
|---|---|---|
| command acceptance | PARTIALLY MEASURABLE | 채널 spike 전 |
| command rejection | PARTIALLY MEASURABLE | RoboTwin 쪽 거절은 이미 있음. PMAC 쪽 거절 코드는 normalize 필요 |
| state transitions | PARTIALLY MEASURABLE | 이름 매핑 필요 |
| target joints | PARTIALLY MEASURABLE | motor unit scale 필요 |
| virtual/actual joints | PARTIALLY MEASURABLE | `ActPos` read 미측정 |
| execution completion | PARTIALLY MEASURABLE | in-position과 `COMPLETED`는 다름 |
| fault | PARTIALLY MEASURABLE | status bit 후보 |
| reset | PARTIALLY MEASURABLE | joint 유지 조건을 adapter가 지켜야 함 |
| timeout | PARTIALLY MEASURABLE | 통신 timeout은 spike 이후 |
| reconnect | PARTIALLY MEASURABLE | 세션 정책 미확인 |
| deterministic behavior | NOT MEASURABLE | 프로젝트 시계와 Simulator servo를 같은 결정성으로 비교할 근거 없음 |

전체 판정: **PARTIAL**.

## Fault injection

| 항목 | Injectable | Observable | Recoverable | Official mechanism | Project-side simulation required |
|---|---|---|---|---|---|
| invalid command | 후보 | adapter reject | 예 | PMAC 오류 문자열은 미확인 | 현재 VC에 있음 |
| joint target invalid | 후보 | adapter 또는 PMAC limit | 예 | soft limit은 문서상 motor 설정 | scale 전에 RoboTwin limit 유지 |
| controller busy | 후보 | adapter 상태 | 예 | program running bit 후보 | VC의 BUSY와 이름이 다름 |
| duplicate command | 후보 | adapter | 예 | PMAC command id는 없음 | adapter가 id를 지켜야 함 |
| stale command | 후보 | adapter | 예 | 공식 stale 토큰 없음 | project rule |
| command timeout | 통신이면 후보 | 미측정 | 미확인 | 없음 | spike |
| communication disconnect | 소켓을 끊으면 후보 | 미측정 | 미확인 | SSH/OPC 재연결 문서 없음 | spike |
| controller-side fault | amp fault 또는 abort 후보 | status read 전제 | clear 절차 미확인 | `Motor[x].AmpFault`는 매뉴얼 항목 | virtual motor라 실물 amp fault는 아님 |
| motion abort | online abort 후보 | 미측정 | 미확인 | script abort 명령은 hardware 문서 | spike |
| reset | 후보 | joint가 유지되는지 봐야 함 | 미확인 | `$$$`는 동일하지 않음 | adapter 정책 |
| reconnect | 후보 | 미측정 | 미확인 | 없음 | spike |
| next-command recovery | reset 뒤에 가능해야 함 | 미측정 | 미확인 | 없음 | spike |

자동 Evidence는 이 표가 측정된 뒤의 일이다.
