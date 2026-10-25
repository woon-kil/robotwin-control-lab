# P-016 contract mapping

조회일: 2026-10-25. PMAC에 RoboTwin state 이름이 있다고 가정하지 않는다. adapter가 의미를 맞춘다.

| RoboTwin | VP-PMAC 후보 | 직접 Mapping | Adapter 필요 | 미지원 |
|---|---|---:|---:|---:|
| CONNECTED | SSH 또는 IDE가 Simulator에 붙은 상태 | | x | |
| READY | motor enable 이후, program이 돌지 않는 상태. 이름 없음 | | x | |
| RUNNING | motion program 실행 또는 motor가 목표로 이동 중 | | x | |
| MOVE_JOINT | motion program 또는 online jog/target. 명령 이름 없음 | | x | |
| target_q | `Motor[x].DesPos` 또는 project 변수. motor unit | | x | |
| actual_q | `Motor[x].ActPos` 또는 `Motor[x].Pos`. virtual motor feedback. encoder 아님 | | x | |
| COMPLETED | in-position 또는 program 종료 bit. settling 규칙과 다름 | | x | |
| ERROR | `Motor[x].AmpFault` 등 status. fault code 문자열은 없음 | | x | |
| reset | abort와 fault clear 후보. `$$$`는 joint 유지 reset이 아님 | | x | |

`step(dt)`는 표의 최소 항목 밖이다. `IRobotController.step`은 프로젝트 시계를 전진한다. Simulator는 자기 servo period로 돈다. adapter가 `step(dt)`를 호출받아도 Simulator 시계를 그 값으로 바꾸지 못한다.

`servo_on` 후보는 virtual motor의 `Motor[x].ServoCtrl`과 amp enable 설정이다. 실물 amp가 없다.

controller-side 프로그램 후보는 구현하지 않는다.

| 역할 | 후보 | 메모 |
|---|---|---|
| Command receive | online command 또는 PLC가 사용자 변수를 폴링 | 외부 client가 있어야 성립 |
| Validation / interlock | PLC | RoboTwin Pre-Execution Validator를 대체하지 않음 |
| Motion start | motion program | |
| Virtual motor execution | `EncTable` type 11 simulated loop | physical encoder 아님 |
| Completion detection | motor status bit | `COMPLETED`로 normalize |
| Status update | 변수 또는 motor status | adapter가 읽음 |
| FAULT → STOP / BLOCK | abort, amp fault | |
| RESET → READY | fault clear 후 enable | `$$$`를 RoboTwin `reset`과 동일시하지 않음 |

Script가 PLC와 motion program을 담는다. C는 Simulator에서 Partial이다. kinematics program은 Unknown이다.
