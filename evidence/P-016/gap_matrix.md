# P-016 gap matrix

조회일: 2026-10-25. Real PMAC 열은 제품 문서상 능력이다. 이 프로젝트의 실물 검증이 아니다.

| Layer | RoboTwin VC | VP-PMAC | Real PMAC |
|---|---|---|---|
| Application command | RobotCommand 있음 | 없음. adapter 필요 | 명령 채널은 문서상 있음. 미검증 |
| Controller abstraction | `IRobotController` 있음 | adapter 미작성 | 해당 없음 |
| PMAC Script | 없음 | 문서상 있음 | 문서상 있음. 미검증 |
| PLC | 없음 | 문서상 있음 | 문서상 있음. 미검증 |
| Motion Program | 없음 | 문서상 있음 | 문서상 있음. 미검증 |
| Virtual Motor | 프로젝트 slew. PMAC virtual motor 아님 | 문서상 있음 | software motor는 문서상 가능. 실물 검증 아님 |
| Real Motor | 없음 | 없음 | 문서상 있음. 미검증 |
| Encoder | 없음 | 없음 | 문서상 있음. 미검증 |
| Physical I/O | 없음 | 없음 | 문서상 있음. 미검증 |
| EtherCAT Hardware | 없음 | 없음 | 문서상 있음. 미검증 |
| Servo Dynamics | 프로젝트 slew | virtual loop만. 실서보 등가 아님 | 문서상 있음. 미검증 |
| Safety Hardware | 없음 | 없음 | 이 조사에서 확인하지 않음 |
| Real Robot | 없음 | 없음 | 없음 |

VP-PMAC의 motor feedback을 encoder state라고 부르지 않는다.
