# P-016 — Official VP-PMAC Integration Feasibility

조회일: 2026-10-25

판정: **CONDITIONAL GO**

RoboTwin Employment MVP는 그대로 둔다. `VirtualPmacController`와 `evidence/P-001`–`P-011`은 수정하지 않았다. `PowerPmacControllerAdapter`는 작성하지 않았다. P-017은 시작하지 않는다.

## 질문

Omron Power PMAC Simulator를 두 번째 controller backend로 써서, 실제 Power PMAC용 controller software와 integration contract를 검증할 수 있는가.

## 답

공식 Simulator와 virtual motor, PLC, motion program은 문서상 있다. 외부 프로토콜 이름도 Simulator 1.2.0 release note에 있다. 그러나 이 조사는 RoboTwin 호스트에서 Simulator로 command를 보내고 state를 읽는 세션을 측정하지 않았다. 실행 호스트는 Windows이고, 현재 RoboTwin 개발 호스트(macOS, Ubuntu)와 분리된다. 그래서 GO가 아니라 CONDITIONAL GO다.

P-017을 연다면 범위는 Environment / Communication Spike뿐이다.

## 버전

출발 기준은 IDE 4.6.4.23과 Simulator 1.2.0이었다. 2026-10-25 공식 다운로드 페이지는 그보다 새 빌드를 올린다.

| 구성요소 | 현재 공식 버전 | 날짜 | 근거 |
|---|---|---|---|
| Power PMAC IDE | 4.6.5.3 | 2026-01-19 | [PMAC Software downloads](https://www.ia.omron.com/product/tool/pmac-software/) |
| Power PMAC Simulator | 1.2.1 | 2026-01-19 | 같은 페이지, `PowerPMACSimulatorSetup.zip` |
| Power PMAC PDK | 4.6.4.4 | 2025-03-17 | 같은 페이지 |

Simulator와 firmware 짝은 [history](https://www.ia.omron.com/product/tool/pmac-software/history.htm)에 있다.

- firmware 2.8.1.x → Simulator 1.2.0
- firmware 2.8.3.x → Simulator 1.2.1

IDE 4.6.4.23과 Simulator 1.2.0의 공개 공지는 2025-04-14 Omron forum이다. history의 1.2.0 upgrade 표기는 2025-03-17이다. 두 날짜를 하나로 합치지 않는다.

설치 전에 Software License Agreement가 뜬다. 가격과 사용 범위는 이 조사에서 확인하지 않았다.

## 호스트

IDE 매뉴얼 O055-E-18은 IDE가 Microsoft Windows에서 동작하고 Windows 10과 Windows 11을 적는다. history는 IDE 4.6.3 이상에서 Windows 11을 지원한다고 적는다. 같은 매뉴얼의 RAM 항목에 virtual machine이 한 번 나온다. Ubuntu, macOS용 Simulator 패키지는 다운로드 페이지에 없다.

따라서 P620 Ubuntu의 RoboTwin / ROS 2 / Isaac과, Windows PC 또는 Windows VM의 IDE / Simulator를 분리한다. Linux native 실행은 가정하지 않는다.

## Simulator가 하는 일

2025-04-14 공지: Simulator는 PC 위의 virtual PMAC이다. 실제 motor와 I/O에 연결하지 않는다. hardware accessory module은 당시 기준 없다. virtual motor로 code를 개발하고 test하고 debug할 수 있다.

Simulator 1.2.0 history: 4종 PMAC, background tray, OPC-UA, DART, MQTT, SSH, SFTP, HTTP, PLC, virtual axis, motion program, custom user algorithm, CK5M과의 performance comparison.

CK5M comparison 항목은 등가 성능을 증명한다는 문장이 아니다. hard real-time equivalence로 쓰지 않는다.

`Sys.SimConfigOk=1`은 줄어들거나 없는 interface hardware에 full configuration을 내려도 configuration error를 내지 않게 하는 saved variable이다. 없는 hardware로 보낸 setup 값은 backup에 남지 않는다. 2025-04-15 forum에서는 CK3M project download segfault에 이 값을 저장하라고 안내했고, 그 뒤에도 같은 실패가 남아 technical support 사안이 되었다. 실제 hardware project가 Simulator에서 100% 같다는 뜻은 아니다.

## 통신

Simulator 1.2.0 release note가 SSH, SFTP, HTTP, OPC-UA, DART, MQTT를 지원 기능으로 적는다. IDE 4.6.4.23은 Simulator에 다른 PMAC처럼 연결한다고 적고, OPC server 설정과 MQTT client(외부 broker) 설정을 추가했다. 최소 firmware는 2.8.1이다.

이 문장만으로 다음을 확정하지 않는다.

- SSH가 gpascii를 여는지
- listen 주소가 localhost인지 LAN인지
- Ubuntu의 RoboTwin process가 Windows Simulator에 접속하는지
- `Motor[x].ActPos` round-trip이 Simulator에서 되는지
- PDK 4.6.4.4가 Simulator를 지원하는지

hardware에서 SSH/gpascii가 된다는 이유만으로 Simulator를 판정하지 않았다. 자동 Evidence는 이 채널이 spike로 확인된 뒤에만 가능하다. IDE GUI만으로 돌리는 구조는 채택하지 않는다.

## Adapter

`IRobotController` 옆에 `PowerPmacControllerAdapter`를 두는 구조는 가능하다. Protocol 이름과 `VirtualPmacController`는 유지한다.

직접 대응하지 않는 것:

- `MOVE_JOINT`, `READY`, `RUNNING`, `COMPLETED`라는 PMAC state 이름은 없다.
- `target_q` / `actual_q`는 radian이 아니다. `Motor[x].DesPos` / `Motor[x].ActPos` 후보를 motor unit으로 읽고 adapter가 환산한다.
- `step(dt)`는 프로젝트 시계다. Simulator servo period를 이 인자로 움직이지 않는다.
- `reset()`은 joint를 유지한다. PMAC `$$$`를 reset으로 그대로 대응하지 않는다.
- virtual motor position은 physical encoder state가 아니다.

그래서 Adapter 판정은 **PARTIAL**이다.

## 측정과 Digital Twin

통신 spike가 되면 command acceptance, rejection, fault, reset, reconnect, timeout은 측정 후보가 된다. joint 비교는 scale 이후의 virtual motor position이다. 프로젝트 시계 settling과 Simulator servo 완료는 같은 양이 아니다.

`RobotState.actual_q`로 들어오면 기존 FK, `DigitalTwinMonitor`, HMI 경로는 재사용할 수 있다. 그 전제는 read path다. 그래서 Digital Twin feedback과 cross-controller validation은 **PARTIAL**이다.

## Gap

| Layer | RoboTwin VC | VP-PMAC | Real PMAC |
|---|---|---|---|
| Application command | 있음 | adapter 필요 | 문서상 명령 채널 있음. 이 프로젝트 미검증 |
| Controller abstraction | 있음 | 아직 없음 | 해당 없음 |
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

## Claim boundary

쓸 수 있는 이후 표현:

- Power PMAC Simulator 기반 Controller Software Integration
- Official Virtual Power PMAC Backend Integration
- PMAC Motion/PLC Software Validation using Virtual Motors
- Cross-Controller Validation

쓰지 않는다.

- Real Power PMAC validated
- Real servo validated
- Encoder validated
- EtherCAT hardware validated
- Industrial safety validated
- Real robot validated
- Production controller proven
- Hard real-time performance proven

현재 `VirtualPmacController` 문장도 유지한다. PROJECT VIRTUAL CONTROLLER. NOT VERIFIED AGAINST REAL POWER PMAC.

## 근거 파일

- `evidence/P-016/official_sources.md`
- `evidence/P-016/version_matrix.md`
- `evidence/P-016/capability_matrix.md`
- `evidence/P-016/communication_matrix.md`
- `evidence/P-016/contract_mapping.md`
- `evidence/P-016/gap_matrix.md`
- `evidence/P-016/architecture.md`
- `evidence/P-016/decision.md`
