# P-016 capability matrix

조회일: 2026-10-25. Unknown을 Supported로 올리지 않는다.

| Capability | Supported | Partial | Unsupported | Unknown | 근거 |
|---|---:|---:|---:|---:|---|
| Virtual PMAC CPU | x | | | | 2025-04-14 공지. PC 위 virtual PMAC |
| Virtual Motor | x | | | | history 1.2.0 virtual axis. 공지의 virtual motor. MoMo: physical motor 설정 불가 |
| Motion Program | x | | | | history 1.2.0 |
| PLC Program | x | | | | history 1.2.0 |
| Power PMAC Script | x | | | | PLC와 motion program의 언어. IDE가 Simulator에 연결된다고 4.6.4.23 history에 있음 |
| C Program | | x | | | IDE 매뉴얼은 C를 지원. Simulator note는 custom user algorithm. Simulator에서 C application 실행이라는 별도 문장은 없음 |
| Kinematics Program | | | | x | Simulator release note에 이름이 없음 |
| Controller Variables | | x | | | `Sys.SimConfigOk`, `Motor[]`, `EncTable` type 11은 문서에 있음. Simulator 변수 목록 전체는 확인하지 않음 |
| Breakpoint / Debug | | x | | | IDE 매뉴얼에 breakpoint. Simulator 전용 debug 문장은 없음 |
| Project Download | | x | | | IDE 연결은 문서상 가능. CK3M project는 `Sys.SimConfigOk=1` 뒤에도 segfault가 남음 |
| Real Motor | | | x | | 2025-04-14 공지 |
| Encoder Hardware | | | x | | 공지의 실제 I/O 없음. encoder 선택이 연결 장치에서 거부됨 |
| Physical I/O | | | x | | 2025-04-14 공지 |
| EtherCAT physical device | | | x | | 실제 I/O와 accessory module 없음. IDE의 EtherCAT 화면은 hardware용 |
| Hardware accessory module | | | x | | 2025-04-14 공지 |
| Servo tuning validation | | x | | | virtual loop는 `EncTable` type 11에 문서화. 실제 servo tuning validation은 아님 |
| Real-time performance equivalence | | | x | | history는 CK5M comparison 기능을 적을 뿐 등가라고 하지 않음 |

## 호스트

| 항목 | 지원 여부 | 공식 근거 | 결정 |
|---|---|---|---|
| Windows 10 | IDE 지원 | O055-E-18 | IDE 호스트로 사용 가능. Simulator만의 OS 문장은 없고 Windows setup으로 배포 |
| Windows 11 | IDE 4.6.3 이상 | history, O055-E-18 | IDE 호스트로 사용 가능 |
| Ubuntu 24.04 | 패키지 없음 | 다운로드 페이지와 IDE 매뉴얼 | Simulator 호스트로 쓰지 않음. RoboTwin / ROS 2 / Isaac은 여기 유지 |
| macOS | 패키지 없음 | 같은 근거 | Simulator 호스트로 쓰지 않음 |
| VM | IDE RAM 문구만 | O055-E-18, virtual machine 2 GB | 지원 매트릭스 아님. spike 후보는 Windows VM |
| Power PMAC IDE | 4.6.5.3 | 다운로드 페이지 | Windows에 설치 |
| Simulator standalone | 1.2.1 | `PowerPMACSimulatorSetup.zip`. 1.2.0은 IDE 4.6.4.x 설치 끝 프롬프트로도 설치 가능 | Windows setup. Linux native 가정 금지 |

분리:

```text
P620 Ubuntu
  └─ RoboTwin / ROS 2 / Isaac

Windows PC 또는 Windows VM
  └─ Power PMAC IDE 4.6.5.3
  └─ Power PMAC Simulator 1.2.1
```
