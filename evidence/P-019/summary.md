# P-019B summary

측정: 2026-10-25T12:11:01.230888+00:00

이 문서는 P-019B PASS 또는 FAIL을 확정하지 않는다. 최종 판정은 별도 review다. P-020은 시작하지 않는다.

기준선: `3515fd13eb6ce39391187ffd81aa9c3cac070e6c`

## Environment

```text
Simulator: Official Power PMAC Simulator 1.2.1
firmware: 2.8.3.0 stated in the P-019B brief; this runner did not query a firmware variable
target motor: Motor[1]
path: MacBook -> 192.168.0.146:22022 -> 172.20.0.200:22 -> SSH + gpascii
credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded
```

`#1J/`는 Power PMAC on-line command `j/`다. 2011 On-Line Command Specification의 기능명은 `Jog stop, or close loop`다. 연속 jog인 `j+`/`j-`는 보내지 않았다.

위치 값은 PMAC motor unit이다. radian도 meter도 아니다. ActPos는 simulator virtual motor의 응답이며 physical encoder가 아니다.

## Gate

motion attempt authorized: YES

reasons:

- none

## Values

| name | before | after enable |
|---|---|---|
| Motor[1].ServoCtrl | 1 | 1 |
| Motor[1].ClosedLoop | 0 | 1 |
| Motor[1].Pos | 0 | 0 |
| Motor[1].ActPos | 0 | 0 |
| Motor[1].DesPos | 0 | 0 |
| Motor[1].DesVelZero | 1 | 1 |
| Motor[1].FeFatal | 0 | 0 |
| Motor[1].FeWarn | 0 | 0 |
| Motor[1].AmpFault | 0 | 0 |
| Motor[1].LimitStop | 0 | 0 |
| Motor[1].InPos | 0 | 1 |
| Motor[1].Status[0] | $4000 | $7800 |
| Motor[1].Status[1] | $0 | $0 |

## Motion Attempt #2

executed: YES
command: `#1j=1`

PMAC response stdout:

```text
STDIN Open for ASCII Input
<ACK 0x06>
// *** exit
```

stderr는 비어 있다. `error #`는 없다. gpascii process return code는 1이다. P-018과 같이 stdout에 command error가 없으면 return code만으로 실패로 보지 않았다.

| name | before command | immediate | after 1.0s |
|---|---|---|---|
| Pos | 0 | 1 | 1 |
| ActPos | 0 | 1 | 1 |
| DesPos | 0 | 1 | 1 |
| ClosedLoop | 1 | 1 | 1 |
| InPos | 1 | 1 | 1 |
| DesVelZero | 1 | 1 | 1 |
| FeFatal | 0 | 0 | 0 |
| FeWarn | 0 | 0 | 0 |
| AmpFault | 0 | 0 | 0 |
| LimitStop | 0 | 0 | 0 |
| Status[0] | $7800 | $7800 | $7800 |
| Status[1] | $0 | $0 | $0 |

1.0s는 두 read 사이의 observation delay다. servo settling 측정도 hard real-time도 아니다. 단위는 PMAC motor unit이다.

## Claim boundary

- Official Power PMAC Simulator only
- software virtual motor only
- no real Power PMAC hardware
- no physical motor
- no encoder/I/O/EtherCAT validation
- no real robot
- no hard-real-time validation

P-018 controller contract, VirtualPmacController, ExecutionGate, Validator는 수정하지 않았다.

## Tests

`python3 -m pytest -q tests/unit`

100 passed, failed 0, 51.89s. 이 실행은 `evidence/P-006`–`P-018`을 다시 쓰지 않았다.
