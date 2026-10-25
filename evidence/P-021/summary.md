# P-021 summary

측정: 2026-10-25T13:11:17.206406+00:00

이 문서는 P-021 PASS 또는 FAIL을 확정하지 않는다. 최종 판정은 별도 review다. P-022는 시작하지 않는다.

기준선: `f7412654be99c7274361505a770438f364eaec5d`

Disconnect recovery와 fault recovery는 서로 다른 class다. reconnect 성공을 READY로 두지 않는다.

LIVE_FAULT_INJECTION = NOT_TESTED

## Live disconnect

pre communication: HEALTHY ready: RECOVERED ServoCtrl: 1.0 ClosedLoop: True
pre Pos/ActPos/DesPos: 0/0/0
pre faults: FeFatal False FeWarn False AmpFault False LimitStop False
disconnected communication: DISCONNECTED command: #1j=4 result: REJECTED command_sent: False
reconnected communication: HEALTHY result: RECOVERED read_back: True
reconnected ServoCtrl: 1.0 ClosedLoop: True fault: None
post target: 2 command: #1j=2 result: FAULTED error_code: TIMEOUT
post response: 'STDIN Open for ASCII Input\n\x06\n// *** exit \n\n'
post Pos/ActPos/DesPos: 0/0/0 InPos: True
post faults: FeFatal False AmpFault False LimitStop False
elapsed_s: 13.178414833033457

elapsed time은 SSH + portproxy + gpascii + polling이다. controller/servo latency가 아니다.

기록된 poll 이후 추가 read에서도 Pos/ActPos/DesPos는 0/0/0이었다. ServoCtrl 1, ClosedLoop 1, InPos true, DesVelZero true, Status[0] `$7800`, Status[1] `$0`, JogSpeed 32, MaxSpeed 32, HomePos 0, AmpEna 1, Coord 0. Status bit는 decode하지 않는다.

같은 gpascii 세션에서 `#1j=2` 뒤에 DesPos를 다시 읽었다. error 문구는 없었고 ACK 뒤 DesPos/Pos/ActPos는 0, ClosedLoop 1, FeFatal 0, AmpFault 0, LimitStop 0이었다.

같은 시점 read: PlusLimit 0, MinusLimit 0, JogTa -10, JogTs -50, Ctrl `Sys.ServoCtrl`. `Motor[1].Fe` assignment는 그 응답에 없었다. 이 원소들은 읽기만 했다. P-019/P-020에서는 측정하지 않았으므로 TIMEOUT의 원인으로 단정하지 않는다. simulator configuration은 쓰지 않았다.

## Scripted fault

verification: RECOVERED blocked_sent: False post: #1j=2 COMPLETED
fault remains command_sent: False jog: 0

## Virtual

blocked: REJECTED after reset: CONNECTED before ready: REJECTED
new command: p021-virtual-after result: COMPLETED

## Metrics

attempted: 9
recovered: 4
failed recovery correctly blocked: 5
post-recovery completed: 3
blocked-state submissions: 10
false recovery: 0
false completion: 0

이 metric은 project controller execution invariant다. physical safety claim이 아니다.

## Validator

E-fixed-1: REJECT virtual calls: 0 pmac calls: 0

## Regression

`python3 -m pytest -q tests/unit`

130 passed in 54.41s

P-021 unit: `10 passed in 1.30s`

## Claim boundary

- Official Power PMAC Simulator only
- software controller fault/recovery validation
- software virtual Motor[1]
- disconnect/reconnect semantics validated
- post-recovery state read-back validated
- scripted fault injection where explicitly identified
- no real Power PMAC hardware
- no physical motor/servo
- no physical amplifier fault
- no physical encoder fault
- no physical limit switch
- no physical I/O
- no EtherCAT recovery
- no safety controller
- no safety certification
- no real robot
- no hard-real-time validation
- no industrial commissioning
