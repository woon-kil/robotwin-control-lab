# P-021A summary

측정: 2026-10-25T14:13:22.866874+00:00

이 문서는 root-cause investigation이다. P-021 FINAL PASS를 선언하지 않는다. P-021은 HOLD다. P-022는 시작하지 않는다.

classification: RC-3 BASELINE_NO_LONGER_REPRODUCIBLE
confidence: MEDIUM
fresh_baseline_reproducible: False
reconnect_failure_reproducible: False
root_cause_confirmed: False
next_action: HOLD. Fresh baseline failed, so the P-021 result is not attributed to reconnect.

## Trial 1

fresh command: #1j=1 reached: False
fresh Pos/ActPos/DesPos before: 0/0/0
fresh Pos/ActPos/DesPos after: 0/0/0
fresh ack: True error: False returncode: 1
reconnect: NOT_RUN

## Trial 2

fresh command: #1j=1 reached: False
fresh Pos/ActPos/DesPos before: 0/0/0
fresh Pos/ActPos/DesPos after: 0/0/0
fresh ack: True error: False returncode: 1
reconnect: NOT_RUN

## Observed split

Trial 1에서 `#1j=1`은 JogPos를 2에서 1로 바꿨다. DesPos, Pos, ActPos, DesVel은 0을 유지했다. Trial 2는 JogPos가 이미 1이었고, 같은 `#1j=1` 뒤에도 위치는 0이었다. ACK와 return code 1은 두 trial 모두에 있었다.

요청한 snapshot 변수 28개는 모두 응답에 있었다. Trial 1의 변화는 JogPos뿐이고, Trial 2는 조사한 변수가 모두 UNCHANGED다.

추가 read: MaxPos 0, MinPos 0, SoftLimit 0, SoftPlusLimit 0, SoftMinusLimit 0, SoftLimitDir 0. 2011 On-Line Command Specification은 software positive limit bit가 Motor[x].MaxPos > Motor[x].MinPos일 때 active라고 적는다. 이번 측정은 그 조건이 아니다. PlusLimit 0은 같은 문서의 hardware limit status가 현재 set되지 않았다는 뜻으로 읽는다. 이 값들은 쓰지 않았다.

JogTa, JogTs, Coord, Status[0]은 실패 trial 안에서 달라지지 않았다. 단독 원인으로 쓰지 않는다. Status bit는 decode하지 않는다.

## Regression

`python3 -m pytest -q tests/unit`

136 passed in 53.70s

P-021A unit: `6 passed in 0.01s`

## Claim boundary

- Root-cause investigation only
- Official Power PMAC Simulator only
- software virtual Motor[1]
- read-only state comparison unless explicitly recorded experimental intervention
- no real Power PMAC hardware
- no physical motor
- no physical encoder
- no physical amplifier
- no physical I/O
- no EtherCAT
- no real robot
- no servo tuning
- no controller commissioning
- ACK is not treated as motion completion
