# P-020 summary

측정: 2026-10-25T12:44:48.251749+00:00

이 문서는 P-020 PASS 또는 FAIL을 확정하지 않는다. 최종 판정은 별도 review다. P-021은 시작하지 않는다.

기준선: `bb58ea3a03cb7147fbc14e3ea59f8fe35d79a2c6`

비교는 semantic이다. Virtual radian과 PMAC motor unit의 숫자는 비교하지 않는다.

## Virtual

operation: MOVE_JOINT A-zero
result: submit ACCEPTED execution COMPLETED
target observable: True units radian
actual observable: True
fault: None

## Power PMAC

live ok: True
target: 0 pmac motor unit
command: #1j=0
result: COMPLETED
Pos: 0 ActPos: 0 DesPos: 0
ClosedLoop: True InPos: True
FeFatal: False FeWarn: False AmpFault: False LimitStop: False
polls: 1 elapsed_s: 6.262350667035207

elapsed time은 SSH + portproxy + gpascii + polling이다. controller/servo latency가 아니다.

## Comparison

- command_acceptance: MATCH
- completion: MATCH
- target_observable: MATCH
- actual_observable: MATCH
- fault_free: MATCH
- numeric_position_equality: NOT_COMPARABLE

## Validator

E-fixed-1: REJECT
virtual calls: 0
pmac calls: 0

## Regression

`python3 -m pytest -q tests/unit`

120 passed in 52.72s

P-020 unit: `11 passed in 1.31s`

## Claim boundary

- Cross-controller semantic validation only
- VirtualPmacController vs Official Power PMAC Simulator software backend
- not numeric position equivalence
- not radian ↔ PMAC unit calibration
- not trajectory equivalence
- not robot-joint mapping
- not UR10e controller validation
- Official Power PMAC Simulator only
- software virtual Motor[1] only
- no real Power PMAC hardware
- no physical motor/servo
- no physical encoder
- no EtherCAT
- no physical I/O
- no real robot
- no hard-real-time validation
- no industrial commissioning
