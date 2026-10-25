# P-021D summary

측정: 2026-10-25T15:21:37.755815+00:00

Official Power PMAC Simulator #1J/ reactivation causal experiment only. P-021 FINAL PASS를 선언하지 않는다. P-022는 시작하지 않는다. stall은 유도하지 않았다.

classification: RD-4 NO_STALL_OBSERVED
confidence: MEDIUM
causal_claim: P021D_WAITING_FOR_STALL
confirmed: False
identity: ppmac / Linux
baseline ServoCtrl/ClosedLoop/AmpEna/JogPos/DesPos/Pos/ActPos: 1/1/1/1/1/1/1
probe 1 #1j=2 observation COMPLETED ack True error False JogPos 2 DesPos 2 Pos 2 ActPos 2
probe 2 #1j=1 observation COMPLETED ack True error False JogPos 1 DesPos 1 Pos 1 ActPos 1
probe 3 #1j=2 observation COMPLETED ack True error False JogPos 2 DesPos 2 Pos 2 ActPos 2
reactivation: not performed
new motion: not performed

## Regression

`python3 -m pytest -q tests/unit`

155 passed in 53.82s

P-021D unit: `7 passed in 0.01s`

## Claim boundary

- Official Power PMAC Simulator only
- software virtual Motor[1]
- #1J/ reactivation causal experiment only
- PMAC motor units
- ACK is not motion completion
- no real Power PMAC hardware
- no physical motor
- no physical encoder
- no amplifier
- no EtherCAT
- no physical I/O
- no real robot
- no servo tuning
- no industrial commissioning
- no hard-real-time validation
