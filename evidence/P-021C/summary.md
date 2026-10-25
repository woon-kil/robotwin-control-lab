# P-021C summary

측정: 2026-10-25T14:59:52.806902+00:00

Official Power PMAC Simulator session-reset A/B experiment only. P-021 FINAL PASS를 선언하지 않는다. P-022는 시작하지 않는다.

classification: RC-S3 RESTART_CHANGES_ACTIVATION_STATE
confidence: LOW
causal_claim: FOLLOW-UP CAUSAL TEST REQUIRED
confirmed: False

before readiness: RECOVERED
before command: #1j=1 completed: False
before T0 JogPos/DesPos/Pos/ActPos: 2/0/0/0
before T1 t_s 4.4740220417734236 JogPos 1 DesPos 0 Pos 0 ActPos 0 TimerEnabled 0
before T2 t_s 7.453280666843057 JogPos 1 DesPos 0 Pos 0 ActPos 0 TimerEnabled 0
before T3 t_s 9.861730749951676 JogPos 1 DesPos 0 Pos 0 ActPos 0 TimerEnabled 0

after initial readiness: NOT_READY
identity: ppmac / Linux
priority Motor[1].ServoCtrl: 1 -> 0 RESET_TO_ZERO
priority Motor[1].ClosedLoop: 1 -> 0 RESET_TO_ZERO
priority Motor[1].JogPos: 2 -> 0 RESET_TO_ZERO
priority Motor[1].AmpEna: 1 -> 0 RESET_TO_ZERO
priority Motor[1].Status[0]: $7800 -> $0 CHANGED
state difference is recorded. It is not declared the root cause.
activation required: True actions: ['Motor[1].ServoCtrl=1', '#1J/'] blocked: False
post-activation readiness: RECOVERED
after command: #1j=1 completed: True ack: True error: False returncode: 1
after T0 JogPos/DesPos/Pos/ActPos: 0/0/0/0
after T1 t_s 5.739892584038898 JogPos 1 DesPos 1 DesVel 0 Pos 1 ActPos 1 TimerEnabled 0
after T2 t_s 9.134135499829426 JogPos 1 DesPos 1 DesVel 0 Pos 1 ActPos 1 TimerEnabled 0
after T3 t_s 12.337829041993245 JogPos 1 DesPos 1 DesVel 0 Pos 1 ActPos 1 TimerEnabled 0

Optional C test는 수행하지 않았다. `#1j=1` 전에 `Motor[1].ServoCtrl=1`과 `#1J/`가 실행되어 restart와 reactivation이 분리되지 않았다.

## Regression

`python3 -m pytest -q tests/unit`

148 passed in 54.12s

P-021C unit tests are included in that run. The file contains 6 tests.

## Claim boundary

- Official Power PMAC Simulator session-reset A/B experiment only
- software virtual Motor[1]
- restart correlation is not automatically root-cause proof
- ACK is not motion completion
- PMAC motor units only
- no real Power PMAC hardware
- no physical motor
- no physical servo
- no physical encoder
- no physical amplifier
- no physical I/O
- no EtherCAT
- no real robot
- no servo tuning
- no industrial commissioning
- no hard-real-time validation
