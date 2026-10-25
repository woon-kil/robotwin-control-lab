# P-021C summary

측정: 2026-10-25T14:45:21.190371+00:00

이 문서는 Simulator session-reset A/B의 restart 전 측정이다. P-021 FINAL PASS를 선언하지 않는다. P-022는 시작하지 않는다.

classification: WAITING_FOR_MANUAL_RESTART
causal_claim: not_run
before readiness: RECOVERED
before command: #1j=1 completed: False
ack: True error: False returncode: 1
T0 JogPos/DesPos/Pos/ActPos: 2/0/0/0
T1 t_s 4.4740220417734236 JogPos 1 DesPos 0 DesVel 0 Pos 0 ActPos 0 TimerEnabled 0
T2 t_s 7.453280666843057 JogPos 1 DesPos 0 DesVel 0 Pos 0 ActPos 0 TimerEnabled 0
T3 t_s 9.861730749951676 JogPos 1 DesPos 0 DesVel 0 Pos 0 ActPos 0 TimerEnabled 0

identity: ppmac / Linux
restart: not performed. SSH session is the ppmac guest. QEMU/Windows Simulator process was not stopped.
transport reconnect was not used as a restart.

ServoCtrl 1 ClosedLoop 1 JogTa -10 JogTs -50 Status[0] $7800

## Regression

`python3 -m pytest -q tests/unit`

148 passed in 54.25s

P-021C unit: `6 passed in 0.01s`

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
