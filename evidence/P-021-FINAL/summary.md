# P-021 final summary

측정: 2026-10-25T15:36:06.281188+00:00

Official Power PMAC Simulator adapter disconnect/reconnect software recovery lifecycle. P-021 FINAL PASS를 선언하지 않는다. P-022는 시작하지 않는다.

classification: PF-1 FULL_RECOVERY_PASS
confidence: MEDIUM
causal_claim: live adapter recovery lifecycle completed on this run
confirmed: False
final_pass_declared: False
identity: ppmac / Linux
precondition ServoCtrl/ClosedLoop/AmpEna/JogPos/DesPos/Pos/ActPos: 1/1/1/2/2/2/2
precondition motion: #1j=1 completed True ack True error False JogPos 1 DesPos 1 Pos 1 ActPos 1
precondition motion T1 t_s 6.699314375175163 JogPos 1 DesPos 1 DesVel 0 Pos 1 ActPos 1 TimerEnabled 0
precondition motion T2 t_s 10.01466575008817 JogPos 1 DesPos 1 DesVel 0 Pos 1 ActPos 1 TimerEnabled 0
precondition motion T3 t_s 13.347181083168834 JogPos 1 DesPos 1 DesVel 0 Pos 1 ActPos 1 TimerEnabled 0
blocked target 2 result REJECTED error DISCONNECTED command_sent False transport_jog False
simulator_alive: True
reconnect: True
fresh readiness: RECOVERED ServoCtrl 1 ClosedLoop 1
recovery required False actions [] verification RECOVERED
post-recovery motion: #1j=3 completed True ack True error False JogPos 3 DesPos 3 Pos 3 ActPos 3
post-recovery motion T1 t_s 6.277957374928519 JogPos 3 DesPos 3 DesVel 0 Pos 3 ActPos 3 TimerEnabled 0
post-recovery motion T2 t_s 9.787938582943752 JogPos 3 DesPos 3 DesVel 0 Pos 3 ActPos 3 TimerEnabled 0
post-recovery motion T3 t_s 13.3345377498772 JogPos 3 DesPos 3 DesVel 0 Pos 3 ActPos 3 TimerEnabled 0
scripted fault detection True recovery True failed_recovery_blocked True live_fault_injection NOT_TESTED
E-fixed-1 REJECT virtual 0 pmac 0

## Regression

`python3 -m pytest -q tests/unit`

165 passed in 53.97s

P-021 final unit: `10 passed in 0.01s`

## Claim boundary

- Official Power PMAC Simulator 1.2.1 only
- software virtual Motor[1]
- PMAC motor units
- adapter disconnect/reconnect software recovery lifecycle
- ACK is not motion completion
- ActPos is Simulator virtual feedback
- LIVE_FAULT_INJECTION = NOT_TESTED
- no real Power PMAC hardware
- no physical motor
- no physical encoder
- no physical amplifier
- no physical I/O
- no EtherCAT
- no real robot
- no hard-real-time validation
- no controller commissioning
- no industrial safety claim
