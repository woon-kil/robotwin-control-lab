# P-022 main integration

측정: 2026-10-25T23:44:52.725631+00:00

classification: DT-1 FULL_INTEGRATION_PASS
p022_final_pass_declared: False

prior environment incident: SIMULATOR_RUNTIME_FAILURE
root cause: NOT_CONFIRMED
clean restart recovery: BR-1 BASELINE_READY

## Precondition

ready: True
ServoCtrl: 1
ClosedLoop: 1
Pos: 1
FeFatal: 0
AmpFault: 0
LimitStop: 0

## Cycles

C1 target 2 command #1j=2 JogPos 2 DesPos 2 Pos 2 ActPos 2 tracking_error 0.0 completed True
  T1 t_s 6.040898458100855 JogPos 2 DesPos 2 Pos 2 ActPos 2
  T2 t_s 9.276900041848421 JogPos 2 DesPos 2 Pos 2 ActPos 2
  T3 t_s 12.524707582779229 JogPos 2 DesPos 2 Pos 2 ActPos 2
C2 target 3 command #1j=3 JogPos 3 DesPos 3 Pos 3 ActPos 3 tracking_error 0.0 completed True
  T1 t_s 6.011768124997616 JogPos 3 DesPos 3 Pos 3 ActPos 3
  T2 t_s 9.24551370786503 JogPos 3 DesPos 3 Pos 3 ActPos 3
  T3 t_s 12.510629499796778 JogPos 3 DesPos 3 Pos 3 ActPos 3
C3 target 1 command #1j=1 JogPos 1 DesPos 1 Pos 1 ActPos 1 tracking_error 0.0 completed True
  T1 t_s 5.982858167029917 JogPos 1 DesPos 1 Pos 1 ActPos 1
  T2 t_s 9.197114916983992 JogPos 1 DesPos 1 Pos 1 ActPos 1
  T3 t_s 12.462493124883622 JogPos 1 DesPos 1 Pos 1 ActPos 1

## Claim boundary

- Official Power PMAC Simulator 1.2.1
- software virtual Motor[1]
- controller-domain Target vs Actual
- PMAC motor units
- ActPos is Simulator virtual feedback
- robot_joint_mapping = NOT_DEFINED
- unit_conversion = NOT_DEFINED
- no real Power PMAC hardware
- no physical motor
- no physical encoder
- no amplifier
- no physical I/O
- no EtherCAT
- no real robot
- no robot joint accuracy claim
- no physical positioning accuracy claim
- no hard-real-time validation
- no controller commissioning
- no industrial safety claim

## Tests

P-022 observation: 11 passed
P-021 final: 10 passed
environment recovery: 8 passed
full regression: 184 passed in 54.13s

Gate: P-022 REVIEW REQUIRED
P-023 started: NO
