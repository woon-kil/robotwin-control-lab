# P-022 environment recovery

측정: 2026-10-25T23:33:09.704184+00:00

classification: BR-1 BASELINE_READY
confidence: MEDIUM
confirmed: False
p022_final_pass_declared: False

## Incident

Before clean restart:

QEMU repeatedly displayed: "Xenomai: suspending kernel thread ... ('rtitask')"

Existing SSH path became unreachable.

P-022 live execution was stopped.

No attempt was made to classify the failure as a RoboTwin, adapter, motion, or Digital Twin defect.

The Simulator was cleanly restarted manually.

Post-restart baseline was then revalidated independently.

incident: SIMULATOR_RUNTIME_FAILURE
root_cause: NOT_CONFIRMED

## Transport

SSH: PASS
hostname: ppmac
OS: Linux
eth0: 2: eth0    inet 172.20.0.200/24 brd 172.20.0.255 scope global eth0\       valid_lft forever preferred_lft forever
gpascii: PASS
Sys.ServoCount: 221417

## CLEAN_RESTART_S0

ServoCtrl: 0
ClosedLoop: 0
AmpEna: 0
JogPos: 0
DesPos: 0
Pos: 0
ActPos: 0
FeFatal: 0
AmpFault: 0
LimitStop: 0
Status0: $0
Status1: $0

## Activation

required: True
ServoCtrl write: True
ServoCtrl read-back: 1
#1J/: True
ClosedLoop read-back: 1

## READY_S1

ready: True
ServoCtrl: 1
ClosedLoop: 1
AmpEna: 1
FeFatal: 0
AmpFault: 0
LimitStop: 0

## Motion probe

target: 1
command: #1j=1
T0 JogPos/DesPos/Pos/ActPos: 0/0/0/0
T1 t_s 6.224366166163236 JogPos/DesPos/Pos/ActPos: 1/1/1/1
T2 t_s 10.49279254116118 JogPos/DesPos/Pos/ActPos: 1/1/1/1
T3 t_s 13.809601083397865 JogPos/DesPos/Pos/ActPos: 1/1/1/1
faults after T3: FeFatal 0, AmpFault 0, LimitStop 0
ack: true
error: false
returncode: 1
completed: True

## Claim boundary

- Official Power PMAC Simulator 1.2.1 only
- software virtual Motor[1]
- PMAC motor units
- ActPos is simulator virtual feedback
- ACK is not motion completion
- Sys.ServoCount is not a servo-state claim
- Xenomai message root cause is NOT_CONFIRMED
- no real Power PMAC hardware
- no physical motor, encoder, amplifier, I/O, or EtherCAT
- no real robot
- no hard-real-time validation
- no industrial safety claim

## Tests

baseline recovery: 8 passed in 0.01s
full regression: 173 passed in 53.75s

Gate: P-022 ENVIRONMENT RECOVERY PASS. P-022 main integration may resume. P-022 FINAL PASS는 선언하지 않는다.
P-023 started: NO
