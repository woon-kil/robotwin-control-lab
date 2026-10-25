# P-021B summary

측정: 2026-10-25T14:26:58.572022+00:00

이 문서는 JogPos에서 DesPos로 이어지는 trajectory 조사다. P-021 motion 해결을 선언하지 않는다. P-021A의 RC-3 결론은 유지한다. P-022는 시작하지 않는다.

classification: RB-1 COMMAND_TARGET_ONLY
confidence: MEDIUM
confirmed: False
command: #1j=2
ack: True error: False returncode: 1
motion_completed: False

T0 JogPos/DesPos/DesVel/Pos/ActPos: 1/0/0/0/0
T1 t_s 5.048473875038326 JogPos 2 DesPos 0 DesVel 0 Pos 0 ActPos 0 ActVel 0 TimerEnabled 0
T2 t_s 7.433833916205913 JogPos 2 DesPos 0 DesVel 0 Pos 0 ActPos 0 ActVel 0 TimerEnabled 0
T3 t_s 11.094978916225955 JogPos 2 DesPos 0 DesVel 0 Pos 0 ActPos 0 ActVel 0 TimerEnabled 0

JogTa/JogTs class: NOT_ENOUGH_EVIDENCE
current JogTa/JogTs: -10/-50
MaxPos/MinPos: 0/0
Coord: 0 Coord[0].ProgRunning: 0
Desired.TimerEnabled at state read: 0

추가 element: `Motor[1].Desired.TimerEnabled`, `Motor[1].Desired.Pos`, `Motor[1].Desired.Vel`, `Motor[1].JogVel`, `Coord[0].ProgRunning`. Software Reference index에 있다. TimerEnabled는 definite jog section이 실행 중이면 1이라는 vendor forum 설명 때문에 읽었다. 세 sample 모두 0이었다. Desired.Pos/Desired.Vel은 0이었다. JogVel은 state read에서 32였다. Coord[0].ProgRunning은 0이다. 이 값들은 쓰지 않았다.

intervention: not performed
restart: not performed

## Regression

`python3 -m pytest -q tests/unit`

142 passed in 53.75s

P-021B unit: `6 passed in 0.01s`

## Claim boundary

- JogPos → DesPos root-cause investigation only
- Official Power PMAC Simulator only
- software virtual Motor[1]
- historical evidence is not back-filled
- unmeasured historical state is NOT_MEASURED
- ACK is not motion completion
- no real Power PMAC hardware
- no physical motor
- no physical encoder
- no physical amplifier
- no physical I/O
- no EtherCAT
- no real robot
- no servo tuning
- no controller commissioning
