# P-019C summary

측정: 2026-10-25T12:27:20.943536+00:00

이 문서는 P-019C PASS 또는 FAIL을 확정하지 않는다. 최종 판정은 별도 review다. P-020은 시작하지 않는다.

기준선: `a36f17b7be1f6b964a750a1da0d88acb13daf185`

Motor[1] seam은 `PowerPmacControllerAdapter` 안에 있다. `execute_command`와 ExecutionGate는 이 seam을 호출하지 않는다.

ServoCtrl이 0이면 `Motor[1].ServoCtrl=1`을 쓰고 read-back이 0보다 클 때만 `#1J/`를 보낸다. 이미 ClosedLoop가 1이면 `#1J/`를 다시 보내지 않는다.

FeWarn은 기록한다. FeFatal, AmpFault, LimitStop과 같지 않으며 fatal로 올리지 않는다.

Status[0]과 Status[1]은 raw token이다. status bit는 decode하지 않는다.

## Live

connect: True

enable: result COMPLETED, activation_write False, enable_command_sent False
target A 0: COMPLETED command #1j=0 DesPos 0 Pos 0 ActPos 0 polls 1 elapsed_s 6.673904957948253
target B 3: COMPLETED command #1j=3 DesPos 3 Pos 3 ActPos 3 polls 1 elapsed_s 5.845785166835412

elapsed time은 SSH + portproxy + gpascii + polling이다. controller/servo latency가 아니다.

## Negative

- nan: ok True code INVALID_TARGET command_sent False
- positive infinity: ok True code INVALID_TARGET command_sent False
- negative infinity: ok True code INVALID_TARGET command_sent False
- newline injection: ok True code INVALID_TARGET command_sent False
- separator injection: ok True code INVALID_TARGET command_sent False
- command injection: ok True code INVALID_TARGET command_sent False
- unsupported motor: ok True code UNSUPPORTED_MOTOR command_sent False
- disconnect: ok True code DISCONNECTED command_sent False
- malformed response: ok True code PROTOCOL_ERROR command_sent False
- enable failure: ok True code ENABLE_FAILED command_sent False
- pre-motion fault: ok True code AMP_FAULT command_sent False
- timeout: ok True code TIMEOUT command_sent True

## Regression

`python3 -m pytest -q tests/unit`

109 passed in 51.51s

## Claim boundary

- Official Power PMAC Simulator only
- software virtual Motor[1] only
- PMAC motor units only
- not radians/degrees/meters
- ActPos is simulator virtual feedback
- not physical encoder feedback
- no real Power PMAC hardware
- no physical servo/motor
- no physical encoder
- no I/O validation
- no EtherCAT validation
- no robot joint mapping
- no UR10e motion
- no real robot
- no hard-real-time validation
- no industrial commissioning
