# Power PMAC Simulator Validation

RoboTwin Control Lab의 controller integration extension 기록이다. 독립 프로젝트가 아니다. 이 문서는 `evidence/`를 다시 쓰지 않고, 승인된 Gate의 측정만 연결한다.

## Scope

Official Power PMAC Simulator 1.2.1의 software virtual Motor[1]이다. 단위는 PMAC motor unit이다. `ActPos`는 Simulator virtual feedback이다.

검증한 것:

```text
command
state feedback
fail-closed execution
reconnect recovery
controller-domain Target vs Actual
```

검증하지 않은 것:

```text
real Power PMAC hardware
physical motor, encoder, amplifier, I/O, EtherCAT
real robot
Motor[1] to UR10e joint mapping
PMAC motor unit to radian conversion
physical accuracy
hard real-time
controller commissioning
industrial safety
industrial reliability
live fault injection
```

`robot_joint_mapping = NOT_DEFINED`
`unit_conversion = NOT_DEFINED`

Employment MVP의 ExecutionGate는 `VirtualPmacController`를 호출한다. `PowerPmacControllerAdapter.execute_command`는 PMAC motor command를 보내지 않는다.

## Architecture

```text
Operator / HMI
      ↓
Canonical Robot API
      ↓
Pre-Execution Validator
      ↓
Execution Gate
      ↓
IRobotController
      ├──────── VirtualPmacController
      │
      └──────── PowerPmacControllerAdapter
                         ↓
               Official Power PMAC Simulator
                         ↓
                    Motor[1]
                         ↓
              ControllerAxisObservation
                         ↓
              ControllerObservationBridge
                         ↓
              Digital Twin Observation
                         ↓
                 Target vs Actual
```

`Motor[1]`에서 UR10e Joint1로 가는 화살표는 없다.

```text
Robot Domain
- radians
- meters
- kinematics / TCP

Controller Test Domain
- Motor[1]
- PMAC motor unit
- Simulator virtual feedback
```

## P-016 ~ P-023 Timeline

| Gate | 검증 | 승인 상태 | 당시 기록 |
| --- | --- | --- | --- |
| P-016 | 적용 가능성 | PASS | [`evidence/P-016/decision.md`](../evidence/P-016/decision.md)의 원문은 CONDITIONAL GO |
| P-017 | Simulator communication | PASS | [`evidence/P-017/summary.md`](../evidence/P-017/summary.md) |
| P-018 | Controller adapter | PASS | [`evidence/P-018/summary.md`](../evidence/P-018/summary.md) |
| P-019 | Motor[1] motion/state | PASS | [`evidence/P-019/summary.md`](../evidence/P-019/summary.md), [`evidence/P-019C/summary.md`](../evidence/P-019C/summary.md) |
| P-020 | Semantic comparison | PASS | [`evidence/P-020/summary.md`](../evidence/P-020/summary.md) |
| P-021 | Disconnect/recovery | PASS | [`evidence/P-021-FINAL/summary.md`](../evidence/P-021-FINAL/summary.md) |
| P-022 | Target-vs-Actual integration | PASS | [`evidence/P-022/main/summary.md`](../evidence/P-022/main/summary.md) |
| P-023 | Repeated reliability | PASS | [`evidence/P-023/summary.md`](../evidence/P-023/summary.md) |

P-016 원문은 통신 실측 전의 문서 판정이다. 그 파일은 수정하지 않았다. P-017 이후 측정이 communication을 채웠다. P-021과 P-022, P-023 evidence의 `final_pass_declared`는 false로 남아 있다. 승인된 Gate 상태와 evidence 파일의 자기 선언은 다르다.

## Communication

P-017은 Mac client에서 Official Simulator의 `gpascii`까지 command round-trip을 측정했다. 그 시간은 network latency나 controller real-time latency가 아니다. 기록은 [`evidence/P-017/summary.md`](../evidence/P-017/summary.md)다.

## Adapter

`PowerPmacControllerAdapter`는 SSH / `gpascii` transport 뒤에 있다. PMAC syntax와 state 해석은 adapter와 transport 안에 둔다. Core `IRobotController` enum, ExecutionGate, Pre-Execution Validator, Employment MVP는 이 backend에 맞춰 바꾸지 않았다.

## Motor Motion

P-019는 software virtual Motor[1]에서 jog command 이후 DesPos, Pos, ActPos read-back을 확인했다. `#1j=`의 ACK는 completion이 아니다. Completion은 target position read-back이다.

## Semantic Comparison

P-020은 `VirtualPmacController`와 Official Simulator adapter의 acceptance, completion, rejection, fault, disconnect, state semantics를 비교했다. Virtual joint radian과 PMAC motor unit을 같은 숫자로 맞추지 않았다. `DIFFERENT`와 `NOT_COMPARABLE`을 `MATCH`로 바꾸지 않았다.

## Recovery

P-021 final live cycle은 다음이다.

```text
disconnect
→ motion attempt REJECTED / DISCONNECTED
→ command_sent = false
→ reconnect
→ fresh read already RECOVERED
→ NEW command
→ JogPos / DesPos / Pos / ActPos target
```

Reconnect 성공이나 fault flag clear만으로 motion을 허용하지 않았다. Fresh read가 READY가 아니면 activation command를 보내지 않는다. 기록은 [`evidence/P-021-FINAL/summary.md`](../evidence/P-021-FINAL/summary.md)다.

## Digital Twin Integration

P-022 live sequence는 시작 Pos 1에서 `2 → 3 → 1`이다. 세 cycle 모두 JogPos, DesPos, Pos, ActPos가 target과 같았고 tracking error는 `0 PMAC motor unit`이다.

```text
Official Power PMAC Simulator
        ↓
Virtual Motor[1]
        ↓
PowerPmacControllerAdapter
        ↓
ControllerAxisObservation
        ↓
ControllerObservationBridge
        ↓
RoboTwin controller-domain observation
        ↓
Target vs Actual
```

`DigitalTwinMonitor`의 robot joint radian sample은 이 경로로 덮지 않는다. 기록은 [`evidence/P-022/main/summary.md`](../evidence/P-022/main/summary.md)다.

## Reliability

P-023은 100 controller motion cycle과 5 reconnect trial이다. 100 cycle을 industrial reliability라고 쓰지 않는다. P-011의 500-cycle deterministic project workload와 다른 evidence다.

| Metric | Result |
| --- | ---: |
| Repeated motion | 100/100 |
| Tracking mismatch | 0 |
| Unexpected timeout | 0 |
| Unexpected disconnect | 0 |
| Controller fault | 0 |
| State leakage | 0 |
| Reconnect lifecycle | 5/5 |
| Disconnected command blocked | 5/5 |
| False controller submission | 0 |
| Full regression | 197 passed |

Tracking error는 `0 PMAC motor unit`이다.

Observed integration elapsed, P-023 100 cycle 반올림:

```text
Mean  6.124 s
P50   6.045 s
P95   6.685 s
P99   7.457 s
Max   7.590 s
```

SSH, Windows portproxy, gpascii, polling이 포함된 시간이다. Servo latency, controller cycle time, real-time performance가 아니다.

원값:

- [`evidence/P-023/motion_metrics.json`](../evidence/P-023/motion_metrics.json)
- [`evidence/P-023/reconnect_metrics.json`](../evidence/P-023/reconnect_metrics.json)
- [`evidence/P-023/regression_results.json`](../evidence/P-023/regression_results.json)

## Failure Investigation

Reconnect 이후 ACK는 왔고 JogPos는 바뀌었지만 DesPos, Pos, ActPos는 target에 가지 않았다. `ACK ≠ COMPLETED`라서 그 측정을 PASS로 두지 않고 HOLD했다.

조사는 command acceptance, JogPos, DesPos, Pos, ActPos를 분리했다. Simulator restart 이후 activation state가 달라지는 것도 보았다. Root cause remained NOT_CONFIRMED.

이후 fresh read와 NEW command로 DesPos, Pos, ActPos가 target에 도달하는 lifecycle을 다시 측정한 뒤에 P-021을 PASS했다.

금지하는 결론:

```text
root cause was Xenomai
Power PMAC bug를 해결했다
Simulator activation bug를 수정했다
```

## Runtime Incident

P-022 시작 전에 QEMU 화면에서 `Xenomai: suspending kernel thread ... ('rtitask')`가 반복되고 SSH가 unreachable이 되었다.

```text
SIMULATOR_RUNTIME_FAILURE
root cause = NOT_CONFIRMED
```

Clean restart 후 [`evidence/P-022/environment-recovery/summary.md`](../evidence/P-022/environment-recovery/summary.md)에서 BR-1 `BASELINE_READY`를 확인하고 P-022를 재개했다. 이 사건은 P-021의 ACK/completion 조사와 섞지 않는다.

## Claim Boundary

허용하는 한 문장:

> Official Power PMAC Simulator virtual Motor[1]을 RoboTwin controller observation 계층과 통합하고, 100회 반복 motion과 5회 disconnect/reconnect lifecycle에서 Target-vs-Actual 및 fail-closed behavior를 검증했다.

짧은 문장:

> Power PMAC Simulator 연동 반복 검증: 100 motion cycles + 5 reconnect recovery trials.

금지:

```text
산업 현장 reliability 검증
Power PMAC hardware reliability 검증
실제 로봇 100회 연속 운전
industrial reliability validated
production reliability
hardware reliability
24/7 reliability
real-time latency
servo latency
robot accuracy
physical accuracy
```

## Evidence Index

| 기록 | 경로 |
| --- | --- |
| P-016 decision | [`evidence/P-016/decision.md`](../evidence/P-016/decision.md) |
| P-017 communication | [`evidence/P-017/summary.md`](../evidence/P-017/summary.md) |
| P-018 adapter | [`evidence/P-018/summary.md`](../evidence/P-018/summary.md) |
| P-019 motion spike | [`evidence/P-019/summary.md`](../evidence/P-019/summary.md) |
| P-019C motion seam | [`evidence/P-019C/summary.md`](../evidence/P-019C/summary.md) |
| P-020 comparison | [`evidence/P-020/summary.md`](../evidence/P-020/summary.md) |
| P-021 final recovery | [`evidence/P-021-FINAL/summary.md`](../evidence/P-021-FINAL/summary.md) |
| P-022 environment recovery | [`evidence/P-022/environment-recovery/summary.md`](../evidence/P-022/environment-recovery/summary.md) |
| P-022 integration | [`evidence/P-022/main/summary.md`](../evidence/P-022/main/summary.md) |
| P-023 reliability | [`evidence/P-023/summary.md`](../evidence/P-023/summary.md) |

P-021, P-022, P-023 evidence는 historical record다. Packaging은 링크만 한다.
