# P-018 — Existing Controller Contract

조사 시각의 구현 기준은 `controller/virtual_pmac.py`, `validator/gate.py`다. 이 문서를 쓰기 전에 interface는 수정하지 않았다. P-018도 이 contract를 PMAC에 맞춰 바꾸지 않는다.

P-018 controller state represents communication/integration state only. It does not claim verified PMAC servo or motor state.

## IRobotController

`IRobotController`는 `Protocol`이다. 구현 class가 아니다.

메서드:

| 메서드 | 반환 |
|---|---|
| `connect()` | `None` |
| `disconnect()` | `None` |
| `servo_on()` | `None` |
| `servo_off()` | `None` |
| `execute_command(command: JointCommand)` | `CommandResult` |
| `step(dt_s: float)` | `RobotState` |
| `get_state()` | `RobotState` |
| `reset()` | `None` |

`VirtualPmacController`가 이 메서드를 갖는다. `enable`/`disable`은 `servo_on`/`servo_off`의 별칭이다. `execute_command`는 `submit`을 호출한다. `ExecutionGate`는 `execute_command`가 아니라 `submit`과 `trace`를 호출한다.

## 상태 어휘

`CONTROLLER_STATES`:

```text
DISCONNECTED, CONNECTED, SERVO_OFF, SERVO_ON, HOMING,
READY, RUNNING, HOLD, ERROR, ESTOP
```

이 baseline이 체류 상태로 쓰는 것: `DISCONNECTED`, `CONNECTED`, `READY`, `RUNNING`, `SERVO_OFF`, `ERROR`.

체류하지 않는 이름: `SERVO_ON`, `HOMING`, `HOLD`, `ESTOP`. `servo_on`의 목적지는 `READY`다. `SERVO_ON`에서 나가는 전이는 없다.

명령 상태: `ACCEPTED`, `EXECUTING`, `COMPLETED`, `REJECTED`, `FAULTED`.

지원 명령 타입: `MOVE_JOINT`만. 단위는 radian, second다.

가상 전이표:

| 현재 | action | 다음 |
|---|---|---|
| DISCONNECTED | connect | CONNECTED |
| CONNECTED | disconnect | DISCONNECTED |
| CONNECTED | servo_on | READY |
| CONNECTED | fault | ERROR |
| READY | servo_off | SERVO_OFF |
| READY | disconnect | DISCONNECTED |
| READY | execute | RUNNING |
| READY | fault | ERROR |
| RUNNING | complete | READY |
| RUNNING | servo_off | SERVO_OFF |
| RUNNING | disconnect | DISCONNECTED |
| RUNNING | fault | ERROR |
| SERVO_OFF | servo_on | READY |
| SERVO_OFF | disconnect | DISCONNECTED |
| SERVO_OFF | fault | ERROR |
| ERROR | reset | CONNECTED |
| ERROR | disconnect | DISCONNECTED |

표에 없는 action은 `ControllerError("ILLEGAL_TRANSITION")`다. `submit`은 `READY`가 아니면 `CommandResult(REJECTED, ILLEGAL_STATE)`를 반환하고 예외를 내지 않는다. `RUNNING`이면 `BUSY`다.

`connect`는 로컬 상태만 바꾼다. 외부 프로세스나 PMAC 응답을 보지 않는다.

## Command, result, fault

`JointCommand`: `command_id`, `command_type`, `joint_targets`, `sequence`, `timestamp_s`, `robot_id`.

`CommandResult`: `command_id`, `result`, `error_code`.

`RobotState`에는 actual joint, target joint, joint error, TCP pose, `controller_state`, execution, fault가 같이 있다. Target과 actual은 별도 필드다.

`ControllerError`는 `code`와 message를 갖는다.

`FAULT_CODES`: `FAULT_INJECTED`, `COMMAND_INVALID`, `JOINT_LIMIT`, `INTERNAL_ERROR`, `TEST_FAULT`.

`inject_fault`는 이 집합만 받는다. fault 중 active command는 `FAULTED`로 끝내고 위치는 유지한다. `reset`은 `ERROR`에서 `CONNECTED`로 가고 actual joint를 target이나 zero로 바꾸지 않는다. `$$$`와 연결되어 있지 않다.

`step(dt)`는 project clock이다. `DISCONNECTED`에서 step하면 `ILLEGAL_STATE`다. 동작은 per-joint velocity slew이고, completion tolerance는 `1e-9` rad인 `PROJECT TEST PARAMETER`다.

거부 코드: `ILLEGAL_STATE`, `BUSY`, `UNSUPPORTED_COMMAND`, `DUPLICATE_COMMAND_ID`, `JOINT_COUNT`, `NON_FINITE`, `JOINT_LIMIT`.

Claim boundary: `PROJECT VIRTUAL CONTROLLER`, `NOT VERIFIED AGAINST REAL POWER PMAC`. 이 상태 기계는 Power PMAC servo state가 아니다.

## ExecutionGate 결합

`validator/gate.py`의 `ExecutionGate`는 생성자 타입이 `VirtualPmacController`다.

`consider`는 `validate`가 `SAFE_TO_ATTEMPT`일 때만 `controller.submit`을 호출한다. 그 외에는 `controller_submitted=False`이고 controller를 호출하지 않는다. submit 뒤에는 `controller.trace(command_id)`를 report에 넣는다.

같은 구체 타입을 만드는 곳:

| 위치 | 사용 |
|---|---|
| `digital_twin/session.py` | `VirtualPmacController`를 만들어 `ExecutionGate`에 넣는다 |
| `reliability/runner.py` | P-011 session이 같은 쌍을 만든다 |
| `hmi/actions.py` | `run_case` 경로로 P-008 gate와 P-009 monitor를 호출한다 |
| `ros2_adapter/node.py` | slew와 fault 전이는 `VirtualPmacController`에만 있다 |

Validator의 `SAFE_TO_ATTEMPT` / `REJECT` 기준은 controller 밖에 있다.

## 의존 테스트

| 테스트 | controller에 기대하는 것 |
|---|---|
| `tests/unit/test_p006_controller.py` | 가상 전이, `MOVE_JOINT` slew, fault reset이 joint를 보존, illegal command 거부 |
| `tests/unit/test_p007_convert.py` | ROS adapter evidence가 `VirtualPmacController(`를 호출하지 않음 |
| `tests/unit/test_p008_validator.py` | `ExecutionGate`가 unsafe command를 submit하지 않음 |
| `tests/unit/test_p008_fix2_acceptance.py` | 같은 gate 경계 |
| `tests/unit/test_p009_monitor.py` | monitor가 controller/validator 소스 밖에 있음 |
| `tests/unit/test_p010_hmi.py` | HMI가 gate report를 표시 |
| `tests/unit/test_p011_reliability.py` | 500-cycle workload가 가상 controller 위에서 유지 |

이 테스트가 보는 historical evidence 파일은 P-018에서 다시 쓰지 않는다.

## P-018 최소 변경 검토

core enum을 늘리지 않는다. `HEALTHY` / `FAILED`는 `PowerPmacControllerAdapter.communication_state`에만 둔다.

`get_state().controller_state`는 기존 어휘 안의 `DISCONNECTED` 또는 `CONNECTED`만 쓴다. 이 값은 통신 세션이 있는지만 뜻한다. `READY`, `RUNNING`, `SERVO_ON`, `SERVO_OFF`, `HOLD`로 PMAC servo를 표현하지 않는다.

`servo_on`, `servo_off`, `step`, `reset`, `execute_command`는 PMAC motor command를 보내지 않는다. motion이 필요한 호출은 거부한다.

`ExecutionGate`, `VirtualPmacController`, `IRobotController`, Validator는 수정하지 않는다. Adapter를 gate에 넣지 않는다. unsafe command가 PMAC으로 나가지 않는 기존 경계는 그대로다.

P-018에서 확인하는 PMAC 연산은 `P100` read/write와 `Sys.ServoCount` read뿐이다.
