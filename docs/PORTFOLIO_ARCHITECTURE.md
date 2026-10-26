# Portfolio Architecture

이 문서는 면접용 요약이다. `docs/ARCHITECTURE.md`의 문장과 번호를 바꾸지 않는다. 계약과 구현 범위가 다르면, 계약은 그대로 두고 구현된 것만 말한다.

## Frozen contract

`docs/ARCHITECTURE.md`는 `IRobotController` 아래에 Unity, Isaac, ROS 2, PMAC adapter 자리를 둔다. Operator 요청은 Pre-Execution Validator를 통과한 뒤에만 controller로 간다. FAIL은 REJECT이며 그 명령의 controller 실행으로 이어지지 않는다. Feedback은 명령을 보냈다는 기록이 아니다.

## Implemented path

현재 코드의 실행 경로는 다음이다.

```text
RobotCommand
    ↓
validator.gate.ExecutionGate
    ↓
REJECT  →  controller.submit 없음, tracking 없음
SAFE_TO_ATTEMPT
    ↓
controller.virtual_pmac.VirtualPmacController
    ↓
Target joint 와 Actual joint 를 분리 기록
    ↓
digital_twin.monitor.DigitalTwinMonitor
    ↓
hmi.view_model
```

`IRobotController`는 `controller/virtual_pmac.py`의 Protocol이다. Employment MVP의 실행 motion backend는 `VirtualPmacController`다. `ExecutionGate`는 `submit`과 `trace`를 호출하고, `PowerPmacControllerAdapter`를 호출하지 않는다.

`ros2_adapter.Ros2ControllerAdapter`는 ROS 2 Jazzy transport다. Canonical command를 받아 같은 virtual controller로 전달하고 target과 actual을 분리해 publish한다. 별도의 실물 모터 backend가 아니다.

`PowerPmacControllerAdapter`는 Official Power PMAC Simulator 1.2.1용 두 번째 backend다. 경로는 SSH와 `gpascii`이고, 검증된 motion 범위는 software virtual `Motor[1]`이다. `execute_command`는 PMAC motor command를 보내지 않는다. Motor[1]은 UR10e joint가 아니고, PMAC motor unit은 radian이 아니다. `robot_joint_mapping`과 `unit_conversion`은 `NOT_DEFINED`다.

Unity 프로젝트와 Isaac Sim script는 nominal UR10e와 workcell을 독립적으로 실행한다. 둘 다 `IRobotController` 구현이 아니다. Cross-sim 수치는 그 두 scene의 TCP를 독립 FK와 비교한 PROJECT TEST다.

Real Power PMAC hardware adapter, real robot adapter, RL policy, domain randomization은 없다.

## Vocabulary that stays

```text
PROJECT TEST
Golden Reference
Virtual Controller
PMAC-compatible semantics
Digital Twin Tracking
Pre-Execution Validation
500-cycle deterministic project workload
```

`SAFE_TO_ATTEMPT`는 이 경로의 실행 전 통과다. Certified safety로 읽지 않는다.
