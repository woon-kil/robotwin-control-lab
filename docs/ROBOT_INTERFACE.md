# Robot Interface

RoboTwin Control Lab — P-000B Canonical Robot API Contract

| 항목 | 값 |
| --- | --- |
| 문서 | Robot Interface |
| 기준선 | `62b9a624083cb6ac584e4eb6fd6a69c8c56197ee` |
| 현재 Gate | P-000B |
| 구현 | NOT STARTED |
| 작성일 | 2026-10-23 |

이 문서는 Canonical Robot API의 논리 계약이다. 프로그래밍 언어와 serialization format은 정하지 않는다. Python, C#, C++ 인터페이스 파일을 만들지 않는다.

좌표와 단위는 `docs/COORDINATE_CONVENTION.md`를 따른다. 포즈의 저장 표현은 Quaternion이다.

## 1. Robot State

최소 개념은 다음이다.

```text
RobotState

timestamp
robot_id
controller_state

joint_position
joint_velocity

tcp_pose

command_id
execution_state

error_code
```

필드의 정확한 자료형은 이후 구현 게이트에서 정할 수 있다. 이 게이트는 이름과 역할만 고정한다.

- `timestamp`: 상태가 관측된 시각. 단위는 second다. epoch와 시계 종류는 정하지 않는다.
- `robot_id`: 상태를 낸 로봇의 식별자.
- `controller_state`: 3절의 상태 어휘 중 하나.
- `joint_position`: 관절 위치. 각 관절의 단위는 radian이다.
- `joint_velocity`: 관절 속도. 단위는 radian/second다.
- `tcp_pose`: `tcp` 프레임의 포즈. 위치는 meter, 자세는 Quaternion이다. 어느 프레임 기준인지는 포즈와 함께 프레임 이름으로 적는다. 이름을 생략한 포즈는 상태가 아니다.
- `command_id`: 이 관측이 연결되는 명령의 식별자. 명령과 Feedback은 서로 다른 기록이다.
- `execution_state`: 그 명령에 대해 컨트롤러가 보고한 진행 상태의 자리다. 허용 단어 목록은 이 게이트에서 더 만들지 않는다.
- `error_code`: 오류의 식별자. 오류가 없을 때 빈 값을 성공 코드로 추정해 채우지 않는다.

Canonical Robot State의 정의자는 Core다. Unity, Isaac Sim, ROS2, PMAC의 네이티브 상태 객체는 이 필드로 바뀐 뒤에만 Core 상태가 된다.

## 2. Canonical Robot Command

최소 어휘는 REQ-006과 같다.

```text
SERVO_ON
SERVO_OFF
HOME

MOVE_JOINT
MOVE_TCP

HOLD
STOP
RESET
ESTOP
```

동작 명령은 HOME, MOVE_JOINT, MOVE_TCP다. 동작 명령은 최소 다음 개념을 표현할 수 있어야 한다.

```text
command_id
robot_id
command_type

target

velocity_scale
acceleration_scale
```

`target`은 명령 종류에 따라 관절 목표 또는 TCP 목표다. TCP 목표는 프레임 이름을 포함한다. `velocity_scale`과 `acceleration_scale`은 비율 개념이며, 물리 단위의 최대 속도 한도를 이 문서에서 정하지 않는다.

이 목록은 문서 예시다. 구현 schema로 고정하지 않는다. 필드 타입, 단위 이외의 인코딩, 식별자 형식은 정하지 않는다.

SERVO_ON, SERVO_OFF, HOLD, STOP, RESET, ESTOP도 `command_id`, `robot_id`, `command_type`을 갖는다. 세 동작 명령에만 필요한 `target`과 scale을 나머지 명령의 필수 값으로 만들지 않는다.

## 3. Task-Level Command

상위 계층의 명령은 다음이다.

```text
HANDLE_FILTER
PICK_OBJECT
PLACE_OBJECT
```

이 셋은 Canonical Robot Command가 아니다. Task Decomposer를 거쳐 Canonical 명령이 된다.

```text
HANDLE_FILTER
      ↓
Task Decomposer
      ↓
MOVE_TCP / tool action / MOVE_TCP ...
```

`tool action`은 분해 그림의 자리다. REQ-006의 어휘에 명령을 추가하지 않는다.

이 계층을 특정 회사의 실제 컨트롤러 명령이라고 주장하지 않는다. Core Robotics 또는 PMAC의 Task Protocol이 아니다. command schema와 코드는 만들지 않는다.

## 4. Controller abstraction

논리 인터페이스의 이름은 `IRobotController`다.

최소 책임은 다음이다.

```text
connect
disconnect

servo_on
servo_off

home

execute_command

hold
stop
reset

get_state
```

이것은 개념 인터페이스다. 메서드의 언어별 시그니처, 예외 타입, 전송 형식은 만들지 않는다.

`execute_command`는 Canonical 명령을 어댑터에 맡기는 자리다. HOME, MOVE_JOINT, MOVE_TCP는 Pre-Execution Validation의 전체 결과가 PASS일 때만 이 인터페이스의 실행으로 넘긴다. PASS는 실행을 허용할 수 있다는 내부 판정이다.

`get_state`의 결과는 1절의 RobotState 개념이다. 어댑터의 네이티브 객체를 그대로 반환하지 않는다.

## 5. Controller state model

최소 상태 어휘는 REQ-011과 같다.

```text
DISCONNECTED
CONNECTED

SERVO_OFF
SERVO_ON

HOMING
READY

RUNNING
HOLD

ERROR
ESTOP
```

한 시점의 `controller_state`는 이 목록의 하나다. 목록 밖 이름으로 실패를 표현하지 않는다.

이 게이트는 완전한 전이표를 잠그지 않는다. PMAC state machine이라고 주장하지 않는다. 허용과 금지의 원칙만 둔다.

허용 원칙:

- 연결 동작은 DISCONNECTED에서 떠나는 방향으로만 이야기한다. 연결되었다는 사실만으로 SERVO_ON이나 READY가 되지 않는다.
- HOME, MOVE_JOINT, MOVE_TCP를 시작하려는 상태 이름은 READY다. 그 명령이 진행 중이면 RUNNING이다. HOMING은 HOME이 진행 중인 상태 이름이다.
- HOLD는 RUNNING에서 동작을 붙잡는 상태다. HOLD는 완료도 READY도 아니다.

금지 원칙:

- DISCONNECTED, CONNECTED, SERVO_OFF, ERROR, ESTOP, HOLD에서 HOME, MOVE_JOINT, MOVE_TCP를 시작하지 않는다.
- ERROR와 ESTOP을 READY, RUNNING, 검증 PASS로 저장하거나 표시하지 않는다.
- MOVE_JOINT, MOVE_TCP, HOME, HOLD, STOP은 ESTOP을 해제하지 않는다.
- RESET만으로 SERVO_ON, READY, 검증 PASS가 되지 않는다.
- SERVO_OFF는 진행 중인 동작의 성공 완료가 아니다.
- 위에 적지 않은 전이는 미정이다. 미정 전이를 허용된 전이로 구현하지 않는다. 완전한 표는 이후 게이트가 명시하기 전에는 없다.

## 6. PMAC boundary

```text
IRobotController
        │
        ▼
PMAC Adapter
        │
        ├─ Virtual PMAC-compatible Controller
        │
        └─ Real PMAC Adapter [Future / Conditional]
```

실제 PMAC protocol을 알려진 것으로 가정하지 않는다. P-000B는 이 경계만 정의한다.

금지:

- register address 추정
- command syntax 추정
- memory map 추정
- motion program syntax 추정
- proprietary state 추정
- Core Robotics-specific protocol 추정

Virtual PMAC-compatible Controller는 통신과 5절의 상태 어휘를 다루는 자리다. Real PMAC Adapter는 공개적으로 검증 가능한 정보가 생기기 전에는 구현 대상이 아니다. 그 정보가 생겨도 Canonical 어휘를 바꾸지 않는다.

## 7. Pre-execution validation contract

Validation Engine의 개별 결과는 다음 셋 중 하나다.

```text
PASS
FAIL
NOT_EVALUATED
```

각 validator는 최소 다음 개념을 제공한다.

```text
validator_id
status
reason
measured_value
threshold
unit
evidence_reference
```

아직 측정하지 않은 값과 threshold를 만들지 않는다. 값이 없으면 그 필드는 비어 있고 status는 NOT_EVALUATED다. 빈 값을 0이나 합격 한도로 두지 않는다.

검사 이름은 Reachability, IK Feasibility, Joint Limit, Collision, Self Collision, Safety Zone, Trajectory, TCP Target, Physics다. Physics는 적용할 때만 의미가 있다. 출처가 없는 물리 파라미터는 PASS의 근거가 되지 못하며, 그 검사는 NOT_EVALUATED로 남는다.

## 8. Validation aggregation

전체 실행 가능 여부는 개별 결과로 정한다.

```text
Required Validator FAIL
→ Overall FAIL
→ execution blocked
```

```text
Required Validator NOT_EVALUATED
→ Overall PASS 금지
```

어떤 validator가 required인지는 Task와 Phase가 명시해야 한다. P-000B는 그 목록을 정하지 않는다. required 목록이 없는 작업에는 Overall PASS를 주지 않는다.

required가 아닌 검사의 FAIL을 막을지 여부는 정하지 않았다. 그 공백을 무시 규칙으로 채우지 않는다.

Overall PASS여도 산업 안전 인증이 아니다.
