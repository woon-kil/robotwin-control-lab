# Requirements

RoboTwin Control Lab — P-001A.1 Controlled Requirements Amendment

| 항목 | 값 |
| --- | --- |
| 문서 | Requirements Baseline |
| P-000A baseline | `bf876d22abdfb567b800ad49c966f27e4c5d7b92` |
| P-001A source pin | `3b94edbc21c454b580468bd6cabbaa9204f70b50` |
| Use case hold | `975a01c5875214976a0f5ac58d4545df7fd90091` |
| 현재 Gate | P-001A.1 — Controlled Requirements Amendment |
| 구현 | NOT STARTED |
| 작성일 | 2026-10-23 |
| 관련 문서 | `docs/PROJECT_CHARTER.md` |

요구사항 ID는 `REQ-XXX`와 `NFR-XXX`를 사용한다. 이 문서는 검증 가능한 문장만 고정한다. 테스트 코드, 증거 파일, Phase 일정은 만들지 않으며 추적 행렬에서 `TBD`로 둔다.

P-000A.1은 REQ-001–REQ-018과 NFR-001–NFR-007의 문장을 바꾸지 않고 REQ-019–REQ-022를 추가했다. P-001A.1은 REQ-001–REQ-022와 NFR-001–NFR-007을 삭제하거나 번호를 다시 매기지 않는다. REQ-001–REQ-018의 문장은 유지한다. REQ-019–REQ-021은 대표 Use Case를 구분하는 데 필요한 문장만 고친다. REQ-023–REQ-028을 추가한다. 이 개정은 구현 Phase가 아니다. P-001B를 시작하지 않는다.

숫자 목표는 4절의 초기 Engineering Target이다. 최종 보증값이 아니다.

## 1. Functional Requirements

### REQ-001 — Canonical Robot Model

시뮬레이터에 종속되지 않는 로봇 모델 규약을 정의할 수 있어야 한다.

규약은 최소한 로봇 식별, 링크/관절 계층, 관절 축과 한계, 단위, 베이스 프레임을 담아 Unity, Isaac Sim, 독립 기구학이 같은 설명을 읽게 해야 한다. 규약의 소유자는 Control / Validation Core 쪽이며, Unity API, Isaac API, ROS2 메시지 정의가 규약 자체가 되어서는 안 된다.

P-000A는 파일 형식과 로봇 소스 버전을 선택하지 않는다. 소스 확정은 P-001 시작 전 Source Baseline Gate의 일이다.

### REQ-002 — Unity Robot

동일한 로봇 설명을 기반으로 Unity에서 로봇을 구성할 수 있어야 한다.

Unity 씬에만 존재하는 별도 관절 계층을 기준 모델로 사용하지 않는다. 임포터나 씬 설정이 설명을 바꾸면 그 차이를 기록한다.

### REQ-003 — Isaac Robot

동일한 로봇 설명을 기반으로 Isaac Sim에서 로봇을 구성할 수 있어야 한다.

Isaac Sim에만 있는 관절 이름, 축, 관성 기본값을 기준 모델로 사용하지 않는다. REQ-002와 같은 설명에서 출발해야 한다.

### REQ-004 — Independent FK

시뮬레이터 API와 독립된 FK 구현으로 TCP Pose를 계산할 수 있어야 한다.

FK는 Unity 또는 Isaac Sim의 포즈 조회 결과를 재포장한 값이 아니어야 한다. 입력은 캐노니컬 모델과 관절 위치이고, 출력은 TCP 위치와 자세이다. 비교 기준은 4절의 FK Engineering Target을 사용한다. 그 수치는 보증값이 아니다.

### REQ-005 — IK

Target TCP Pose로부터 Joint target을 계산할 수 있어야 한다.

IK 결과는 관절 목표이며, 그 목표를 다시 독립 FK에 넣었을 때의 TCP와 목표 TCP의 관계는 측정 가능해야 한다. 해가 없거나 한계를 넘는 경우는 실패로 보고한다. 실패를 가장 가까운 성공 포즈로 바꾸어 PASS로 기록하지 않는다.

관절 목표 오차의 초기 기준은 4절의 Joint Target Error이다.

### REQ-006 — Canonical Robot Command

공통 Robot Command를 정의해야 한다.

최소 명령:

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

HMI, 테스트, ROS2 어댑터, 가상 컨트롤러는 이 명령 집합을 통해 로봇에 요청한다. 시뮬레이터별 함수 이름을 상위 명령으로 올리지 않는다. 명령마다 식별자, 발행 시각, 파라미터, 완료 조건을 나중에 추적할 수 있어야 한다. 파라미터 스키마는 이 기준선에서 확정하지 않는다.

### REQ-007 — Controller Abstraction

다음과 같은 컨트롤러 추상화를 지원해야 한다.

```text
IRobotController
```

Control / Validation Core는 이 추상화에 의존한다. Unity, Isaac Sim, ROS2, PMAC-compatible Virtual Controller는 각각의 어댑터가 추상화를 구현한다. 코어가 특정 시뮬레이터 SDK를 직접 호출하는 구조는 이 요구사항을 만족하지 않는다.

### REQ-008 — Cross-Simulator Validation

동일한 명령에 대해 Unity, Isaac Sim, 독립 FK 결과를 비교할 수 있어야 한다.

비교 대상에는 TCP 위치가 포함된다. 초기 기준은 4절의 Cross-Simulator TCP Position Error이다. 비교 기록에는 명령 식별자, 모델 식별자, 좌표계, 단위가 포함되어야 한다. 셋 중 하나의 출력이 없으면 그 실행은 비교 PASS가 될 수 없다.

### REQ-009 — Physics Validation

다음을 측정할 수 있어야 한다.

- gravity
- mass
- inertia
- payload
- friction
- collision

측정값은 사용한 파라미터와 그 출처를 함께 남긴다. 엔진이 채워 넣은 기본값을 측정 대상으로 쓸 때는 기본값임을 표시한다. 기대하지 않은 충돌의 초기 기준은 4절의 Unexpected Collision이다.

### REQ-010 — Virtual PMAC-compatible Controller

실제 PMAC 하드웨어가 없어도 컨트롤러 통신과 상태 전이를 검증할 수 있어야 한다.

이 요구사항의 범위는 추상화와 가상 컨트롤러이다. 적용 가정은 3절과 같다. 비공개 프로토콜, 레지스터 맵, FilterBot 내부 제어 변수를 재현할 것을 요구하지 않는다. 공개적으로 검증 가능한 정보가 생기기 전에는 그 정보를 추정해 채우지 않는다.

### REQ-011 — Controller State Machine

최소 다음 상태를 표현한다.

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

현재 상태는 한 시점에 하나로 보고할 수 있어야 한다. ERROR와 ESTOP은 정상 동작 상태로 대체해 숨기지 않는다. 상태 전이표는 P-000A에서 확정하지 않는다. 구현 단계에서 전이를 문서화하며, 위 목록에 없는 상태로 실패를 표현하지 않는다.

### REQ-012 — Command / Feedback

Command와 실제 또는 가상 컨트롤러의 Feedback을 분리해 처리한다.

Command는 요청이다. Feedback은 컨트롤러가 보고한 상태, 관절, 오류, 완료 여부이다. Command 로그를 Feedback의 대용으로 사용하지 않는다. Digital Twin은 Feedback을 시뮬레이터 상태에 반영하는 REQ-014를 따른다.

### REQ-013 — ROS2 Integration

로봇 Command와 State를 ROS2를 통해 전달할 수 있는 어댑터 구조를 제공한다.

ROS2 메시지 정의가 Canonical Robot Command를 대체하지 않는다. 어댑터는 REQ-006 명령과 REQ-011 상태, REQ-012 피드백을 ROS2 경계 밖으로 옮긴다. P-000A는 ROS2 패키지를 생성하지 않는다.

### REQ-014 — Digital Twin

컨트롤러 Feedback을 시뮬레이터 로봇 상태에 반영할 수 있어야 한다.

반영 경로는 Feedback에서 시뮬레이터 상태로 향한다. 시뮬레이터가 먼저 이동한 결과를 컨트롤러 Feedback인 것처럼 기록하지 않는다. Unity와 Isaac Sim 모두 이 반영 경로의 대상이 될 수 있다. 어느 한쪽 화면을 다른 쪽의 뷰어로 두는 방식은 이 요구사항의 구현이 아니다.

### REQ-015 — Vision Coordinate Pipeline

다음 좌표 변환 파이프라인을 지원한다.

```text
Camera
 ↓
World
 ↓
Robot Base
 ↓
TCP Target
```

각 단계에서 사용한 프레임 이름과 변환을 추적할 수 있어야 한다. 파이프라인의 출력은 TCP Target이며, 그 목표는 REQ-006의 `MOVE_TCP` 입력으로 전달될 수 있어야 한다. 복잡한 AI 비전 모델은 MVP 필수 범위가 아니다. 가상 비전은 이 좌표 파이프라인의 입력 소스가 될 수 있다.

### REQ-016 — HMI

최소 다음 상태를 확인할 수 있어야 한다.

- Controller state
- Joint state
- TCP pose
- Command state
- Simulator state
- Collision
- Errors
- Communication latency

HMI의 조회와 명령은 Canonical Robot API를 통한다. HMI가 Unity API 또는 Isaac API를 직접 제어 경로로 사용하지 않는다. 통신 지연은 값이 있을 때 표시하고, 값이 없으면 측정되지 않음으로 표시한다. 빈 지연을 0으로 보여 주지 않는다.

### REQ-017 — Telemetry

정량적인 실행 데이터를 기록할 수 있어야 한다.

기록에는 명령, 피드백, 오류, 타임스텝, 모델 식별자가 포함된다. 통신 지연은 임계값 판정 전에 다음 통계로 남긴다.

```text
Mean
P50
P95
P99
Max
```

Latency의 PASS 임계값은 이 기준선에서 정하지 않는다. 측정 결과와 실제 아키텍처를 확인한 뒤에 확정한다. 통계를 계산할 수 없는 실행은 표본 수와 함께 미측정으로 남긴다.

### REQ-018 — Reliability Test

반복 동작 테스트를 자동으로 실행할 수 있어야 한다.

목표 반복 횟수:

```text
500 cycles
```

이 횟수는 4절의 초기 Engineering Target이다. 자동 실행은 사이클 번호, 명령, 종료 상태, 오류, 충돌을 남긴다. 중간 실패가 있는 실행을 500회 도달로 집계하지 않는다. 실패 사이클은 원인과 함께 증거에 포함한다.

### REQ-019 — Workcell Simulation

로봇 단독이 아니라 다음 관계를 검증할 수 있는 Workcell을 표현한다.

```text
Robot + Environment + Task
```

Workcell은 최소한 다음 개념을 표현할 수 있어야 한다.

- Robot Base
- Work Object
- Task Target
- Destination / Disposal Area
- Obstacles
- Restricted Zones
- Virtual Camera
- Lighting

도메인 동기는 위험하거나 원격인 산업설비 유지보수 작업이다. P-000A.1이 일반화 예시로 적었던 다음 문장은 그 동기다. MVP의 대표 구현 시나리오가 아니다.

```text
Hazardous Filter Handling Simulation
```

```text
The workcell is a generic hazardous filter-handling simulation inspired by industrial remote-manipulation tasks. It is not a replica of Core Robotics FilterBot or any proprietary robot system.
```

대표 포트폴리오 Use Case는 REQ-023의 Bolt Fastening / Loosening이다. 필터 전체 교체, 커버 취급, 필터 운반은 MVP 구현 범위가 아니다. 특정 회사 제품의 전체 공정을 복제하지 않는다.

실제 FilterBot의 다음 항목을 추정해서는 안 된다.

- Link Length
- DH Parameter
- Joint Range
- Mass
- Inertia
- Payload
- Controller Protocol
- Motor Specification
- Safety Logic

P-000A.1은 Workcell 모델과 시나리오 자산을 만들지 않는다.

### REQ-020 — Task-Level Command

REQ-006의 Canonical Robot Command 위에 산업 작업 단위의 명령 계층을 정의한다.

Generic 후보:

```text
HANDLE_FILTER
PICK_OBJECT
PLACE_OBJECT
```

이 셋은 generic task architecture에 남긴다. `HANDLE_FILTER`는 필터 전체 공정을 MVP에서 구현하라는 명령이 아니다.

대표 Use Case 명령:

```text
BOLT_FASTEN
BOLT_LOOSEN
```

이 둘은 RoboTwin 내부의 canonical task-level abstraction이다. Proprietary controller command가 아니다. Core Robotics 또는 PMAC의 실제 Task Protocol이 아니다. 이후 lower-level motion/process command로 분해된다. 개념 흐름은 REQ-027이다.

Task-Level Command는 Motor나 Joint를 직접 제어하지 않는다.

```text
Task-Level Command
        ↓
Task Decomposition
        ↓
Canonical Robot Command
        ↓
Motion / Validation
        ↓
Controller Interface
```

예:

```text
HANDLE_FILTER
   ↓
MOVE_TCP approach
   ↓
MOVE_TCP grasp
   ↓
tool action
   ↓
MOVE_TCP extract
   ↓
MOVE_TCP destination
   ↓
tool release
```

이 예의 `tool action`과 `tool release`는 분해 흐름의 자리다. REQ-006의 명령 목록에 이름을 추가하지 않는다. command schema와 코드는 이 게이트에서 만들지 않는다. 이 계층을 Core Robotics 또는 PMAC의 실제 Task Protocol이라고 주장하지 않는다.

### REQ-021 — Pre-Execution Validation

Controller에 명령을 전달하기 전에 Digital Twin에서 실행 가능성과 위험 요소를 검사한다.

```text
Task / Robot Command
        ↓
Pre-Execution Validator
        ↓
+---------------------------+
| Reachability              |
| IK Feasibility            |
| Joint Limits              |
| Collision                 |
| Self Collision            |
| Safety Zone               |
| Trajectory                |
| TCP Target                |
| Physics when applicable   |
+---------------------------+
        ↓
     PASS / FAIL
       /     \
      /       \
   FAIL       PASS
    ↓           ↓
 REJECT     APPROVE
                ↓
             EXECUTE
                ↓
           Controller
```

최소 검사는 다음과 같다.

#### Reachability

목표 TCP pose가 robot workspace 안에서 실현 가능한지 검사한다.

#### IK Feasibility

목표 pose에 대해 유효한 joint solution이 존재하는지 검사한다.

#### Joint Limit

계산된 trajectory가 정의된 joint limit을 위반하는지 검사한다.

#### Collision

Robot과 Environment의 collision 가능성을 검사한다.

#### Self Collision

Robot link 간 충돌 가능성을 검사한다.

#### Safety Zone

작업 경로가 정의된 restricted zone 정책을 위반하는지 검사한다. Zone의 종류는 REQ-022이다.

#### Trajectory

Trajectory가 정의된 motion constraints를 만족하는지 검사한다.

#### TCP Target

최종 TCP pose가 target tolerance를 만족하는지 검사한다.

#### Physics

필요한 경우 payload, gravity, inertia처럼 모델링된 물리 조건을 이용해 검증한다. 출처가 없는 물리 파라미터는 PASS 근거로 사용하지 않는다.

실행 정책:

```text
Validation FAIL
→ Controller execution prohibited

Validation PASS
→ execution may be approved
```

PASS는 실제 산업 안전 인증이 아니다. RoboTwin 내부의 engineering validation result이다.

이 요구사항은 REQ-004, REQ-005, REQ-006, REQ-009의 문장을 바꾸지 않는다. 그 검사들을 실행 전 관문으로 묶는다. P-000A.1은 validator를 구현하지 않는다.

`BOLT_FASTEN`과 `BOLT_LOOSEN`에서 이 절의 전체 PASS는 프로세스 완료가 아니다. 그 결과 이름은 REQ-026의 `SAFE_TO_ATTEMPT`다. 전체 FAIL의 결과 이름은 `REJECTED`다. 체결 또는 해체 완료는 REQ-027의 Execution 결과다. 이 절의 그림에서 APPROVE는 작업자 승인이며, 승인 없이 컨트롤러 실행을 시작하지 않는다.

### REQ-022 — Safety Zone

Workcell에 다음 세 가지 논리적 Safety Zone을 정의할 수 있어야 한다.

```text
NORMAL
WARNING
PROHIBITED
```

#### NORMAL

정상 작업 허용 영역.

#### WARNING

위험 가능성이 있는 영역. 향후 구현에서는 warning 표시 또는 reduced-velocity policy의 입력으로 사용할 수 있다.

#### PROHIBITED

작업 경로가 진입해서는 안 되는 영역. Pre-Execution Validation에서 trajectory reject 조건으로 사용할 수 있다.

이 세 상태는 RoboTwin의 시뮬레이션 정책 모델이다. 실제 산업 안전 규격의 준수나 인증 기능이라고 주장하지 않는다. REQ-011의 컨트롤러 상태와 다른 축이다. P-000A.1은 Zone의 기하와 정책을 구현하지 않는다.

### REQ-023 — Representative Use Case

플랫폼의 정체는 Industrial Robot Digital Twin & Pre-Execution Validation Platform이다. Bolt 작업은 그 플랫폼의 대표 Use Case다. 프로젝트 전체를 Bolt 전용 프로그램으로 바꾸지 않는다.

```text
Robot Control Validation Platform
        ↓
Generic Task Architecture
        ↓
Representative Use Case
        ↓
Bolt Fastening / Loosening
```

대표 Use Case의 이름은 다음이다.

```text
Generic Industrial Bolt Fastening / Loosening with Torque-Aware Validation
```

핵심 질문은 둘이다.

```text
이 로봇이 지정된 볼트 작업을 안전하게 시도할 수 있는가?
```

```text
체결/해체 과정이 정의된 Torque/Angle 조건에 따라 정상적으로 완료되었는가?
```

첫 질문은 Pre-Execution이다. 둘째 질문은 Execution이다. 둘을 한 판정으로 합치지 않는다.

도메인 동기는 위험하거나 원격인 산업설비 유지보수다. 표현은 다음 범위를 넘지 않는다.

```text
Generic Industrial Maintenance Workcell inspired by publicly described remote maintenance / bolt-operation domain
```

금지하는 주장:

- 실제 FilterBot 구조를 복제했다
- 실제 FilterBot End-Effector다
- 실제 FilterBot DH를 사용한다
- 실제 FilterBot Torque를 사용한다
- 실제 FilterBot 작업 sequence를 사용한다
- 실제 FilterBot control algorithm을 사용한다
- 실제 PMAC proprietary protocol을 사용한다

이 요구사항은 시나리오 자산과 공구 모델을 만들지 않는다.

### REQ-024 — Bolt Task Commands

대표 Task-Level Command는 다음 둘이다.

```text
BOLT_FASTEN
BOLT_LOOSEN
```

이 명령은 REQ-006의 Canonical Robot Command가 아니다. Proprietary controller command가 아니다. Task Decomposition을 거쳐 lower-level motion/process command가 된다. 분해 스키마와 코드는 이 게이트에서 만들지 않는다.

`HANDLE_FILTER`, `PICK_OBJECT`, `PLACE_OBJECT`는 REQ-020의 generic 후보로 남는다. MVP는 그 명령들로 필터 교체 공정을 구현하지 않는다.

### REQ-025 — Bolt Tool and Tool TCP

MVP의 대표 End-Effector는 Bolt Fastening / Loosening Tool 하나다.

개념적 frame chain:

```text
robot flange
     ↓
bolt tool
     ↓
bolt_tool_tcp
```

Bolt Tool은 로봇 본체와 따로 다음을 가질 수 있어야 한다.

- TCP
- geometry
- orientation
- collision representation

`bolt_tool_tcp`는 `flange`와 다른 프레임이다. `tool0`과도 같은 프레임으로 두지 않는다. 공구 길이, 자세, 충돌 geometry의 숫자는 이 개정에 없다. 특정 제품의 Tool dimensions를 추정하지 않는다.

MVP에서 제외한다.

- Filter Gripper
- Cover Handling Tool
- Filter Transport Tool
- Inspection Tool
- Automatic Tool Changer implementation

이후 architecture는 tool extension을 허용할 수 있다. 이 게이트는 multi-tool framework를 구현하지 않는다.

### REQ-026 — Bolt Pre-Execution Result

`BOLT_FASTEN`과 `BOLT_LOOSEN`의 Pre-Execution 결과는 다음 둘 중 하나다.

```text
SAFE_TO_ATTEMPT
REJECTED
```

`SAFE_TO_ATTEMPT`는 작업을 시도할 조건이 갖춰졌다는 뜻이다. 다음을 의미하지 않는다.

- `FASTENED`
- `LOOSENED`
- 실제 작업 성공 보장
- 산업 안전 인증
- 실제 체결 Torque 보장

REQUIRED validator는 다음 전부다. OPTIONAL validator는 이 개정에서 만들지 않는다.

| validator | 구분 |
| --- | --- |
| Reachability | REQUIRED |
| IK Feasibility | REQUIRED |
| Joint Limits | REQUIRED |
| Robot Collision | REQUIRED |
| Bolt Tool Collision | REQUIRED |
| Safety Zone | REQUIRED |
| Bolt Tool TCP Alignment | REQUIRED |
| Approach Trajectory | REQUIRED |
| Task / Tool Compatibility | REQUIRED |

Robot Collision은 로봇 몸과 링크 사이 간섭을 포함한다. 그 검사가 비어 있으면 Robot Collision은 NOT_EVALUATED다. Bolt Tool Collision은 공구 geometry의 충돌이다. 둘을 한 검사로 합쳐 숨기지 않는다.

Task / Tool Compatibility는 명령이 `BOLT_FASTEN` 또는 `BOLT_LOOSEN`이고, 선택된 공구가 Bolt Fastening / Loosening Tool일 때 성립한다. 다른 공구는 MVP에 없으므로 그 조합은 FAIL이다.

REQUIRED 중 하나라도 FAIL이면 결과는 `REJECTED`다. Controller execution을 허용하지 않는다. REQUIRED 중 하나라도 NOT_EVALUATED이면 `SAFE_TO_ATTEMPT`를 주지 않는다. 이 집계는 `docs/ROBOT_INTERFACE.md` 8절과 같다.

Torque 한계는 이 REQUIRED 목록에 없다. Torque는 REQ-028의 Execution feedback이다.

`SAFE_TO_ATTEMPT`인 경우에만 작업자의 `APPROVE & EXECUTE`가 가능하다. 승인 전에 Controller execution을 시작하지 않는다. 승인은 REQ-006 명령이 아니다.

STOP과 ESTOP의 protective/termination 의미는 P-000B 결정을 유지한다. 그 명령은 이 절의 승인을 기다리지 않는다.

대표 Failure Demo는 문서 수준의 요구다. 구현과 측정은 이 게이트의 일이 아니다.

```text
BOLT_FASTEN
       ↓
SIMULATE
       ↓
Reachability      PASS
IK                PASS
Joint Limit       PASS
Tool Collision    FAIL
       ↓
REJECTED
       ↓
EXECUTION BLOCKED
```

이 흐름은 이후 포트폴리오의 대표 Failure Evidence로 쓸 수 있다. 이 게이트는 그 Evidence 파일을 만들지 않는다.

### REQ-027 — Bolt Execution Process

Bolt 작업의 실행 과정은 Generic Fastening Process abstraction이다. 특정 FilterBot controller state machine의 복제가 아니다. REQ-011의 controller state와 다른 축이다.

개념 state:

```text
APPROACH
ALIGN
ENGAGE
ROTATE
COMPLETE
RETRACT
```

Failure와 Fault로의 이탈을 허용한다. 이 게이트는 완전한 전이표를 잠그지 않는다. 잠그지 않은 전이를 허용된 전이로 구현하지 않는다.

Execution 결과는 Pre-Execution 결과와 분리한다.

```text
FASTENED
LOOSENED
FAILED
FAULT
```

다음 조합이 모두 가능해야 한다.

```text
Pre-Execution: SAFE_TO_ATTEMPT
Execution: FASTENED
```

```text
Pre-Execution: SAFE_TO_ATTEMPT
Execution: FAILED
```

`SAFE_TO_ATTEMPT`만으로 `FASTENED`나 `LOOSENED`를 기록하지 않는다. `REJECTED`인 작업은 Execution result를 성공으로 남기지 않는다.

대표 정상 흐름은 문서 수준의 요구다.

```text
BOLT_FASTEN
       ↓
Bolt Target Pose
       ↓
Bolt Tool TCP
       ↓
SIMULATE
       ↓
Reachability
IK
Joint Limit
Robot / Tool Collision
Safety Zone
Tool TCP Alignment
Approach Trajectory
       ↓
SAFE_TO_ATTEMPT
       ↓
Operator Approval
       ↓
APPROACH
ALIGN
ENGAGE
ROTATE
       ↓
Virtual Torque / Angle Feedback
       ↓
FASTENED
       ↓
RETRACT
       ↓
Evidence
```

`BOLT_LOOSEN`의 완료 결과 이름은 `LOOSENED`다. 나머지 관문은 같다. 이 게이트는 이 흐름을 실행하지 않는다.

### REQ-028 — Virtual Torque and Angle Evidence

Execution 단계에서 다음 feedback을 사용할 수 있어야 한다.

- Rotation Angle
- Rotation Speed
- Torque Feedback
- Contact State
- Tool State
- Controller State

이 feedback은 Pre-Execution 결과가 아니다.

실제 산업용 체결 Tool과 Force/Torque Sensor가 있다는 전제를 두지 않는다. 초기 구현은 Virtual Fastening Process와 Virtual Torque Sensor를 사용할 수 있다. HMI와 Evidence는 simulated/virtual임을 식별할 수 있어야 한다. 실제 센서 측정값으로 표현하지 않는다.

실제 FilterBot이나 특정 산업설비의 Torque 값을 추정하지 않는다. 시험에 숫자가 필요하면 그 숫자를 `SIMULATION TEST PARAMETER` 또는 `PROJECT TEST PARAMETER`로 정의한다. 실제 장비 specification이라고 적지 않는다. 이 개정은 Torque 숫자를 만들지 않는다.

이후 Execution Evidence는 다음을 저장할 수 있어야 한다.

- Target Torque
- Peak Torque
- Final Torque
- Rotation Angle
- Torque-Angle history
- Completion State
- Fault State
- Timestamp

측정하지 않은 값을 PASS evidence로 만들지 않는다. 이 게이트는 숫자와 PASS evidence를 생성하지 않는다.

## 2. Non-Functional Requirements

### NFR-001 — Reproducibility

검증 결과는 재실행 가능해야 한다.

재실행에 필요한 모델 식별자, 소스 버전, 명령 시퀀스, 시뮬레이터 설정, 타임스텝을 결과와 함께 남긴다. 같은 입력을 다시 실행했을 때 비교에 사용한 설정이 결과 파일만으로 복원되어야 한다.

### NFR-002 — Traceability

Requirement → Test → Evidence 관계를 추적할 수 있어야 한다.

각 요구사항은 5절 행렬의 Requirement, Component, Test, Evidence, Gate로 연결한다. 아직 없는 Test와 Evidence는 `TBD`로 둔다. 존재하지 않는 테스트 이름이나 증거 경로를 미리 합격란에 적지 않는다.

### NFR-003 — Simulator Independence

Control Core가 특정 시뮬레이터 API에 직접 종속되지 않도록 한다.

코어의 제어 경로는 Canonical Robot API와 `IRobotController`이다. 시뮬레이터 SDK 자료형이 코어의 명령, 상태, FK 입력으로 새어 들어가면 이 요구사항을 위반한다. 대응 원칙은 Project Charter 7절이다.

### NFR-004 — Measurability

PASS/FAIL 판정에 쓸 수 있는 정량 데이터가 존재해야 한다.

판정에는 측정값, 단위, 기준, 표본이 함께 있어야 한다. 기준이 아직 없는 항목은 FAIL도 PASS도 아닌 미확정으로 둔다. Latency가 이 경우에 해당한다. 정성 설명만으로 PASS를 기록하지 않는다.

### NFR-005 — Logging

Command, Feedback, Error, Latency를 기록할 수 있어야 한다.

로그는 REQ-012의 분리 규칙을 따른다. 오류 로그를 남기지 않은 채 성공 로그만 보관하는 구성은 이 요구사항을 만족하지 않는다.

### NFR-006 — Failure Visibility

오류를 숨기거나 자동으로 PASS 처리하지 않는다.

예외를 삼켜 다음 사이클을 성공으로 이어 가지 않는다. 타임아웃, 임포터 불일치, 기구학 실패, 기대하지 않은 충돌, 피드백 누락은 각각 실패 또는 미측정으로 남긴다. 테스트를 통과시키기 위해 기대값을 결과 쪽으로 옮기는 행위는 실패를 숨기는 것이다.

### NFR-007 — Version Pinning

중요 의존성과 로봇 소스의 버전을 기록한다.

기록 대상에는 로봇 설명 소스, 임포터, 시뮬레이터, 미들웨어, 검증에 영향을 주는 라이브러리가 포함된다. P-000A는 로봇 소스를 아직 핀하지 않는다. 소스를 채택하는 Source Baseline Gate에서 버전, 배포 경로, 확인 날짜를 적는다. 버전이 비어 있는 동안 해당 모델 검증을 PASS로 닫지 않는다.

## 3. Assumptions

요구사항에 적용하는 가정이다. Project Charter 10절과 같은 문장이다.

- 실제 PMAC hardware는 확보되지 않았다고 가정한다.
- 실제 Core Robotics proprietary protocol은 알 수 없다고 가정한다.
- FilterBot 내부 설계/제어 변수는 알 수 없다고 가정한다.
- 따라서 PMAC 계층은 공개적으로 검증 가능한 정보가 확보되기 전까지 abstraction/virtual-controller 수준으로 구현한다.
- 실제 제품의 사양을 추정하여 구현하지 않는다.
- 실제 체결 Tool과 Force/Torque Sensor는 확보되지 않았다고 가정한다. Bolt Torque 숫자는 제품 사양이 아니다.

REQ-010은 이 가정 안에서만 해석한다. REQ-028의 Torque도 이 가정 안에서만 해석한다.

## 4. Initial Engineering Targets

아래 값은 최종 보증값이 아니다. 초기 Engineering Target이다. 측정 방법과 증거가 생기기 전에는 합격 선언에 사용하지 않는다.

```text
FK Position Error       < 0.5 mm
FK Orientation Error    < 0.1 deg

Joint Target Error      < 0.1 deg

Cross-Simulator
TCP Position Error      < 1.0 mm

Unexpected Collision    0

Reliability Target      500 cycles
```

| 항목 | 초기 Engineering Target | 연결 요구사항 | 이번 기준선의 상태 |
| --- | --- | --- | --- |
| FK Position Error | < 0.5 mm | REQ-004 | 보증값 아님 |
| FK Orientation Error | < 0.1 deg | REQ-004 | 보증값 아님 |
| Joint Target Error | < 0.1 deg | REQ-005 | 보증값 아님 |
| Cross-Simulator TCP Position Error | < 1.0 mm | REQ-008 | 보증값 아님 |
| Unexpected Collision | 0 | REQ-009 | 보증값 아님 |
| Reliability Target | 500 cycles | REQ-018 | 보증값 아님 |

Latency는 PASS 임계값을 정하지 않는다. REQ-017이 정의하는 측정 항목만 먼저 수집한다.

```text
Mean
P50
P95
P99
Max
```

임계값은 측정 결과와 실제 아키텍처를 확인한 뒤에 확정한다.

## 5. Requirement Traceability Matrix

Test와 Evidence 파일은 아직 없다. 만들지 않은 테스트와 증거 경로 대신 `TBD`를 적는다.

Gate도 `TBD`이다. 추적 행렬 예시의 `P-001`은 열 형식을 보여 주기 위한 표기이며, 이 기준선은 Phase를 배정하지 않는다. P-001A.1은 Test 파일을 만들지 않으며 P-001B를 시작하지 않는다.

Component 열은 이후 구현이 놓일 논리적 위치이다. 해당 경로의 코드는 아직 없다.

| Requirement | Component | Test | Evidence | Gate |
| --- | --- | --- | --- | --- |
| REQ-001 Canonical Robot Model | robot_description | TBD | TBD | TBD |
| REQ-002 Unity Robot | unity | TBD | TBD | TBD |
| REQ-003 Isaac Robot | isaac | TBD | TBD | TBD |
| REQ-004 Independent FK | core/kinematics | TBD | TBD | TBD |
| REQ-005 IK | core/kinematics | TBD | TBD | TBD |
| REQ-006 Canonical Robot Command | core | TBD | TBD | TBD |
| REQ-007 Controller Abstraction | core | TBD | TBD | TBD |
| REQ-008 Cross-Simulator Validation | tests | TBD | TBD | TBD |
| REQ-009 Physics Validation | core | TBD | TBD | TBD |
| REQ-010 Virtual PMAC-compatible Controller | adapters | TBD | TBD | TBD |
| REQ-011 Controller State Machine | core | TBD | TBD | TBD |
| REQ-012 Command / Feedback | core | TBD | TBD | TBD |
| REQ-013 ROS2 Integration | ros2_ws, adapters | TBD | TBD | TBD |
| REQ-014 Digital Twin | core | TBD | TBD | TBD |
| REQ-015 Vision Coordinate Pipeline | core | TBD | TBD | TBD |
| REQ-016 HMI | hmi | TBD | TBD | TBD |
| REQ-017 Telemetry | evidence | TBD | TBD | TBD |
| REQ-018 Reliability Test | tests | TBD | TBD | TBD |
| REQ-019 Workcell Simulation | core | TBD | TBD | TBD |
| REQ-020 Task-Level Command | core | TBD | TBD | TBD |
| REQ-021 Pre-Execution Validation | core | TBD | TBD | TBD |
| REQ-022 Safety Zone | core | TBD | TBD | TBD |
| NFR-001 Reproducibility | evidence | TBD | TBD | TBD |
| NFR-002 Traceability | docs | TBD | TBD | TBD |
| NFR-003 Simulator Independence | core | TBD | TBD | TBD |
| NFR-004 Measurability | evidence | TBD | TBD | TBD |
| NFR-005 Logging | evidence | TBD | TBD | TBD |
| NFR-006 Failure Visibility | evidence | TBD | TBD | TBD |
| NFR-007 Version Pinning | docs | TBD | TBD | TBD |
| REQ-023 Representative Use Case | docs | TBD | TBD | TBD |
| REQ-024 Bolt Task Commands | core | TBD | TBD | TBD |
| REQ-025 Bolt Tool and Tool TCP | core | TBD | TBD | TBD |
| REQ-026 Bolt Pre-Execution Result | core | TBD | TBD | TBD |
| REQ-027 Bolt Execution Process | core | TBD | TBD | TBD |
| REQ-028 Virtual Torque and Angle Evidence | evidence, hmi | TBD | TBD | TBD |

P-001A.1이 추가한 요구사항의 연결이다. Planned Gate는 열지 않았다. Evidence type은 이후 기록이 담아야 할 종류이며, 이번 게이트의 PASS 파일이 아니다.

| Requirement | Architecture responsibility | Planned Gate | Evidence type |
| --- | --- | --- | --- |
| REQ-023 Representative Use Case | 플랫폼 정체와 대표 Use Case의 경계. Task Command Layer가 작업 이름을 받고, Workcell은 generic으로 남는다. | TBD. 열지 않음 | 문서 결정. 측정 Evidence 아님 |
| REQ-024 Bolt Task Commands | Task Command Layer가 명령을 받고 Task Decomposer가 motion/process 후보로 분해한다. | TBD. 열지 않음 | task command 기록 |
| REQ-025 Bolt Tool and Tool TCP | Frame Manager가 `flange`, bolt tool, `bolt_tool_tcp`를 구분한다. 숫자는 없다. | TBD. 열지 않음 | tool frame 정의 기록 |
| REQ-026 Bolt Pre-Execution Result | Validation Engine이 REQUIRED 목록으로 `SAFE_TO_ATTEMPT` 또는 `REJECTED`를 낸다. | TBD. 열지 않음 | pre-execution result 기록 |
| REQ-027 Bolt Execution Process | 실행 과정 state와 `FASTENED` / `LOOSENED` / `FAILED` / `FAULT`는 controller state와 다른 축이다. | TBD. 열지 않음 | execution result 기록 |
| REQ-028 Virtual Torque and Angle Evidence | Evidence Recorder와 Operator HMI가 virtual 표시와 Torque-Angle 항목을 남긴다. | TBD. 열지 않음 | virtual로 표시된 torque-angle 기록 |
