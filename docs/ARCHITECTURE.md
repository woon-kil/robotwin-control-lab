# Architecture

RoboTwin Control Lab — P-000B Architecture Contract

| 항목 | 값 |
| --- | --- |
| 문서 | Architecture Contract |
| 기준선 | `62b9a624083cb6ac584e4eb6fd6a69c8c56197ee` |
| 현재 Gate | P-000B |
| 구현 | NOT STARTED |
| 작성일 | 2026-10-23 |
| 관련 문서 | `docs/PROJECT_CHARTER.md`, `docs/REQUIREMENTS.md`, `docs/COORDINATE_CONVENTION.md`, `docs/ROBOT_INTERFACE.md`, `docs/VALIDATION_PLAN.md`, `docs/EVIDENCE_RULES.md` |

이 문서는 REQ-001–REQ-022를 구현 가능한 구조 계약으로 옮긴다. 로봇 모델, 시뮬레이터 프로젝트, 컨트롤러 코드는 만들지 않는다. P-001을 시작하지 않는다.

## 1. Layer contract

```text
Operator / HMI
      │
      ▼
Task-Level Command
      │
      ▼
Task Decomposition
      │
      ▼
Canonical Robot Command
      │
      ▼
Motion / Control Core
      │
      ▼
Pre-Execution Validator
      │
      ├─ Reachability
      ├─ IK Feasibility
      ├─ Joint Limit
      ├─ Collision
      ├─ Self Collision
      ├─ Safety Zone
      ├─ Trajectory
      ├─ TCP Target
      └─ Physics when applicable
      │
      ▼
PASS / FAIL
 │        │
FAIL     PASS
 │        │
REJECT   ▼
      IRobotController
           │
    ┌──────┼──────┬──────┐
    ▼      ▼      ▼      ▼
 Unity   Isaac   ROS2   PMAC
Adapter  Adapter Adapter Adapter
    │      │      │      │
    └──────┴──────┴──────┘
           │
        Feedback
           │
           ▼
      Digital Twin
           │
           ▼
     Evidence / HMI
```

Operator / HMI는 Task-Level Command 또는 Canonical Robot Command로 요청한다. Motion / Control Core는 시뮬레이터 SDK를 호출하지 않는다. 동작과 작업 실행 명령은 Pre-Execution Validator의 적용 가능한 required 결과가 PASS일 때만 `IRobotController`로 넘어간다. FAIL은 REJECT이며 그 명령의 컨트롤러 실행으로 이어지지 않는다.

Unity, Isaac, ROS2, PMAC는 `IRobotController` 아래의 독립 어댑터다. PMAC Adapter는 ROS2 Adapter에 종속되지 않는다. 이후 배치에서 Control Core에서 ROS2를 거쳐 외부 컨트롤러로 가는 경로나 다른 전송을 쓸 수는 있다. 그 경로는 Canonical Architecture의 의존성으로 고정하지 않는다.

Feedback은 어댑터에서 Digital Twin으로 돌아온다. 명령을 보냈다는 기록은 Feedback이 아니다.

## 2. Principles

### Core independence

`core/`는 Unity, Isaac Sim, ROS2, PMAC의 API, 자료형, 씬 그래프에 종속되지 않는다. 종속 금지는 NFR-003과 같다.

### Adapter isolation

엔진 또는 컨트롤러에만 있는 코드는 해당 adapter 뒤에 둔다. Core가 그 코드를 직접 호출하지 않는다.

### No simulator authority

Unity와 Isaac Sim은 Canonical Robot State를 정의하지 않는다. 두 시뮬레이터의 포즈는 비교 대상이며, 기준 상태의 소유자가 아니다.

### No PMAC authority

PMAC에만 있는 표현은 Canonical API를 바꾸지 않는다. 가상 컨트롤러와 향후 실제 어댑터는 `IRobotController`를 구현한다.

### Validation before execution

동작 명령의 컨트롤러 실행은 Pre-Execution Validation의 전체 결과가 PASS인 뒤에만 허용되는 구조다. 이 규칙은 프로젝트 내부 공학 정책이다. 산업 안전 인증이 아니다.

## 3. Component responsibility

논리 컴포넌트다. 이 게이트는 디렉터리나 코드를 추가하지 않는다.

### Task Command Layer

- **Responsibility:** REQ-020의 작업 단위 명령을 받는다. Canonical Robot Command와 섞지 않는다.
- **Inputs:** Operator / HMI의 작업 요청, 작업 대상 식별, Workcell에서 이미 이름이 있는 목표.
- **Outputs:** Task Decomposer로 전달하는 작업 명령.
- **Dependencies:** Task Decomposer. Canonical 상태의 소유자가 아니다.
- **Must NOT Do:** Motor나 Joint를 직접 제어한다. 명령을 Canonical 목록에 몰래 추가한다. 특정 회사의 작업 프로토콜이라고 기록한다.
- **Related Requirements:** REQ-020, REQ-016, REQ-019

### Task Decomposer

- **Responsibility:** 작업 명령을 Canonical Robot Command 흐름으로 분해한다.
- **Inputs:** Task-Level Command, 작업에 필요한 목표 이름.
- **Outputs:** 순서가 있는 Canonical Robot Command 후보.
- **Dependencies:** Canonical Command Layer, Frame Manager.
- **Must NOT Do:** 분해 결과를 검증 없이 컨트롤러로 보낸다. `tool action` 같은 설명용 단계를 REQ-006 명령으로 승격한다. 분해 스키마를 이 게이트에서 고정한다.
- **Related Requirements:** REQ-020, REQ-006

### Canonical Command Layer

- **Responsibility:** REQ-006 어휘와 `docs/ROBOT_INTERFACE.md`의 명령 개념을 유지한다.
- **Inputs:** 분해된 명령 또는 HMI가 직접 보낸 Canonical 명령.
- **Outputs:** Motion / Control Core가 받는 명령 레코드.
- **Dependencies:** Robot Interface 계약.
- **Must NOT Do:** 시뮬레이터 함수 이름을 명령으로 올린다. 파라미터 자료형이나 serialization을 이 게이트에서 확정한다.
- **Related Requirements:** REQ-006, REQ-012

### Frame Manager

- **Responsibility:** `docs/COORDINATE_CONVENTION.md`의 프레임 이름과 표기를 적용한다. 수치 변환이 생기기 전에는 없음을 숨기지 않는다.
- **Inputs:** 프레임 이름, 나중에 핀될 모델의 프레임 관계.
- **Outputs:** 이름과 단위가 붙은 변환 참조. 수치가 없으면 미정.
- **Dependencies:** Coordinate Convention.
- **Must NOT Do:** Unity 또는 Isaac의 축을 Core 축으로 채택한다. 측정되지 않은 변환 숫자를 만든다.
- **Related Requirements:** REQ-001, REQ-015, NFR-003

### Kinematics Engine

- **Responsibility:** 시뮬레이터 API와 분리된 FK와 IK를 계산하는 자리다.
- **Inputs:** Canonical 모델, 관절 위치, TCP 목표.
- **Outputs:** TCP pose, joint target, 해 없음 또는 한계 위반.
- **Dependencies:** Frame Manager, Canonical 모델.
- **Must NOT Do:** Unity 또는 Isaac의 포즈 조회를 FK 결과로 재포장한다. 시뮬레이터 좌표 규칙을 직접 처리한다. 실패를 가장 가까운 성공 포즈로 바꾼다.
- **Related Requirements:** REQ-004, REQ-005, NFR-003

### Trajectory Engine

- **Responsibility:** Canonical 동작 명령의 시간 순서와 관절 또는 TCP 경로를 만든다.
- **Inputs:** MOVE_JOINT, MOVE_TCP, HOME의 목표와 scale 개념.
- **Outputs:** 검증 엔진이 검사할 trajectory.
- **Dependencies:** Kinematics Engine, Frame Manager.
- **Must NOT Do:** 타임스텝 차이를 오차 합격 사유로 쓴다. 컨트롤러에 검증 전 trajectory를 실행 요청으로 보낸다.
- **Related Requirements:** REQ-006, REQ-021

### Validation Engine

- **Responsibility:** 실행 전에 Reachability, IK Feasibility, Joint Limit, Collision, Self Collision, Safety Zone, Trajectory, TCP Target, 해당할 때의 Physics를 개별 결과로 만든다.
- **Inputs:** 명령, trajectory, 관절 해, Workcell 상태, 출처가 있는 물리 파라미터.
- **Outputs:** validator별 PASS, FAIL, NOT_EVALUATED와 전체 집계.
- **Dependencies:** Kinematics Engine, Trajectory Engine, Frame Manager, Workcell 개념.
- **Must NOT Do:** 측정값이나 threshold를 만들어 채운다. NOT_EVALUATED를 PASS로 바꾼다. 출처 없는 물리 값으로 PASS를 준다. 결과를 산업 안전 인증이라고 부른다.
- **Related Requirements:** REQ-021, REQ-008, REQ-009, REQ-022, NFR-004, NFR-006

### Controller Abstraction

- **Responsibility:** `IRobotController` 경계를 유지한다. Core는 이 인터페이스에만 동작 실행을 요청한다.
- **Inputs:** 전체 검증이 PASS인 동작 명령, 상태 조회, 동작 권한을 거두는 요청.
- **Outputs:** 어댑터로의 논리 호출, Feedback으로 돌아올 상태의 자리.
- **Dependencies:** Robot Interface 계약.
- **Must NOT Do:** 언어별 인터페이스 파일을 이 게이트에서 만든다. 하나의 어댑터 자료형을 Core 상태로 둔다.
- **Related Requirements:** REQ-007, REQ-011, REQ-012

### Unity Adapter

- **Responsibility:** Canonical 명령과 상태를 Unity 표현으로 바꾸고, Unity Feedback을 Canonical 상태로 되돌린다.
- **Inputs:** `IRobotController` 호출, Unity 쪽 상태.
- **Outputs:** Canonical Feedback, 변환에 사용한 규약의 참조.
- **Dependencies:** UnityCoordinateAdapter, Controller Abstraction.
- **Must NOT Do:** Canonical Robot State를 정의한다. Core에 Unity API 자료형을 노출한다. 좌표 변환 행렬을 측정 없이 확정한다.
- **Related Requirements:** REQ-002, REQ-014, REQ-008

### Isaac Adapter

- **Responsibility:** Canonical 명령과 상태를 Isaac Sim 표현으로 바꾸고, Feedback을 Canonical 상태로 되돌린다.
- **Inputs:** `IRobotController` 호출, Isaac 쪽 상태.
- **Outputs:** Canonical Feedback, 변환 규약의 참조.
- **Dependencies:** IsaacCoordinateAdapter, Controller Abstraction.
- **Must NOT Do:** Canonical Robot State를 정의한다. Isaac 기본 관성이나 기본 축을 모델 값으로 올린다. 좌표 변환 행렬을 측정 없이 확정한다.
- **Related Requirements:** REQ-003, REQ-014, REQ-008

### ROS2 Adapter

- **Responsibility:** Canonical 명령과 상태를 ROS2 경계로 옮긴다.
- **Inputs:** `IRobotController` 호출, ROS2에서 도착한 상태.
- **Outputs:** Canonical Feedback.
- **Dependencies:** Controller Abstraction, Coordinate Convention.
- **Must NOT Do:** ROS2 메시지를 Canonical 명령의 정의로 둔다. 패키지를 이 게이트에서 만든다. PMAC Adapter를 자신의 하위로 둔다. PMAC 비공개 프로토콜을 ROS2 메시지에 추정해 넣는다.
- **Related Requirements:** REQ-013, REQ-006, REQ-012

### PMAC Adapter

- **Responsibility:** `IRobotController`와 PMAC 측 표현 사이의 경계만 유지한다. 현재 구현 자리는 Virtual PMAC-compatible Controller다.
- **Inputs:** 논리 컨트롤러 호출.
- **Outputs:** 가상 컨트롤러의 통신과 상태 전이 결과. 실제 하드웨어 결과는 이 게이트의 출력이 아니다.
- **Dependencies:** Controller Abstraction, PMAC Boundary.
- **Must NOT Do:** ROS2 Adapter를 거쳐야만 호출되는 구조로 둔다. 레지스터 주소, 명령 문법, 메모리 맵, 모션 프로그램 문법, 비공개 상태, Core Robotics 전용 프로토콜을 추정한다. Canonical API를 PMAC 표현에 맞추어 바꾼다.
- **Related Requirements:** REQ-010, REQ-007, REQ-011

### Digital Twin State

- **Responsibility:** 컨트롤러 Feedback을 시뮬레이터 로봇 상태에 반영할 상태의 자리다.
- **Inputs:** Canonical Feedback.
- **Outputs:** Twin에 반영할 관절과 TCP, 반영 시각.
- **Dependencies:** Controller Abstraction, Frame Manager.
- **Must NOT Do:** 시뮬레이터가 먼저 움직인 결과를 Feedback인 것처럼 기록한다. Unity 화면을 Isaac의 뷰어로 두거나 그 반대를 한다.
- **Related Requirements:** REQ-014, REQ-012

### Telemetry

- **Responsibility:** 명령, Feedback, 오류, 지연, 타임스텝, 모델 식별자를 정량 기록으로 남길 자리다.
- **Inputs:** 명령 레코드, Feedback, validator 결과, 시계 표본.
- **Outputs:** Mean, P50, P95, P99, Max를 계산할 수 있는 지연 표본과 그 외 측정 후보.
- **Dependencies:** Evidence Recorder.
- **Must NOT Do:** 지연 threshold를 이 게이트에서 정한다. 빈 지연을 0으로 기록한다. 통계를 낼 수 없는 실행을 측정된 값으로 바꾼다.
- **Related Requirements:** REQ-017, NFR-004, NFR-005

### Evidence Recorder

- **Responsibility:** `docs/EVIDENCE_RULES.md`에 따라 입력, 결과, 판정, 원자료를 남긴다.
- **Inputs:** 테스트 실행의 입력과 출력, 환경, Git commit.
- **Outputs:** 덮어쓰지 않는 Evidence 기록.
- **Dependencies:** Evidence Rules.
- **Must NOT Do:** 화면에서 움직였다는 문장만으로 PASS를 쓴다. 실패 기록을 지운다. 없는 값을 추정해 메타데이터를 채운다. 다른 프로젝트의 측정값을 이 저장소의 PASS로 재사용한다.
- **Related Requirements:** NFR-001, NFR-002, NFR-006, NFR-007

### Operator HMI

- **Responsibility:** 컨트롤러 상태, 관절, TCP, 명령, 시뮬레이터, 충돌, 오류, 지연을 Canonical API를 통해 보여 준다.
- **Inputs:** Canonical 상태, validator 결과, telemetry.
- **Outputs:** 운용자 표시, 작업 또는 Canonical 명령 요청.
- **Dependencies:** Task Command Layer, Canonical Command Layer.
- **Must NOT Do:** Unity API나 Isaac API를 제어 경로로 호출한다. 측정되지 않은 지연을 0으로 보여 준다. Euler 각을 저장 표현으로 쓴다.
- **Related Requirements:** REQ-016, REQ-020, NFR-003

## 4. Execution rule

동작과 작업 실행 명령은 HOME, MOVE_JOINT, MOVE_TCP, HANDLE_FILTER, PICK_OBJECT, PLACE_OBJECT, BOLT_FASTEN, BOLT_LOOSEN이다. 이 명령은 적용 가능한 required validator의 PASS 없이 컨트롤러 실행으로 진행하지 않는다.

```text
Validation FAIL
→ Controller execution prohibited

Validation PASS
→ execution may be approved
```

PASS는 실행을 허가할 수 있다는 뜻이다. 산업 안전 인증이 아니다.

`BOLT_FASTEN`과 `BOLT_LOOSEN`의 Pre-Execution 전체 결과 이름은 `SAFE_TO_ATTEMPT`와 `REJECTED`다. `SAFE_TO_ATTEMPT`는 체결이나 해체 완료가 아니다. 작업자 승인 전에 컨트롤러 실행을 시작하지 않는다. REQUIRED 목록과 Execution 결과는 REQ-026, REQ-027이다. 실행 과정 state는 REQ-011의 controller state를 대체하지 않는다.

STOP과 ESTOP은 보호 및 종료 명령이다. 동작 가능성을 묻는 명령이 아니므로 Pre-Execution Motion Validation의 PASS를 요구하지 않는다.

```text
ESTOP
→ Pre-Execution Validation
→ NOT_EVALUATED
→ BLOCKED
```

위 경로는 이 아키텍처의 명령 의미가 아니다.

```text
STOP / ESTOP
→ protective/termination control path
```

이 구분은 RoboTwin 소프트웨어의 command semantics다. 인증된 비상정지 구현, 기능 안전 적합성, 하드웨어 안전 회로와의 등가, ISO/IEC 안전 인증, 실제 PMAC E-stop 동작을 주장하지 않는다.

SERVO_OFF, HOLD, RESET에 동작 검증을 적용할지는 정하지 않는다. 8절 Open Issue로 둔다. REQ-001–REQ-022의 문장은 바꾸지 않는다.

## 5. P-001 intent

P-000B는 P-001을 실행하지 않는다. 다음 게이트의 목적만 적는다.

```text
P-001
UR10e Golden Reference
```

```text
trusted public robot model
        ↓
Canonical Robot Model
        ↓
Independent FK
        ↓
known joint configurations
        ↓
quantitative FK baseline
```

P-001을 시작할 때 Source Selection과 Pinning을 먼저 한다. 이 문서에서 UR10e의 source, repository, commit, model version을 고르지 않는다.

## 6. Maintenance boundary

진단, 진동, 베어링, 잔여수명, 정비 추천, maintenance agent는 이 아키텍처의 컴포넌트가 아니다. 책임은 Maintenance-Agentic-Twin에 있다. 통합은 이 게이트의 작업이 아니다.

## 7. Adapter independence

`docs/PROJECT_CHARTER.md` 3절과 7절은 이 문서와 같이 Unity, Isaac, ROS2, PMAC를 `IRobotController` 아래의 독립 어댑터로 둔다. PMAC를 ROS2의 하위로 두지 않는다.

## 8. Open items

이 게이트가 결정하지 않은 항목이다. 추정으로 채우지 않는다.

- SERVO_OFF, HOLD, RESET에 Pre-Execution Motion Validation을 적용할지는 정하지 않았다.
- 작업별로 어떤 validator가 required인지는, Bolt 대표 명령을 제외하면 아직 목록이 없다. `BOLT_FASTEN`과 `BOLT_LOOSEN`의 REQUIRED 목록은 REQ-026이다.
- 타임스탬프의 epoch와 시계 종류는 정하지 않았다.
- Quaternion의 부호 정규화는 정하지 않았다.
- 프레임 사이 수치 변환과 시뮬레이터 변환 행렬은 정하지 않았다.
- UR10e 소스는 P-001A에서 official nominal description으로 핀했다. 기록은 `robot_description/SOURCE_MANIFEST.md`다. 이 아키텍처 문서는 그 핀의 파라미터를 다시 정의하지 않는다.

## 9. Representative use case

P-001A.1이 대표 Use Case를 이 계약 위에 얹는다. 1절의 계층, 어댑터 분리, STOP/ESTOP 경로는 유지한다.

```text
Robot Control Validation Platform
        ↓
Generic Task Architecture
        ↓
Representative Use Case
        ↓
Bolt Fastening / Loosening
```

Task Command Layer는 `BOLT_FASTEN`과 `BOLT_LOOSEN`을 task-level 명령으로 받는다. Canonical Robot Command로 올리지 않는다. `HANDLE_FILTER`는 generic 후보로 남고, 필터 전체 공정의 MVP 구현 명령이 아니다.

Frame Manager는 대표 공구의 제어점을 `bolt_tool_tcp`로 구분한다. 사슬은 `flange` → bolt tool → `bolt_tool_tcp`다. 숫자는 없다.

Validation Engine은 그 두 명령의 REQUIRED 검사로 `SAFE_TO_ATTEMPT` 또는 `REJECTED`를 낸다. Evidence Recorder와 Operator HMI는 Execution의 Torque/Angle을 virtual로 표시한다. 실제 제품 Torque와 proprietary protocol은 이 절의 입력이 아니다.

이 절은 FK, IK, Tool, Torque, Unity, Isaac, P-001B를 시작하지 않는다.
