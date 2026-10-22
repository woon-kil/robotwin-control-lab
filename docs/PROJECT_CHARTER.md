# RoboTwin Control Lab

Industrial Robot Digital Twin & Control Validation Platform

| 항목 | 값 |
| --- | --- |
| 문서 | Project Charter |
| 단계 | P-000A — Architecture Baseline |
| 현재 Gate | P-000A |
| 구현 | NOT STARTED |
| 작성일 | 2026-10-23 |

이 문서는 프로젝트의 목적, 범위, 구조 원칙, 가정, 초기 위험을 고정한다. 로봇 소스, 시뮬레이터 프로젝트, 제어 코드는 만들지 않는다. P-000A 완료가 다음 단계를 자동으로 시작하지 않는다.

## 1. Project Name

```text
RoboTwin Control Lab
```

Subtitle:

```text
Industrial Robot Digital Twin & Control Validation Platform
```

이 프로젝트는 로봇 애니메이션이나 단일 시뮬레이터 데모가 아니다. Unity와 NVIDIA Isaac Sim에서 같은 로봇 모델과 표준 Robot Command를 실행하고, 시뮬레이터 API와 분리된 Kinematics/Validation 계층으로 결과를 검증하며, ROS2 및 PMAC-compatible Controller와 연동할 수 있는 산업용 로봇 Digital Twin 및 제어 검증 플랫폼을 만든다.

실제 Core Robotics FilterBot을 복제했다는 주장은 하지 않는다.

## 2. Problem Statement

산업용 로봇 소프트웨어는 실제 장비에 올리기 전에, 모델·좌표·동작·물리·통신·감시가 서로 맞는지 확인해야 한다. 화면에서 로봇이 움직인다는 사실만으로는 그 확인이 끝나지 않는다.

현장에 들어가기 전에 검증이 필요한 문제는 다음과 같다.

- **Robot model correctness.** 링크 길이, 관절 축, 한계, 단위가 사용하려는 로봇의 설명과 일치해야 한다. 모델이 어긋나면 이후의 궤적과 충돌 판단이 함께 어긋난다.
- **Joint/Link hierarchy.** 부모-자식 관계와 관절 순서가 설명 파일, 컨트롤러, 시뮬레이터에서 같아야 한다. 계층이 바뀌면 같은 관절 명령이 다른 링크를 움직인다.
- **Coordinate system consistency.** 원점, 축 방향, 단위, 오른손/왼손 좌표계가 경계마다 달라지면 TCP 수치를 비교할 수 없다.
- **FK/IK correctness.** 순기구학으로 나온 TCP와 역기구학으로 되돌린 관절 목표가 독립적으로 검토되어야 한다. 시뮬레이터 내부 솔버 출력만으로 기구학을 합격 처리하지 않는다.
- **Trajectory correctness.** 관절 또는 TCP 명령이 의도한 경로와 시간 순서를 따르는지 확인해야 한다.
- **Physics behavior.** 중력, 질량, 관성, 페이로드, 마찰이 명시된 값으로 작용하는지 측정해야 한다. 엔진 기본값에 맡긴 동작은 물리 검증이 아니다.
- **Collision.** 기대하지 않은 접촉이 없어야 하며, 접촉이 있으면 숨기지 않고 기록해야 한다.
- **Controller communication.** 명령이 컨트롤러 경계에 전달되고, 응답이 그 명령과 구분되어 돌아와야 한다.
- **Command/Feedback consistency.** 보낸 명령과 컨트롤러가 보고한 상태는 다른 데이터다. 명령을 기록했다고 도달했다고 보지 않는다.
- **Simulator-to-simulator consistency.** 같은 모델과 같은 명령에 대해 Unity, Isaac Sim, 독립 기구학의 결과를 대조할 수 있어야 한다.
- **Digital Twin synchronization.** 컨트롤러 피드백이 시뮬레이터 로봇 상태에 반영되어야 한다. 시뮬레이터를 먼저 움직이고 그 결과를 실제 상태로 간주하지 않는다.
- **Operator monitoring.** 운영자가 컨트롤러 상태, 관절, TCP, 명령, 시뮬레이터, 충돌, 오류, 지연을 볼 수 있어야 한다.
- **Long-running reliability.** 한 번의 시연이 아니라 반복 동작에서 상태가 유지되는지 확인해야 한다. 초기 목표는 500 cycles이다.

## 3. Project Objective

최종 구조는 다음과 같다.

```text
Operator HMI
     │
     ▼
Canonical Robot API
     │
     ▼
Control / Validation Core
     │
     ▼
IRobotController
     │
 ┌───┼────────────┐
 ▼   ▼            ▼
Unity Isaac       ROS2
                  │
                  ▼
            PMAC-compatible
              Controller
```

Operator HMI는 Canonical Robot API만 호출한다. Control / Validation Core는 명령, 상태, 기구학, 검증 기록을 담당한다. `IRobotController` 아래의 Unity, Isaac Sim, ROS2, PMAC-compatible Controller는 각각의 어댑터이며, 이 중 하나의 API가 프로젝트의 제어 구조가 되지 않는다.

## 4. Golden Reference Strategy

초기 검증은 자료를 확보하기 쉬운 산업용 6축 로봇을 Golden Reference로 사용한다.

현재 후보:

```text
UR10e
```

P-000A는 이 후보의 소스와 버전을 확정하지 않는다. URDF, USD, 벤더 패키지를 받지 않는다. 제조사, 배포 경로, 파일 버전, 라이선스, 기구학 파라미터의 출처는 P-001을 시작하기 전에 별도의 Source Baseline Gate에서 공식 소스와 대조해 검증한다. 그 게이트를 통과하기 전에는 Golden Reference가 잠긴 것으로 기록하지 않는다.

## 5. Target Robot Strategy

Golden Reference 검증이 끝난 뒤 다음 대상으로 확장한다.

```text
Generic Long-Reach Manipulator
```

확장 목적:

- Long reach kinematics
- Payload variation
- Gravity/inertia effects
- Collision
- Vision
- Controller integration
- Digital Twin validation

```text
The Generic Long-Reach Manipulator is a portfolio model
inspired by industrial long-reach manipulation requirements.

It is NOT a reproduction of any proprietary Core Robotics robot.
```

이 모델의 치수, 질량, 관성, 제어 이득을 실제 제품 사양인 것처럼 채우지 않는다. 출처가 문서화되지 않은 물리 값은 검증 통과의 근거가 될 수 없다.

## 6. Simulator Responsibilities

Unity와 Isaac Sim은 같은 로봇 설명과 같은 Canonical Robot Command를 실행한다. 두 시뮬레이터는 각자의 역할을 가진다. 어느 한 시뮬레이터를 다른 시뮬레이터의 단순 Viewer로 정의하지 않는다. 두 시뮬레이터의 결과를 독립 기구학 및 컨트롤러 피드백과 비교한다.

### Unity

주요 목적:

- Interactive 3D simulator
- Operator interaction
- HMI
- Robot visualization
- Control visualization

Unity는 운영자가 로봇과 제어 상태를 보고 조작하는 대화형 환경이다. Unity API 호출 순서를 프로젝트의 제어 프로토콜로 사용하지 않는다.

### Isaac Sim

주요 목적:

- Robotics simulation
- Physics validation
- PhysX
- Sensor simulation
- ROS2 integration
- 향후 AI/RL validation

Isaac Sim은 물리, 센서, ROS2 연동을 포함한 로보틱스 시뮬레이션을 담당한다. 향후 AI/RL 검증은 최종 범위의 확장 항목이다. MVP의 완료 조건은 아니며, P-000A에서 착수하지 않는다.

## 7. Control Architecture Principle

```text
Simulator-specific API must not become the project's control architecture.
```

금지되는 의존 방향:

```text
BAD

HMI
 ↓
Unity API
```

기준 구조:

```text
GOOD

HMI
 ↓
Canonical Robot API
 ↓
IRobotController
 ↓
UnityAdapter
```

같은 원칙을 Isaac Sim, ROS2, PMAC-compatible Controller에 적용한다.

```text
HMI
 ↓
Canonical Robot API
 ↓
IRobotController
 ↓
IsaacAdapter

HMI
 ↓
Canonical Robot API
 ↓
IRobotController
 ↓
Ros2Adapter
 ↓
PmacCompatibleController
```

Control / Validation Core는 특정 시뮬레이터 SDK, 씬 그래프, 또는 벤더 모션 API의 자료형에 직접 의존하지 않는다. 변환은 어댑터 경계에서 일어난다.

## 8. Evidence-Driven Development

이 프로젝트의 개발 순서는 다음이다.

```text
IMPLEMENT
   ↓
TEST
   ↓
MEASURE
   ↓
EVIDENCE
   ↓
REVIEW
   ↓
PASS / FAIL
```

기능이 화면에 보이는 것만으로 PASS로 기록하지 않는다. PASS는 요구사항, 테스트, 측정값, 증거가 서로 연결되고 검토를 통과했을 때 사용한다. 실패, 누락, 타임아웃, 예기치 않은 충돌은 결과에 남긴다. 오류를 삼키거나 자동으로 PASS로 바꾸지 않는다.

P-000A는 이 순서의 문서 기준선이다. 구현, 테스트 실행, 측정, 증거 파일은 아직 없다.

## 9. Scope

### 9.1 In Scope

최종 프로젝트 범위:

- URDF/USD Robot Model
- Unity simulation
- Isaac Sim simulation
- Independent FK
- IK
- Joint control
- TCP pose
- Coordinate transformation
- Trajectory
- Physics validation
- Collision validation
- ROS2
- Controller abstraction
- PMAC-compatible Virtual Controller
- Controller state machine
- Command/Feedback
- Digital Twin
- Virtual Vision
- Camera → World → Robot transformation
- Operator HMI
- Telemetry
- Latency measurement
- Cross-simulator validation
- Reliability testing

향후 확장:

- Reinforcement Learning
- Imitation Learning
- Domain Randomization
- Sim-to-Real optimization

향후 확장은 최종 범위에 포함된다. MVP 완료 조건이 아니며, 이번 기준선에서 구현하지 않는다.

### 9.2 Out of Scope — MVP

MVP에서 필수 요구사항으로 두지 않는 항목:

- 실제 PMAC hardware
- 실제 FilterBot
- 실제 산업용 Robot hardware
- Humanoid
- Locomotion
- Multi-robot fleet
- 복잡한 AI Vision model
- LLM Robot Agent
- Production cloud platform

### 9.3 P-000A에서 하지 않는 일

이번 문서는 다음을 수행하지 않는다.

- UR10e 또는 기타 로봇 자산 다운로드
- URDF 다운로드
- Unity 프로젝트 생성
- Isaac Sim 실행 또는 프로젝트 생성
- ROS2 package 생성
- PMAC 구현
- Python, C#, C++ 구현
- FK/IK 구현
- Long-Reach Robot 모델링
- HMI 구현
- P-000B 및 P-001 이후 작업

## 10. Assumptions

다음 가정은 요구사항 해석과 이후 구현 범위에 적용된다. 가정이 깨지면 해당 요구사항의 범위를 다시 문서화한 뒤에만 구현을 넓힌다.

- 실제 PMAC hardware는 확보되지 않았다고 가정한다.
- 실제 Core Robotics proprietary protocol은 알 수 없다고 가정한다.
- FilterBot 내부 설계/제어 변수는 알 수 없다고 가정한다.
- 따라서 PMAC 계층은 공개적으로 검증 가능한 정보가 확보되기 전까지 abstraction/virtual-controller 수준으로 구현한다.
- 실제 제품의 사양을 추정하여 구현하지 않는다.

이 가정 때문에 REQ-010은 가상 컨트롤러의 통신과 상태 전이를 검증하는 요구사항이다. 비공개 프로토콜의 재구현을 요구하지 않는다.

## 11. Risk Register

각 위험의 Validation은 이후 단계에서 수행할 확인이다. P-000A에서는 테스트를 실행하지 않았고, 증거 파일도 없다.

### RISK-001 — Unity/Isaac coordinate convention mismatch

- **Description:** Unity와 Isaac Sim은 상향 축, 손 좌표계, 단위, 관절 부호를 서로 다르게 해석할 수 있다. 같은 로봇 설명을 넣어도 TCP 좌표가 어댑터마다 달라질 수 있다.
- **Impact:** 교차 시뮬레이터 비교와 Digital Twin 동기화의 수치를 신뢰할 수 없게 된다. 잘못된 변환이 통과 기록으로 남을 수 있다.
- **Mitigation:** 좌표, 단위, 축, 부호의 기준은 Canonical Robot Model에 둔다. 시뮬레이터 네이티브 좌표계는 어댑터 안에서만 변환한다. 변환에 사용한 규약을 측정 기록과 함께 남긴다.
- **Validation:** 동일 명령에 대한 Unity, Isaac Sim, 독립 FK의 TCP를 REQ-008로 비교한다. 프레임 이름과 단위가 기록되지 않은 결과는 비교 증거로 사용하지 않는다.

### RISK-002 — URDF importer differences

- **Description:** Unity와 Isaac Sim의 URDF/USD 임포터는 관절 누락, 링크 이름 변경, 축 재정렬, 관성 원점 이동을 만들 수 있다.
- **Impact:** 같은 설명 파일에서 서로 다른 기구학 트리가 생긴다. 이후 FK/IK와 충돌 검사가 다른 로봇을 검증하게 된다.
- **Mitigation:** 임포터 출력을 원본 설명과 대조한다. 관절 이름, 부모-자식, 축, 한계, 단위의 차이를 증거로 남기고, 차이를 조용히 수정본에 흡수하지 않는다.
- **Validation:** 구조 비교 테스트는 아직 없다 (TBD). Source Baseline Gate 이후에 원본 설명과 각 임포터 결과를 항목별로 비교한다.

### RISK-003 — Physics parameter mismatch

- **Description:** 질량, 관성, 마찰, 페이로드가 엔진 기본값 또는 서로 다른 솔버 설정으로 들어갈 수 있다.
- **Impact:** 물리 거동 비교가 모델 차이가 아니라 설정 누락을 측정하게 된다.
- **Mitigation:** 물리 파라미터는 출처가 있는 값으로 명시한다. 시뮬레이터 기본값을 모델 값으로 간주하지 않는다. 출처가 없는 값은 비워 두고 미정으로 남긴다.
- **Validation:** REQ-009에 따라 gravity, mass, inertia, payload, friction, collision을 측정한다. 파라미터 출처가 없는 항목은 PASS로 닫지 않는다.

### RISK-004 — Simulator timestep differences

- **Description:** Unity와 Isaac Sim의 타임스텝, 서브스텝, 솔버 반복 횟수가 다르다.
- **Impact:** 같은 명령이라도 궤적 샘플과 접촉 시점이 달라져, 프레임 번호 비교가 오탐 또는 미탐이 된다.
- **Mitigation:** 텔레메트리에 타임스텝과 명령 시각을 남긴다. 비교는 동일한 명령 경계에서 수행하고, 렌더 프레임 인덱스를 시간 기준으로 삼지 않는다.
- **Validation:** REQ-017 기록에 타임스텝과 명령 시각이 포함되는지 확인한다. 타임스텝 차이만으로 오차를 합격 처리하지 않는다.

### RISK-005 — Controller latency/jitter

- **Description:** Unity 어댑터, Isaac 어댑터, ROS2, 가상 PMAC 경로의 명령-피드백 지연과 지터가 다르다.
- **Impact:** Digital Twin이 늦거나, 지연이 기구학 오차로 오인된다.
- **Mitigation:** Latency의 Mean, P50, P95, P99, Max를 먼저 측정한다. 측정값과 실제 구조를 보기 전에는 PASS 임계값을 정하지 않는다.
- **Validation:** REQ-017과 NFR-004. 임계값이 비어 있는 동안 지연 데이터 부재는 실패로 남기고, 임의 임계값으로 채우지 않는다.

### RISK-006 — Unavailable proprietary PMAC information

- **Description:** 실제 PMAC 하드웨어, Core Robotics 비공개 프로토콜, FilterBot 내부 설계 및 제어 변수는 알 수 없다고 가정한다.
- **Impact:** 출처가 없는 통신 계층이 실제 제품과 맞는 구현으로 기록될 수 있다.
- **Mitigation:** 공개적으로 검증 가능한 정보가 생기기 전까지 PMAC 계층은 abstraction과 virtual controller로 한정한다. 제품 사양을 추정해 구현하지 않는다. FilterBot 복제를 주장하지 않는다.
- **Validation:** REQ-010의 증거는 가상 컨트롤러의 통신과 상태 전이로 제한한다. 비공개 레지스터, 패킷, 이득을 재현했다는 결과는 수용하지 않는다.

### RISK-007 — Long-Reach model physical realism

- **Description:** Generic Long-Reach Manipulator는 포트폴리오 모델이다. 측정된 질량, 강성, 처짐이 없으면 화면의 물리 거동이 실제와 비슷해 보여도 근거가 없다.
- **Impact:** 중력, 관성, 페이로드 결론이 출처 없는 숫자가 된다.
- **Mitigation:** Golden Reference 검증 전에 Long-Reach 모델을 만들지 않는다. 이후에도 파라미터마다 출처를 적고, 출처가 없으면 placeholder로 표시해 물리 PASS에 사용하지 않는다.
- **Validation:** NFR-006과 NFR-007. 물리 증거는 파라미터 출처를 포함해야 한다. 독점 로봇의 사양으로 읽힐 수 있는 값은 거부한다.

### RISK-008 — Scope expansion into unnecessary AI features

- **Description:** Reinforcement Learning, Imitation Learning, Domain Randomization, 복잡한 비전 모델, LLM Robot Agent는 MVP 필수 범위 밖에 있거나 향후 확장이다.
- **Impact:** 모델, 명령, 피드백, 교차 검증이 끝나기 전에 시연 기능이 기준선을 대체한다.
- **Mitigation:** MVP Out of Scope와 향후 확장 목록을 게이트마다 대조한다. 시뮬레이터가 해당 기능을 제공한다는 이유만으로 착수하지 않는다. P-000A는 다음 Phase를 열지 않는다.
- **Validation:** 각 게이트의 변경 범위가 9절과 요구사항 ID 안에 있는지 검토한다. 목록 밖 작업은 범위 변경 기록 없이 병합하지 않는다.

## 12. Baseline Boundary

P-000A가 잠그는 것:

- 프로젝트 이름, 목적, 구조 원칙
- In Scope / Out of Scope
- 기능 요구사항 REQ-001–REQ-018
- 비기능 요구사항 NFR-001–NFR-007
- 초기 Engineering Target과, Latency 임계값을 아직 정하지 않는다는 결정
- 가정과 초기 위험 RISK-001–RISK-008

P-000A가 잠그지 않는 것:

- 로봇 소스와 버전
- 좌표 규약의 최종 수치표
- 컨트롤러 상태 전이표
- 테스트 식별자, 증거 파일, Phase Gate 배정
- Latency PASS 임계값
- 의존성 버전 핀 (핀을 기록해야 한다는 요구만 둔다)

요구사항의 식별자와 추적 행렬은 `docs/REQUIREMENTS.md`가 정본이다.
