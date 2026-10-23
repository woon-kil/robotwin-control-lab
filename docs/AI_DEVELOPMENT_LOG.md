# AI Development Log

RoboTwin Control Lab의 AI 코딩 기록이다. 포트폴리오 증거로 남긴다.

AI가 제안했다는 사실만으로 기술 결정을 정당화하지 않는다. 최종 결정과 검증 책임은 Developer에게 있다. 이 로그의 초안 작성은 AI가 수행했고, 초안 수용 여부는 Developer 검수에서 결정한다.

## P-000A — Project Charter & Requirements Baseline

```text
Task
P-000A. 신규 프로젝트 robotwin-control-lab의 설계 기준선을 문서만으로 작성한다.

Problem
구현, 로봇 소스, 시뮬레이터 프로젝트가 없는 상태에서 목적과 범위가 먼저 고정되어 있지 않으면
이후 작업이 특정 시뮬레이터 API, 추정 사양, 화면 동작 여부에 의존하게 된다.
P-000A는 그 기준선을 문서화하는 단계이며, 구현 단계가 아니다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. 사용자 명세의 항목을 README.md, docs/PROJECT_CHARTER.md, docs/REQUIREMENTS.md,
   docs/AI_DEVELOPMENT_LOG.md와 지정된 빈 디렉터리(.gitkeep)로 나눈다.
2. 기능 요구사항 18개(REQ-001–REQ-018)와 비기능 요구사항 7개(NFR-001–NFR-007),
   위험 8개(RISK-001–RISK-008)를 명세 문장 그대로 식별자로 고정한다.
3. 성능 수치는 초기 Engineering Target으로만 적고 보증값으로 표기하지 않는다.
4. Latency는 Mean, P50, P95, P99, Max를 측정 항목으로만 정의하고 PASS 임계값은 비운다.
5. 아직 없는 Test, Evidence, Phase Gate는 TBD로 둔다.
   추적 행렬 예시의 P-001은 형식 예시로만 보고, 이번 문서에서 Phase를 배정하지 않는다.
6. 거부한 제안: UR10e 소스/버전 핀, URDF 또는 모델 다운로드, Unity/Isaac/ROS2 프로젝트 생성,
   FK/IK 및 컨트롤러 구현, 상태 전이표 창작, .gitignore 등 명세 밖 파일 추가,
   P-000B 또는 P-001 착수.

Developer Decision
명세에 있는 문서와 디렉터리만 작성한다. 로봇 소스와 버전은 확정하지 않는다.
PMAC 계층은 실제 하드웨어와 비공개 프로토콜이 없다는 가정 아래 추상화/가상 컨트롤러 범위로 남긴다.
Generic Long-Reach Manipulator는 포트폴리오 모델이며 독점 로봇의 복제로 기록하지 않는다.
문서 초안의 최종 수용은 Developer 검수에 둔다. 이 로그는 그 수용을 대신하지 않는다.

Accepted / Rejected
Accepted — 명세 범위의 문서 구조, Engineering Target 표기, Latency 임계값 보류, TBD 추적 행렬.
Rejected — 소스 핀, 자산 다운로드, 구현 스켈레톤, Phase 선배정, 다음 단계 자동 진행.
Pending — 초안 내용에 대한 Developer 검수.

Reason
기술 근거의 책임은 Developer 검수와 이후 Test/Measure/Evidence에 있다.
AI 초안은 명세를 문서 형태로 옮긴 결과이며, 시뮬레이터 동작이나 측정으로 검증된 설계가 아니다.
명세에 없는 Phase 번호와 로봇 버전을 채우면 확정되지 않은 결정을 기준선처럼 보이게 만든다.

Verification
2026-10-23 08:11 KST에 작업 트리에서 확인했다.
파일은 README.md, docs 3개, 빈 디렉터리 10개의 .gitkeep뿐이다.
REQ 18, NFR 7, RISK 8. 추적 행렬의 Test/Evidence/Gate TBD 행은 25개다.
Python, C#, C++, URDF, USD, Unity 프로젝트 파일은 0개다.
런타임, FK, 물리, 지연은 측정하지 않았다.

Elapsed Time
2026-10-23 08:07 KST 수신, 08:11 KST 산출물 확인. 약 4분.
로컬 커밋은 이 기록 직후 1회. 구현 사이클 시간은 측정하지 않았다.
```

## P-000A.1 — Workcell & Pre-Execution Validation Requirements Amendment

```text
Task
P-000A.1. P-000A FINAL PASS 기준선 bf876d22abdfb567b800ad49c966f27e4c5d7b92 위에
Workcell, Task-Level Command, Pre-Execution Validation, Safety Zone 요구사항을 추가한다.
구현 Phase가 아니다.

Problem
기존 기준선은 모델, 명령, 시뮬레이터, 컨트롤러 추상화를 정의한다.
산업용 로봇 업무에서 필요한 질문, 즉 컨트롤러에 명령을 전달하기 전에
작업을 안전하고 정확하게 수행할 수 있는지가 요구사항으로 분리되어 있지 않았다.
동시에 진단·정비 기능이 RoboTwin으로 섞이면 그 질문이 흐려진다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. REQ-001–REQ-018과 NFR-001–NFR-007의 본문은 유지하고 REQ-019–REQ-022만 추가한다.
2. 추적 행렬에 네 행을 추가하고 Test, Evidence, Gate는 TBD로 둔다.
3. PROJECT_CHARTER에 책임 경계, Maintenance-Agentic-Twin 경계, RISK-009를 추가한다.
   RISK-008의 기존 항목은 고치지 않는다. 7절과 10절 번호는 유지한다.
4. README의 현재 Gate를 P-000A.1로 바꾸고 실행 전 검증 질문을 적는다.
5. 거부한 제안: 기존 요구사항 문장 수정, 테스트 파일명 창작, FilterBot 파라미터 추정,
   PMAC 비공개 프로토콜 추정, 산업 안전 인증 주장, 진단 기능 편입,
   구현 파일 생성, P-000B 또는 P-001 착수, .cursor/ 커밋.

Developer Decision
개발자 지시에 따라 문서 네 개만 개정한다. 기존 기능 요구사항과 비기능 요구사항의 문장은 유지한다.
Hazardous Filter Handling Simulation은 일반화된 시나리오이며 독점 시스템의 복제가 아니다.
Pre-Execution Validation의 PASS는 프로젝트 내부 공학 검증 결과이다.
진단과 정비 판단은 Maintenance-Agentic-Twin에 둔다.
이 개정 초안의 최종 수용은 Developer 검수에 둔다.

Accepted / Rejected
Accepted — 지정된 네 요구사항, TBD 추적 행, 프로젝트 경계, RISK-009, README Gate 표기.
Rejected — 구현, 기존 REQ/NFR 본문 변경, 시험·증거 경로 창작, 프로젝트 통합, 인증 주장.
Pending — P-000A.1 초안에 대한 Developer 검수.

Reason
요구사항 본문을 바꾸면 P-000A FINAL PASS 기준선이 개정 범위 밖으로 움직인다.
AI가 시험 이름이나 물리 값을 채우면 아직 없는 증거를 있는 것처럼 만든다.
진단 기능을 같은 문서의 책임으로 넣으면 핵심 질문이 바뀐다.

Verification
2026-10-23 08:36 KST에 작업 트리에서 확인했다.
기준 HEAD는 bf876d22abdfb567b800ad49c966f27e4c5d7b92.
REQ 22, NFR 7, RISK 9.
REQUIREMENTS.md에서 삭제된 기존 REQ/NFR 본문 줄은 없다.
구현 파일은 0개다. git diff --check 는 공백 오류가 없다.
런타임, FK, IK, 충돌, 안전 영역은 측정하거나 구현하지 않았다.

Elapsed Time
2026-10-23 08:33 KST 수신, 08:36 KST 산출물 확인. 약 3분.
로컬 커밋은 이 기록 직후 1회. 구현 사이클 시간은 측정하지 않았다.
```

## P-000B — Architecture / Coordinate / Canonical API Baseline

```text
Task
P-000B. 기준선 62b9a624083cb6ac584e4eb6fd6a69c8c56197ee 위에서
REQ-001–REQ-022를 구현 가능한 아키텍처 계약으로 옮긴다.
로봇 모델과 시뮬레이터는 구현하지 않는다. P-001을 시작하지 않는다.

Problem
요구사항은 명령, 좌표계 일치, 실행 전 검증, 증거를 요구하지만
프레임 표기, 회전 저장 형식, Core와 Adapter의 경계, 검증 집계, 증거 보존이
한 계약으로 고정되어 있지 않다.
엔진 좌표를 추정해 섞으면 이후 교차 검증이 성립하지 않는다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. Canonical coordinate convention.
   ROS REP-103 (https://reps.openrobotics.org/rep-0103/ , 조회 2026-10-23)의
   SI 단위, 오른손, 몸체 축 x forward / y left / z up,
   optical 축 z forward / x right / y down을 계약으로 명시한다.
   Unity와 Isaac의 변환 행렬은 만들지 않는다.
2. Transform notation.
   T_A_B는 source B, destination A다.
   p_A = T_A_B * p_B 이고 T_A_C = T_A_B * T_B_C 다.
3. Quaternion policy.
   저장과 전송은 Hamilton quaternion (x, y, z, w)다.
   Euler는 HMI 표시에만 쓰고, 축 순서와 intrinsic/extrinsic과 단위를 매번 적는다.
   기본 Euler 순서는 잠그지 않는다. q와 -q의 대표 부호는 잠그지 않는다.
4. Core / Adapter boundary.
   core는 네 SDK에 종속되지 않는다.
   Unity, Isaac, ROS2, PMAC는 IRobotController 아래의 대등한 adapter다.
   시뮬레이터와 PMAC는 Canonical State의 정의자가 아니다.
5. Controller abstraction.
   IRobotController는 개념 인터페이스다. 언어 파일은 만들지 않는다.
   상태 어휘만 고정하고 완전한 전이표는 만들지 않는다.
   PMAC 레지스터, 문법, 메모리 맵, 비공개 상태는 추정하지 않는다.
6. Validation aggregation.
   required FAIL은 전체 FAIL이고 실행을 막는다.
   required NOT_EVALUATED는 전체 PASS를 금지한다.
   required 목록이 없으면 전체 PASS를 주지 않는다.
   측정값과 threshold를 만들지 않는다.
7. Evidence policy.
   동작했다는 서술만으로 PASS하지 않는다.
   PASS 파일을 덮어쓰지 않고 실패 기록을 지우지 않는다.
   myCobot 계열 숫자와 20 ms 한도를 이 저장소의 PASS로 쓰지 않는다.
8. 거부한 제안: 기존 REQ/NFR 수정, 차터 그림 수정, UR10e 소스 선택,
   변환 행렬 작성, 구현 파일, 테스트 실행, P-001 착수.

Developer Decision
개발자 지시에 따라 새 문서 다섯 개와 이 로그만 변경한다.
좌표 계약은 REP-103을 프로젝트 규칙으로 명시하되 엔진 행렬은 비운다.
T_A_B의 방향은 source B에서 destination A로 잠근다.
Quaternion을 저장 표현으로 두고 Euler는 표시용으로 제한한다.
기존 요구사항 문장은 바꾸지 않는다.
차터 3절 그림과의 어댑터 배치 차이는 Open Issue로 남긴다.
이 초안의 최종 수용은 Developer 검수에 둔다.

Accepted / Rejected
Accepted — 위 일곱 계약, TBD에 해당하는 수치와 소스의 공백, Evidence 불변 규칙.
Rejected — 엔진 좌표 추정, PMAC 프로토콜 추정, 지연 한도 창작, myCobot 증거 재사용,
           요구사항 파일 수정, 구현, P-001 시작.
Pending — P-000B 초안에 대한 Developer 검수.
           STOP/ESTOP을 검증 FAIL의 예외로 둘지, required validator 목록,
           Trajectory Completion과 Joint Limit Violation을 요구사항 목표 표에 넣을지.

Reason
표기 방향을 비우면 이후 구현이 서로 다른 행렬을 같은 이름으로 부른다.
REP-103에 없는 Unity 행렬을 지금 적으면 실측 전의 추정이 계약이 된다.
기존 요구사항을 고쳐 새 목표를 넣으면 P-000A.1 PASS 문장이 이 게이트에서 바뀐다.
AI 제안은 개발자 지시와 원문 규칙을 문서에 옮긴 것이며, 측정으로 검증된 설계가 아니다.

Verification
기준 HEAD는 62b9a624083cb6ac584e4eb6fd6a69c8c56197ee.
허용 변경은 docs 다섯 개 신규와 이 로그다.
REQ와 NFR 파일은 수정하지 않는다.
구현 파일은 만들지 않는다.
FK, 좌표 변환, 지연은 측정하지 않았다.

Elapsed Time
2026-10-23 08:40 KST 수신. 문서 작성과 확인은 한 세션이다.
로컬 커밋은 이 기록 직후 1회. 구현 사이클 시간은 측정하지 않았다.
```

## P-001A — UR10e Source Selection & Model Pinning

```text
Task
P-001A. UR10e Golden Reference의 official source와 정확한 upstream commit을 핀한다.
FK와 시뮬레이터 import는 하지 않는다.

Problem
브랜치 이름만 적으면 이후 jazzy가 움직였을 때 같은 모델을 다시 만들 수 없다.
비공식 URDF나 특정 장비 calibration을 nominal 설명과 섞으면 비교 기준이 바뀐다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. 공식 저장소 UniversalRobots/Universal_Robots_ROS2_Description를 고른다.
   ROS baseline이 Jazzy이므로 검토 브랜치는 jazzy다.
2. 조회 시점의 브랜치 팁 커밋을 SHA로 핀한다. 브랜치 이름만 기록하지 않는다.
3. 저장소 루트가 패키지 ur_description이다. ur_description/ 하위 경로는 만들지 않는다.
4. 설명은 official nominal UR10e description이다.
   default_kinematics.yaml의 hash 필드는 소스 필드로만 적고,
   특정 실제 장비의 calibration으로 부르지 않는다.
5. 라이선스는 package.xml의 BSD-3-Clause와 LICENSE 본문,
   Graphical Documentation 주석의 적용 목록을 그대로 적는다.
   UR10e mesh에 LICENSE.txt가 없다는 사실을 제약 없음으로 쓰지 않는다.
6. 거부한 소스: 비공식 URDF 저장소.
   거부한 작업: FK, IK, Unity, Isaac, ROS2 controller, PMAC, trajectory,
   collision 구현, Long-Reach, HMI, Vision, benchmark, myCobot 증거 재사용,
   xacro 수정, P-001B.

Developer Decision
개발자 지시에 따라 공식 jazzy 팁 7ce98d8c3512b9d0239da93b75de68ea64f61bd6을 핀한다.
UR10e 설명 입력과 mesh만 수정 없이 복사하고 SHA-256을 남긴다.
물리 YAML의 숫자는 소스 값으로만 적고 검증된 거동으로 적지 않는다.

Accepted / Rejected
Accepted — 공식 저장소, jazzy, 조회된 SHA, nominal 경계, 소스에서 읽은 joint와 frame.
Rejected — 비공식 URDF, 장비 calibration 주장, 제약 없음 해석, FK와 import, P-001B.

Reason
공식 description이 아니면 이후 Canonical Model과 FK 비교의 기준 커밋이 재현되지 않는다.
브랜치는 이동하므로 SHA가 필요하다.
패키지가 싣는 기본 YAML과 컨트롤러에서 추출하는 calibration 파일은 소스가 서로 다른 입력이라고 적혀 있다.

Verification
2026-10-23T00:04:38Z에 refs/heads/jazzy가 7ce98d8c3512b9d0239da93b75de68ea64f61bd6였다.
entry point, UR10e config 네 파일, joint 6개, 요청 frame, LICENSE를 그 커밋에서 확인했다.
로컬 복사 23개 파일의 shasum -a 256 -c는 통과했다.
FK 구현, Unity, Isaac, P-001B는 만들지 않았다.
기록은 robot_description/SOURCE_MANIFEST.md와 evidence/P-001A/source_selection.md다.

Elapsed Time
2026-10-23 09:02 KST 수신. 조회와 문서 작성은 한 세션이다.
구현 사이클 시간은 측정하지 않았다.
```

## P-001A.1 — Bolt Use Case Controlled Requirements Amendment

```text
Task
P-001A.1. HELD였던 대표 Use Case를 요구사항과 아키텍처에 정식 반영한다.
구현은 하지 않는다.

Problem
Hazardous Filter Handling이 대표 구현 시나리오처럼 남아 있으면
이후 Workcell과 Tool과 HMI가 필터 전체 공정으로 넓어진다.
Pre-Execution 성공과 체결 완료를 한 단어로 부르면 시도 허용이 작업 성공이 된다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. 대표 구현을 Generic Industrial Bolt Fastening / Loosening으로 좁힌다.
   필터 작업 문장은 도메인 동기로 남기고 삭제하지 않는다.
2. MVP Tool은 Bolt Fastening / Loosening Tool 하나다.
   필터 그리퍼, 커버, 운반, 검사, Automatic Tool Changer는 구현 대상이 아니다.
3. Pre-Execution 결과는 SAFE_TO_ATTEMPT 또는 REJECTED다.
   Execution 결과는 FASTENED, LOOSENED, FAILED, FAULT다.
4. Torque는 Pre-Execution REQUIRED 조건이 아니다.
   Execution feedback이며, 초기 값은 Virtual Fastening Process와 Virtual Torque Sensor다.
   실제 제품 Torque는 추정하지 않는다.
   숫자가 필요하면 SIMULATION TEST PARAMETER 또는 PROJECT TEST PARAMETER다.
5. HMI와 Evidence는 simulated/virtual임을 식별한다.
6. 거부한 작업: FK, IK, Bolt/Torque simulator, Tool 모델, URDF 수정,
   Unity, Isaac, ROS2, PMAC, HMI, trajectory, collision, Long-Reach, P-001B.
   REQ-001–REQ-022와 NFR-001–NFR-007은 삭제하거나 번호를 바꾸지 않는다.

Developer Decision
개발자 지시에 따라 975a01c의 HELD 결정을 정식 반영한다.
기존 번호는 유지하고 REQ-023–REQ-028을 추가한다.
BOLT_FASTEN과 BOLT_LOOSEN의 REQUIRED validator 9개는 모두 필수이고 OPTIONAL은 만들지 않는다.

Accepted / Rejected
Accepted — 대표 Use Case 축소, Tool 하나, 두 결과 축의 분리, virtual Torque 표시, 제품 Torque 미사용.
Rejected — 필터 전체 공정 구현, 제품 사양 추정, proprietary protocol, 이번 게이트의 구현과 P-001B.

Reason
필터 전체 공정은 대표 데모의 범위를 공구 여러 개와 미확인 제품 순서로 넓힌다.
Bolt Tool 하나만 두면 Tool TCP와 충돌과 승인의 경계를 한 공구로 잠글 수 있다.
SAFE_TO_ATTEMPT는 시도 허용이고 FASTENED는 실행 후 완료라서 같은 PASS가 아니다.
Torque는 시도 전에 성공을 보장하지 못한다. 실행 중 피드백으로만 완료를 판정한다.
실제 제품 Torque는 알 수 없고, 추정하면 시험 조건이 장비 사양처럼 남는다.
Virtual 표시가 없으면 그 시험 조건을 센서 측정으로 읽는다.

Verification
REQ-001–REQ-022와 NFR-001–NFR-007의 번호는 유지했다.
REQ-023–REQ-028과 추적 표를 추가했다.
대표 정상 흐름과 Tool Collision FAIL → REJECTED → EXECUTION BLOCKED를 문서에 적었다.
Torque 숫자는 만들지 않았다.
FK, Unity, Isaac, P-001B 파일은 만들지 않았다.

Elapsed Time
2026-10-23 09:37 KST 수신. 문서 개정은 한 세션이다.
구현 사이클 시간은 측정하지 않았다.
```

## P-002 Fix Attempt 2 — Unity joint accuracy

```text
Task
P-002 Fix Attempt 2. Unity UR10e joint error를 진단하고, 설명 가능한 수정 뒤
같은 5개 configuration을 다시 측정한다. P-003은 시작하지 않는다.

Problem
Attempt 2A의 global max는 0.0026187931653112173 rad, 0.15004579579003846 deg였다.
Engineering Target은 Joint Target Error < 0.1 deg다. 5개 configuration이 모두 timeout이었다.
가장 큰 오차는 shoulder_lift_joint였다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. Parameter를 바꾸기 전에 G-ON, G-OFF, root movable, collider enabled를 측정한다.
2. driveForce와 forceLimit을 비교한다. 값이 없으면 만들지 않는다.
3. stiffness/damping은 관측 뒤 3×3 유한 행렬만 본다.
4. 거절: gravity OFF를 최종 PASS로 쓰기, 무한 stiffness, 무한 force limit,
   actual=target, timeout 연장, 실패 configuration 제외, Engineering Target 변경.

Developer Decision
진단 측정 뒤에 solver position iterations만 6에서 24로 바꾼다.
stiffness 1000000, damping 2000, URDF force limit, gravity ON, timeout 8 s,
settle 규칙은 유지한다. base_link immovable은 prefab에 명시한다.
collider 7개는 진단에서 오차가 같았으므로 끈 상태를 유지한다.

Accepted / Rejected
Accepted — solver position iterations 24. base_link immovable을 prefab에 기록.
Rejected — gravity OFF PASS, force limit 변경, stiffness/damping 변경,
timestep 0.005를 최종 조건으로 사용, velocity iterations 4, 무한 강성, 무한 force limit.

Reason
G-OFF에서 shoulder_lift 오차와 -156 Nm drive force가 사라졌다.
G-ON에서 |drive force| / 330 = 0.473이라 saturation이 아니었다.
stiffness 1000000–3000000, damping 500–8000은 오차 바닥을 0.134 deg 아래로 내리지 못했다.
solver iterations 6, 12, 24에서 A-zero 오차는 0.150, 0.053, 0.016 deg였다.
iterations 24만 reported velocity도 0.001 rad/s 아래였다.
timestep 0.005는 오차를 줄였지만 velocity 기준을 넘지 못했다.

Verification
최종 5개 configuration, timeout 0, global max 0.0006152464775368571 rad,
0.035251026523152325 deg. gravity ON. feedback은 articulation jointPosition이다.
Attempt 1의 0.014747420325875282 rad와 Attempt 2A의 0.0026187931653112173 rad는 유지했다.
이 결과는 UR10e controller 재현이 아니다.

Elapsed Time
2026-10-23 진단과 재시험은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-003 — Isaac UR10e joint control baseline

```text
Task
P-003. 같은 Golden Reference UR10e를 Isaac Sim headless에서 불러
canonical joint command와 PhysX joint feedback의 오차를 측정한다.
Unity, Python FK와 비교하지 않는다. P-004는 시작하지 않는다.

Problem
Attempt 1은 import 뒤 stage가 열려 있지 않아 articulation root가 0개였다.
Attempt 1b는 open_stage가 (bool, stage)를 반환하는데 stage로 사용했다.
Attempt 2는 root를 찾았지만 reset_to_default_state가 set_default_state 없이는
아무 일도 하지 않았다. shoulder_lift_joint가 이전 자세를 물려받아
global max 95.9492830527047 deg, timeout 1이었다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.
실행 호스트는 p620. Isaac Sim VERSION 파일은 6.2.0-rc.14+release.52805.02189a84.gl.

AI Proposal
1. import_urdf가 반환한 USDA를 open_stage의 성공 플래그와 함께 연다.
2. configuration마다 DOF position을 command로 옮기고 velocity를 0으로 둔 뒤
   position target을 걸고, settling 이후 physics tensor를 읽는다.
3. 거절: 0.1 deg 완화, gravity OFF PASS, actual=target, Unity 우열, Python FK mm claim,
   solver attribute가 없을 때 스키마 기본값 32를 사용한 것처럼 기록.

Developer Decision
위 1–3을 수용한다. stiffness 100000 Nm/rad, damping 1000 Nm*s/rad,
max effort는 URDF effort, gravity는 runner가 만든 Z-up 9.81, timeout 8 s,
settle 20 step을 유지한다. 이 drive는 PROJECT TEST PARAMETER다.

Accepted / Rejected
Accepted — stage를 다시 열고, configuration을 독립된 hold로 측정한다.
Rejected — Engineering Target 변경, gravity OFF, 무한 stiffness, P-004, IK.

Reason
A-zero와 B-positive-1은 0.1 deg 아래였고, 실패한 관절은 shoulder_lift뿐이었다.
reset이 없어 넘어진 자세에서 effort 330 Nm로 복귀하지 못했다.
command 자세에서 다시 잡으면 같은 drive의 hold 오차가 0.1 deg 아래였다.
USD stiffness 1745.3292236328125는 authored attribute이고,
get_dof_gains readback은 100000이다. solver iteration attribute는 stage에 없다.

Verification
Headless. configuration 5. timeout 0.
global max 0.0015286624431610218 rad, 0.08758590629328365 deg.
deterministic rerun max abs 3.3173018891829997e-06 rad.
tool0 frame isaac_world, quaternion wxyz.
PYTHONPATH=. python3 -m pytest tests/unit -q → 17 passed.
Attempt 1, 1b, 2 evidence는 남겼다.
이 결과는 UR10e controller 재현이 아니고 Unity와의 비교가 아니다.

Elapsed Time
2026-10-23 측정과 수정은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-003 Fix Attempt 1 — tool0 state integrity

```text
Task
P-003 Fix Attempt 1. joint feedback과 tool0 world pose가 같은 physical state인지 확인한다.
joint gain은 바꾸지 않는다. P-004는 시작하지 않는다.

Problem
기존 P-003은 joint global max 0.08758590629328365 deg, timeout 0으로 숫자 PASS였다.
tool0 world z는 A-zero 0.057508014142513275에서 E-fixed-3 -40.485015869140625까지
configuration마다 내려갔다. xy와 quaternion은 거의 그대로였다.
joint metric PASS를 전체 PASS로 보지 않는다. 그 evidence는 evidence/P-003 루트에 남긴다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.
실행 호스트는 p620. Isaac Sim VERSION 파일은 6.2.0-rc.14+release.52805.02189a84.gl.

AI Proposal
1. 수정 전에 link chain, fresh state, RigidPrim(tool0) free-fall을 따로 측정한다.
2. tool0는 RigidBodyAPI가 없는 Xform이므로 UsdGeom.XformCache로 읽는다.
   RigidPrim 생성자는 없는 API를 적용하고, 그 body는 중력으로 떨어진다.
3. 거절: tool0 evidence 삭제, Python FK로 Isaac pose 교정, 0.1 deg 완화, drive retune, P-004.

Developer Decision
위 1–3을 수용한다. articulation root는 base_link_inertia로 유지한다.
PROJECT SANITY CHECK tool0-base distance limit은 2.0 m다.
UR10e 공식 reach 1300 mm보다 위이고, FK accuracy target이 아니다.
adjacent link span limit은 0.001 m다.

Accepted / Rejected
Accepted — XformCache 관측과 sanity/integrity 실패 조건을 재측정에 넣는다.
Rejected — RigidPrim(tool0), root 임의 변경, Python FK 비교, P-004.

Reason
Xform chain은 다섯 configuration에서 같이 움직이고 adjacent distance span은
2.3841857632600494e-07 m다. fresh와 continuous USD z는 같다.
RigidPrim(tool0) 이후 joint command 없이 43 update를 반복하면 z가
0.051248036324977875, -2.526602029800415, -10.142973899841309,
-22.79786491394043, -40.491275787353516으로 내려간다.
기존 실패와 같은 자유낙하이고, joint state와 tool0 관측이 다른 body였다.

Verification
evidence/P-003/fix-attempt1. headless. configuration 5. timeout 0.
global max 0.0015286624431610218 rad, 0.08758590629328365 deg.
tool0 base distance max 1.3166888748578596 m. cumulative_world_drift false.
A-zero fresh rerun joint, tool0 position, tool0 orientation 차이는 0.0.
pose API는 UsdGeom.XformCache.GetLocalToWorldTransform. frame isaac_world. quaternion wxyz.
Attempt 1, 1b, 2와 루트 joint_results.json은 덮어쓰지 않았다.
이 결과는 UR10e controller 재현이 아니고 Unity 또는 Python FK와의 비교가 아니다.

Elapsed Time
2026-10-23 진단과 재측정은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-004 — Cross-simulator tool0 TCP

```text
Task
P-004. 같은 UR10e Golden Reference와 같은 5개 canonical joint configuration에서
독립 Python FK, Unity, Isaac의 tool0 pose를 base_link 기준으로 비교한다.
P-005는 시작하지 않는다.

Problem
P-001B는 FK 구현, P-002는 Unity joint, P-003은 Isaac joint와 tool0 자유낙하를 봤다.
세 TCP가 같은 canonical pose인지는 아직 측정하지 않았다.
P-003은 joint metric이 맞아도 tool0 world pose가 틀릴 수 있음을 남겼다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.
Unity는 이 Mac의 2022.3.62f2. Isaac은 p620, VERSION 6.2.0-rc.14+release.52805.02189a84.gl.

AI Proposal
1. raw pose는 각 runtime이 만들고, 비교 스크립트는 raw를 수정하지 않는다.
2. Unity는 importer hash 90f353e4352aae4df52fa2c05e49b804631d2a63의 Unity2Ros로
   canonical 오른손 Z-up에 맞춘다. Isaac tool0는 XformCache만 읽는다.
3. 거절: Python FK로 simulator pose를 대체, 0.5 mm 또는 1.0 mm 완화, FK 파라미터 수정, P-005.

Developer Decision
위 1–3을 수용한다. Python↔simulator는 FK Position < 0.5 mm, FK Orientation < 0.1 deg를
actual joint 입력에 적용한다. Unity↔Isaac position은 < 1.0 mm다.
Unity↔Isaac orientation acceptance는 REQUIREMENTS와 VALIDATION_PLAN에 없어 만들지 않는다.

Accepted / Rejected
Accepted — actual joint FK와 target joint end-to-end를 분리해 기록한다.
Rejected — tolerance 완화, silent offset, RigidPrim(tool0), P-005.

Reason
Unity TCP와 Python FK(Unity actual joint)의 최대 위치 오차는 0.004052324538793909 mm다.
Isaac XformCache tool0와 Python FK(Isaac actual joint)의 최대 위치 오차는
1.4705603411282602 mm이고 자세는 0.066672609317108195 deg다.
축이 바뀐 크기가 아니다. Unity↔Isaac 최대 위치 오차는 3.1548989314166063 mm다.
0.5 mm와 1.0 mm를 넘으므로 P-004는 FAIL이다.

Verification
evidence/P-004. configuration 5.
A-zero rerun의 joint, tool0 position, tool0 orientation 차이는 Unity와 Isaac 모두 0.
PYTHONPATH=. python3 -m pytest tests/unit -q → 22 passed.
P-001B, P-002, P-003 evidence는 수정하지 않았다.
이 결과는 실제 UR10e 정확도가 아니다.

Elapsed Time
2026-10-23 측정과 비교는 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-004 Fix Attempt 1 — Isaac transform chain

```text
Task
P-004 Fix Attempt 1. Python FK와 Isaac tool0의 약 1.47 mm 차이가
base_link부터 tool0까지 어느 frame에서 처음 생기는지 측정한다.
원인을 측정하기 전에는 FK, URDF, drive, timestep, tolerance를 바꾸지 않는다.
P-005는 시작하지 않는다.

Problem
P-004 FAIL의 kinematic 차이는 Python ↔ Isaac position 1.4705603411282602 mm,
orientation 0.066672609317108195 deg다. Unity는 같은 FK와 0.0040523245387939092 mm다.
최종 TCP만 다시 재면 발생 joint를 알 수 없다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.
Isaac은 p620, VERSION 6.2.0-rc.14+release.52805.02189a84.gl.
Importer helper는 설치된 fix_articulation_root_for_fixed_base다.

AI Proposal
1. 같은 Isaac actual joint로 chain_frames와 XformCache chain을 비교한다.
2. Authored USD local이 Python과 같으면 runtime root를 본다.
3. 거절: 1.36 mm Z offset, tolerance 완화, drive 변경, FK geometry 변경, P-005.

Developer Decision
위 1–3을 수용한다. 측정된 원인은 base_link_inertia에 남은 ArticulationRootAPI다.
수정은 그 API를 world-anchoring fixed joint로 옮기는 것이다.

Accepted / Rejected
Accepted — fix_articulation_root_for_fixed_base를 P-004 Isaac runner에 적용한다.
Rejected — empirical offset, drive retune, tolerance 변경.

Reason
relocation 전 A-zero post-settle orientation error는 base_link_inertia에서
0.06468463585198597 deg이고 tool0 position error는 1.363292 mm다.
Authored local orientation error는 0 deg다.
relocation 후 Python ↔ Isaac position maximum은 0.00075148570272387397 mm다.
Unity ↔ Isaac position maximum은 1.2526456740680483 mm이고, 이 값은
두 simulator의 settled joint로 계산한 FK 차이와 같다. 1.0 mm를 넘으므로 FAIL이다.

Verification
evidence/P-004/fix-attempt1. 기존 evidence/P-004 루트는 덮어쓰지 않았다.
A-zero rerun의 joint, tool0 position, tool0 orientation 차이는 0이다.
PYTHONPATH=. python3 -m pytest tests/unit -q → 23 passed.
이 결과는 실제 UR10e 정확도가 아니다.

Elapsed Time
2026-10-23 진단과 재측정은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-004 Fix Attempt 2 — settled joint alignment

```text
Task
P-004 Fix Attempt 2. 같은 canonical target에서 Unity와 Isaac의 settled actual joint가
왜 달라지고, 그 차이가 Unity ↔ Isaac 1.2526456740680483 mm를 설명하는지 측정한다.
P-005는 시작하지 않는다.

Problem
Fix Attempt 1 이후 Python ↔ Unity와 Python ↔ Isaac transform은 PASS다.
남은 차이는 두 simulator의 TCP다. A-zero shoulder_lift는 Unity 0.0002858018851839006 rad,
Isaac 0.0012141603510826826 rad다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.
Isaac은 p620, VERSION 6.2.0-rc.14+release.52805.02189a84.gl.

AI Proposal
1. FK(actual q)와 simulator TCP의 벡터 차로 joint settling을 먼저 귀속한다.
2. Gravity OFF와 stiffness step은 diagnostic으로만 측정하고, gravity OFF로 PASS하지 않는다.
3. 거절: TCP tolerance 증가, target offset, FK 수정, solver 기본값을 추측해 적기, P-005.

Developer Decision
위 1–3을 수용한다. 측정된 원인은 gravity 아래 shoulder_lift PD equilibrium이다.
Isaac importer stiffness만 100000에서 200000 Nm/rad로 올린다. Damping은 1000을 유지한다.

Accepted / Rejected
Accepted — override_joint_stiffness 200000.
Rejected — empirical TCP offset, tolerance 변경, solver iteration 숫자를 만들지 않은 채 기록, gravity OFF acceptance.

Reason
Attribution residual 최대는 0.003836883818283858 mm다.
A-zero gravity compensation shoulder_lift는 -121.28190612792969 Nm이고 max effort는 330 Nm이다.
Gravity magnitude 0에서 joint error는 0이다.
Stiffness 200000에서 A-zero shoulder_lift error는 0.0005065396544523537 rad이고 timeout은 false다.
Solver iteration attribute는 authoring되어 있지 않고 set은 typeName이 비어 실패했다.
재측정 Unity ↔ Isaac position 최대는 0.29056228373221366 mm다.

Verification
evidence/P-004/fix-attempt2. evidence/P-004와 fix-attempt1은 덮어쓰지 않았다.
A-zero rerun difference는 0이다. Timeout은 0이다.
Unity joint error 최대 0.035251026523152325 deg, Isaac 0.029022584356134966 deg.
PYTHONPATH=. python3 -m pytest tests/unit -q → 23 passed.
이 결과는 실제 UR10e controller tuning이 아니다.

Elapsed Time
2026-10-23 진단과 재측정은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-005 — industrial workcell physics baseline

```text
Task
P-005. 검증된 nominal UR10e를 재현 가능한 industrial workcell에 배치하고
frame, physics, collision, reset, stability를 측정한다.
P-006은 시작하지 않는다.

Problem
Long-reach 모델의 source가 없다. Bolt 공정, IK, trajectory, safety zone은 이번 Gate 밖이다.
Zero pose에서 tool0는 base_link 기준 약 x=1.184 m, y=0.291 m, z=0.061 m라서
전방 panel이 팔 높이에 있으면 command 없이 penetration이 생긴다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.
Isaac은 p620. PhysX contact report와 raycast_closest는 설치된 omni.physx 110.4.4 binding에서 확인했다.

AI Proposal
1. Provenance가 있는 robot만 쓴다. 없으면 UR10e-based reference workcell로 둔다.
2. Isaac을 physics baseline으로 하고 Unity workcell은 복제하지 않는다.
3. Scene는 workcell/config/workcell.json에서 다시 만든다.
4. 거절: FilterBot, IK, torque, trajectory, safety-zone engine, P-004 stiffness 변경.

Developer Decision
위 1–4를 수용한다. Robot은 pinned nominal UR10e다.
Floor, pedestal, posts, vertical bolt panel, B01–B06, tool placeholder를 PROJECT TEST GEOMETRY로 둔다.
Panel은 zero-pose collision mesh AABB 밖에 둔다. 접촉은 PhysX contact report로 센다.

Accepted / Rejected
Accepted — Option A, Isaac primary, stiffness 200000, damping 1000, articulation root relocation, gravity ON.
Rejected — generic long-reach model, FilterBot claim, bolt fastening, IK, Unity 중복 scene.

Reason
xformOpOrder에 scale을 translate보다 앞에 두면 translation이 scale된다.
그 배치의 tool center 오차는 0.08460000407771068 m였다.
Translate를 앞으로 둔 재측정 오차는 1.7402633963655867e-08 m다.
10 s, 600 steps, dt 0.016666666666666666 s.
Base displacement 0 m. Max joint drift 3.637978807091713e-10 rad.
Tool0 drift 0 m. Unexpected collisions 0. Initial penetrations 0.
Reset joint difference 1.2514647096395493e-09 rad.
Build position difference 0 m.
Directional raycast clearance 0.02761254273355007 m.
Robot base에서 panel frame까지 1.2757742747053649 m.
Nominal reach 1300 mm와 비교한 sphere 판정은 geometric sanity이며 IK가 아니다.
Gravity readback magnitude는 9.8100004196167이다. 입력은 9.81이다.

Verification
evidence/P-005. evidence/P-001B, P-002, P-003, P-004는 덮어쓰지 않았다.
PYTHONPATH=. python3 -m pytest tests/unit -q → 30 passed.
P-004 runner의 DRIVE_STIFFNESS 200000과 DRIVE_DAMPING 1000은 유지했다.
이 결과는 현장 안전 검증도, bolt fastening 검증도, FilterBot도 아니다.
REQ-019의 camera, lighting, restricted zone은 이번 baseline에 없다.

Elapsed Time
2026-10-23 구현과 측정은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-006 — virtual PMAC-compatible controller baseline

```text
Task
P-006. Canonical MOVE_JOINT를 받는 VirtualPmacController를 만들고
lifecycle, slew, fault, reset, replay를 측정한다.
P-007은 시작하지 않는다.

Problem
P-000B의 controller_state 어휘와 P-006 프롬프트의 ENABLED/EXECUTING/FAULT/JOINT_POSITION이 다르다.
실제 Power PMAC 명령과 레지스터는 공개된 구현 근거가 없다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. controller_state는 REQ-011 이름만 쓴다. enable은 READY, 동작 중은 RUNNING, fault는 ERROR다.
2. 관절 명령 타입은 MOVE_JOINT다. JOINT_POSITION이라는 둘째 schema는 만들지 않는다.
3. 동작은 관절별 sourced velocity limit의 deterministic slew다.
4. 거절: real PMAC protocol, Isaac/Unity/ROS 호출, MOVE_TCP, IK, fault reset 시 joint zeroing.

Developer Decision
위 1–4를 수용한다. reset destination은 CONNECTED 하나다.
EXECUTING 중 새 명령은 REJECT_BUSY다.
servo_on은 READY로 들어가고 SERVO_ON은 이 baseline의 체류 상태가 아니다.

Accepted / Rejected
Accepted — VirtualPmacController, MOVE_JOINT, project slew, TEST_FAULT injection.
Rejected — Power PMAC compatibility claim, command teleport, P-007.

Reason
다섯 configuration의 최종 joint error는 0 rad다.
Replay 두 실행의 final q 차이는 0 rad다.
Fault reset 전후 joint position은 같다.
Timeout 0, non-finite 0.
속도 한도는 pinned UR10e joint_limits.yaml의 max_velocity다.

Verification
evidence/P-006. evidence/P-001B부터 P-005는 덮어쓰지 않았다.
PYTHONPATH=. python3 -m pytest tests/unit -q → 37 passed.
이 결과는 실제 Power PMAC timing, protocol, motion behavior가 아니다.

Elapsed Time
2026-10-23 구현과 측정은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-007 — ROS 2 controller adapter baseline

```text
Task
P-007. Ros2ControllerAdapter로 ROS 2 transport와 VirtualPmacController를 연결한다.
P-006 command semantics는 유지한다. P-008은 시작하지 않는다.

Problem
이 Mac에는 ROS 2가 없다. JointState에는 command_id, controller_state, fault가 없다.
P-006의 MOVE_JOINT와 REQ-011 상태를 ROS 메시지 정의로 바꾸면 안 된다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.
JointState 필드: https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/JointState.msg
QoS default: https://raw.githubusercontent.com/ros2/rmw/jazzy/rmw/include/rmw/qos_profiles.h
rmw_qos_profile_default는 KEEP_LAST, depth 10, RELIABLE, VOLATILE이다.

AI Proposal
1. 실행 호스트는 p620 Ubuntu 24.04 / ROS 2 Jazzy로 실측한다.
2. /joint_states는 actual_q만, target은 다른 topic에 둔다.
3. command_id와 controller state는 std_msgs/String JSON으로 옮기고 canonical type은 MOVE_JOINT로 둔다.
4. QoS는 rmw_qos_profile_default를 측정 전에 고정한다. sensor_data best effort는 쓰지 않는다.
5. 거절: ros2_control, MoveIt, Isaac, 카메라, 실제 로봇, real-time 인증, PMAC protocol.

Developer Decision
위 1–5를 수용한다. step dt는 0.01 s로 주입한다. wall clock은 latency와 feedback rate 측정에만 쓴다.
shutdown은 disconnect이며 E-stop이 아니다. fault reset destination은 CONNECTED다.

Accepted / Rejected
Accepted — 별도 프로세스 publisher와 adapter, 이름 기준 joint reorder, REJECT_BUSY, TEST_FAULT.
Rejected — ROS 메시지를 command model로 두는 것, target을 /joint_states에 싣는 것, P-008.

Reason
다섯 configuration의 최종 joint error는 0 rad다.
/robotwin/state published 200, received 200, lost 0.
/joint_states도 published 200, received 200, lost 0.
observed rate는 100.00364838337862 Hz이고 configured rate 100 Hz는 결과에 맞춰 바꾸지 않았다.
Latency 100표본의 정의는 command publish에서 ACCEPTED feedback까지다.

Verification
evidence/P-007. 실행 호스트는 Ubuntu 24.04.4, ROS_DISTRO jazzy, Python 3.12.3이다.
Isaac Sim과 Unity는 다시 실행하지 않았다. evidence/P-001B부터 P-006은 수정하지 않았다.
p620에서 PYTHONPATH=. python3 -m pytest tests/unit -q → 41 passed.
이 결과는 real-time 인증도, 실제 UR10e 검증도, 실제 Power PMAC 검증도 아니다.

Elapsed Time
2026-10-23 구현과 측정은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-008 — Pre-execution validator baseline

```text
Task
P-008. MOVE_JOINT를 Controller에 전달하기 전에 Pre-Execution Validator로 검사한다.
P-009는 시작하지 않는다.

Problem
위험한 명령을 Controller 안에서 거절하는 것과, 전달 전에 차단하는 것은 다르다.
SAFE_TO_ATTEMPT를 체결 성공이나 안전 인증으로 쓰면 안 된다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. Validator는 VirtualPmacController 밖의 ExecutionGate다.
2. 충돌은 pinned UR10e collision STL과 P-005 workcell box로 계산한다.
3. Trajectory sample count는 100으로 측정 전에 고정한다.
4. MOVE_JOINT에는 bolt target이 없으므로 alignment와 IK는 NOT_APPLICABLE이다.
5. 거절: Cartesian command, IK reachability 주장, self-collision stub PASS, ROS adapter 내부 복제, P-009.

Developer Decision
위 1–5를 수용한다. Joint limit 포함 규칙은 기존 limit_violations와 같다. 한계값은 안이다.
Safety zone은 bolt panel +X 뒤 0.4 m AABB이며 PROJECT TEST SAFETY ZONE이다.

Accepted / Rejected
Accepted — fail-closed REJECT, ExecutionGate submit count, 인접 link를 제외한 self-collision.
Rejected — canonical 5개를 하드코딩 PASS로 두는 것, 충돌 stub, 안전 인증 문장, P-009.

Reason
A-zero, B-positive-1, C-mixed-1, E-fixed-3은 SAFE_TO_ATTEMPT 뒤 ACCEPTED, EXECUTING, COMPLETED다.
최종 joint error는 0 rad다. step은 1, 23, 24, 2다.
E-fixed-1은 sample 44/100에서 wrist_3_link와 bolt_panel이 교차해 REJECT다. submit count는 0에서 0이다.
tool_placeholder와 bolt_panel 교차도 REJECT다.
Safety zone 위반 pose는 COLLISION과 SAFETY_ZONE을 함께 냈다.
NaN, Inf, unsupported command는 Controller에 전달되지 않았다.

Verification
evidence/P-008. evidence/P-001B부터 P-007은 수정하지 않았다.
Isaac Sim, Unity, P-007 ROS evidence는 다시 실행하지 않았다.
PYTHONPATH=. python3 -m pytest tests/unit -q → 52 passed.
Timeout 0, non-finite 0. Deterministic replay는 같았다.
이 결과는 산업 안전 인증이 아니다. SAFE_TO_ATTEMPT는 pre-execution checks passed다.
Canonical safe 결과는 4/5이므로 P-008 acceptance는 FAIL이다.

Elapsed Time
2026-10-23 구현과 측정은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-008 Fix Attempt 1 — E-fixed-1 collision investigation

```text
Task
E-fixed-1의 wrist_3_link vs bolt_panel sample 44/100이 실제 교차인지 판별한다.
Validator를 PASS로 만들기 위해 고치지 않는다. P-009는 시작하지 않는다.

Problem
Canonical configuration 5개가 모두 SAFE_TO_ATTEMPT여야 한다는 가정과,
P-005 workcell 안에서 그 궤적이 패널을 지날 수 있다는 사실이 섞여 있었다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. 같은 validate()로 sample 44를 재현한다.
2. P-008 scene은 resolve_workcell()이므로 P-005 evidence와 숫자로 비교한다.
3. SAT와 별개로 triangle edge clip과 box-edge intersection을 계산한다.
4. N=100/500/1000은 진단만 하고 production sample count는 100으로 둔다.
5. 거절: tolerance 확대, collision 제외, E-fixed-1 특수 처리, acceptance 자동 수정, P-009.

Developer Decision
위 1–5를 수용한다. Isaac contact는 동적 step 뒤 관절이라 이 joint vector의 oracle로 쓰지 않는다.

Accepted / Rejected
Accepted — TRUE_COLLISION이면 validator를 유지한다.
Rejected — AABB overlap만으로 충돌을 취소하는 것, panel이나 wrist_3를 검사에서 빼는 것.

Reason
재현은 sample 44/100, wrist_3_link, bolt_panel이다.
P-005와 P-008의 base, panel, floor, pedestal, post transform 차이는 0이다.
SAT와 exact triangle-box는 삼각형 3개에서 같고, 불일치는 0이다.
샘플된 최대 침투는 9.275217475473996e-05 m이고, clip된 edge 길이는 0.0006292049816059846 m다.
충돌 구간은 sample 44부터 75까지다.
N=100/500/1000의 첫 충돌 s는 0.4444444444444444, 0.4348697394789579, 0.4344344344344344다.
FK 위치 차이와 transform composition 차이는 0이다.

Verification
evidence/P-008-FIX1. evidence/P-008 summary의 FAIL은 덮어쓰지 않았다.
PYTHONPATH=. python3 -m pytest tests/unit/test_p008_validator.py tests/unit/test_p006_controller.py tests/unit/test_p007_convert.py -q → 22 passed.
Isaac scene은 이 joint vector로 재실행하지 않았다.
P-008 final status는 FAIL이다. Acceptance 문장은 이 commit에서 바꾸지 않았다.

Elapsed Time
2026-10-23 조사는 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-008 Fix Attempt 2 — acceptance reconciliation

```text
Task
E-fixed-1의 TRUE_COLLISION을 유지한 채, canonical configuration 5개가 모두 SAFE_TO_ATTEMPT여야 한다는 acceptance를 수정한다.
P-009는 시작하지 않는다.

Problem
P-008 최초 가정은 5개 canonical configuration이 workcell 안에서 collision-free trajectory라는 뜻으로 쓰였다.
FIX1은 E-fixed-1이 bolt panel과 실제로 교차한다고 확인했다. Validator defect는 아니었다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. A-zero, B-positive-1, C-mixed-1, E-fixed-3은 DESIGNATED_SAFE로 다시 측정한다.
2. E-fixed-1은 KNOWN_COLLISION으로 두고 REJECT를 유지한다.
3. 기존 negative case를 expected decision과 actual decision으로 비교한다.
4. 거절: tolerance 확대, collision 제외, sample 감소, required check optional화, historical evidence 수정, P-009.

Developer Decision
위 1–4를 수용한다. Canonical configuration 이름과 joint vector는 바꾸지 않는다.
Validator decision code와 collision config는 바꾸지 않는다.

Accepted / Rejected
Accepted — safe command만 SAFE_TO_ATTEMPT, unsafe command는 REJECT.
Rejected — E-fixed-1을 SAFE로 바꾸거나 designated safe 1건을 실패 후 reject case로 재분류하는 것.

Reason
Designated safe 4/4는 SAFE_TO_ATTEMPT 뒤 ACCEPTED, EXECUTING, COMPLETED다.
E-fixed-1은 sample 44/100에서 wrist_3_link와 bolt_panel로 REJECT다. submit은 증가하지 않았다.
Negative suite는 11/11 REJECT이고 controller_submitted는 false다.
NaN과 joint-limit도 controller에 전달되지 않았다.
동일 suite 2회 replay는 decision, reason, collision pair, sample, submission이 같았다.
Timeout 0, non-finite validator output 0.

Verification
evidence/P-008-FIX2. evidence/P-008과 evidence/P-008-FIX1은 수정하지 않았다.
PYTHONPATH=. python3 -m pytest tests/unit/test_ur10e_fk.py tests/unit/test_p001b_evidence.py tests/unit/test_p006_controller.py tests/unit/test_p007_convert.py tests/unit/test_p008_validator.py tests/unit/test_p008_fix2_acceptance.py -q → 44 passed.
P-008 result는 FINAL PASS다. 산업 안전 인증이 아니다.

Elapsed Time
2026-10-23 reconciliation은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-009 — Digital twin target vs actual

```text
Task
SAFE_TO_ATTEMPT 명령의 target joint와 actual joint를 project clock으로 기록하고, joint/TCP 오차와 settling을 계산한다.
E-fixed-1은 P-008 REJECT이면 tracking을 시작하지 않는다. P-010은 시작하지 않는다.

Problem
Controller COMPLETED와 monitor SETTLED를 같은 값으로 두면 단일 sample 수렴과 fault 이후 hold를 구분할 수 없다.
A-zero는 시작부터 target과 actual이 같아서 separation 증거가 되지 않는다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. Monitor는 controller와 validator 밖에 두고 ExecutionGate 통과 뒤에만 begin한다.
2. TCP는 target_q와 actual_q에 각각 P-001B FK를 적용한다. Orientation은 기존 shortest-angle 함수를 쓴다.
3. Settling은 1e-9 rad를 연속 3 sample 유지할 때다. COMPLETED 플래그로 대입하지 않는다.
4. 거절: target/actual 복사, Euler 차감, E-fixed-1 trace, HMI, 실측 정확도 주장, P-010.

Developer Decision
위 1–4를 수용한다. 각 case는 zero joint state의 새 controller에서 시작한다.
dt는 P-006의 0.01 s를 유지한다.
FIX2 acceptance의 p009_not_started는 이후 evidence 디렉터리 존재가 아니라 FIX2 출력 경로를 보게 고친다. evidence/P-008-FIX2 파일은 다시 쓰지 않는다.

Accepted / Rejected
Accepted — designated safe 4건만 추적하고, fault 중에는 SETTLED로 두지 않는다.
Rejected — A-zero만으로 target/actual separation을 주장하는 것, 거절된 명령의 execution trace.

Reason
Designated safe 4/4는 SAFE_TO_ATTEMPT 뒤 추적, COMPLETED, SETTLED다.
Global max joint tracking error는 0.7 rad다. 이것은 이동 시작의 target-actual 차이며 positioning accuracy가 아니다.
Max TCP position tracking error는 667.340659995662 mm다.
Max TCP orientation tracking error는 108.88601121861682 deg다.
Final max joint error는 0.0 rad다.
B-positive-1, C-mixed-1, E-fixed-3은 초기와 중간 sample에서 target_q와 actual_q가 달랐다.
E-fixed-1은 REJECT이고 submit과 tracking은 시작되지 않았다.
Fault는 tracking이 시작된 뒤 SETTLED가 아니고 COMPLETED도 아니다.
Timeout 0, non-finite 0. Replay는 exact match다.

Verification
evidence/P-009. evidence/P-001부터 P-008-FIX2는 수정하지 않았다.
PYTHONPATH=. python3 -m pytest tests/unit/test_ur10e_fk.py tests/unit/test_p001b_evidence.py tests/unit/test_p006_controller.py tests/unit/test_p007_convert.py tests/unit/test_p008_validator.py tests/unit/test_p008_fix2_acceptance.py tests/unit/test_p009_monitor.py -q → 52 passed.
ROS, Isaac, Unity는 다시 실행하지 않았다.
이 결과는 실제 UR10e 정확도나 hard real-time이 아니다.

Elapsed Time
2026-10-23 측정은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```

## P-010 — Operator HMI baseline

```text
Task
P-008 검증과 P-009 tracking 결과를 한 화면에서 보여주고, SAFE_TO_ATTEMPT가 아닌 명령은 controller에 전달하지 않는다.
P-011은 시작하지 않는다.

Problem
REJECT가 버튼 표시만으로 막히면 실행 경로가 남을 수 있다.
Fault를 COMPLETED나 SETTLED로 보이면 수렴으로 오해한다.

AI Tool
Cursor agent (Grok 4.7), 2026-10-23.

AI Proposal
1. HMI는 validator.validate와 digital_twin.session.run_case의 결과를 표시한다. Tracking 식과 FK를 다시 만들지 않는다.
2. Execute는 validation_decision이 SAFE_TO_ATTEMPT이고 execute_enabled일 때만 runner를 호출한다.
3. 대표 실행은 B-positive-1이다. E-fixed-1은 REJECT다. Fault는 B-positive-1에 TEST_FAULT를 주입한 live project run이다.
4. 거절: replay를 live execution으로 표시, SAFE_TO_ATTEMPT를 안전 인증으로 표시, NaN을 0으로 표시, P-011.

Developer Decision
위 1–4를 수용한다. controller, validator, digital_twin은 수정하지 않는다.

Accepted / Rejected
Accepted — 한 화면에서 검증, 차단 이유, target/actual, tracking, settling, fault를 구분한다.
Rejected — REJECT 뒤 controller 제출, fault를 수렴으로 표시.

Reason
B-positive-1은 SAFE_TO_ATTEMPT 뒤 controller에 전달되고 COMPLETED, SETTLED다.
Peak joint error는 0.7 rad다. completion_time_s는 0.23000000000000007이고 settled_at_s는 0.25000000000000006이다.
E-fixed-1은 REJECT, COLLISION, wrist_3_link와 bolt_panel, sample 44/100이다. runner는 호출되지 않았다.
Fault는 ERROR, FAULTED, settled false, completed false다.
Regression은 33 passed in 21.95s다.
Screenshot는 3개다.

Verification
evidence/P-010.
PYTHONPATH=. python3 -m pytest tests/unit/test_p006_controller.py tests/unit/test_p008_validator.py tests/unit/test_p009_monitor.py tests/unit/test_p010_hmi.py -q → 33 passed.
이 화면은 산업 안전 인증이나 실제 UR10e 정밀도가 아니다.

Elapsed Time
2026-10-23 측정은 한 세션이다. 구현 사이클 시간은 따로 측정하지 않았다.
```
