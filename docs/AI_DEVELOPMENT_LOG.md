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
