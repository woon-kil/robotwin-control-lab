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
