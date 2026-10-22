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
