# RoboTwin Control Lab

Status:
P-001A.1 — Controlled Requirements Amendment

Current Gate:
P-001A.1 Controlled Requirements Amendment

Implementation:
NOT STARTED

RoboTwin Control Lab is an industrial robot Digital Twin and Pre-Execution Validation platform.
대표 Use Case는 Generic Industrial Bolt Fastening / Loosening with Torque-Aware Validation이다.
질문은 둘로 나눈다. 지정된 볼트 작업을 안전하게 시도할 수 있는가. 실행 후 체결 또는 해체가 정의된 Torque/Angle 조건으로 완료되었는가.
Pre-Execution 성공은 SAFE_TO_ATTEMPT이며 체결 성공이 아니다.
REQ-001–REQ-022와 NFR-001–NFR-007은 유지한다. 이번 개정은 REQ-023–REQ-028을 추가한다.
Golden Reference는 핀된 official nominal UR10e다. 기록은 `robot_description/SOURCE_MANIFEST.md`다.
Hazardous Filter Handling은 도메인 동기다. 대표 구현 시나리오가 아니다.
구현은 시작하지 않았다. P-001B는 시작하지 않았다. 진단과 정비 판단은 Maintenance-Agentic-Twin의 책임이다.
