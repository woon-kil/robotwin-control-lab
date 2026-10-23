# Validation Plan

RoboTwin Control Lab — P-000B Measurement Plan

| 항목 | 값 |
| --- | --- |
| 문서 | Validation Plan |
| 기준선 | `62b9a624083cb6ac584e4eb6fd6a69c8c56197ee` |
| 현재 Gate | P-000B |
| 구현 | NOT STARTED |
| 작성일 | 2026-10-23 |

이 문서는 P-001 이후에 만들 Evidence가 어떤 구조로 검증되는지 정한다. 테스트를 실행하지 않고, 벤치마크 숫자를 만들지 않는다. 아래 목표 숫자는 PASS Evidence가 아니다.

## 1. Measurement candidates

이후 게이트에서 측정할 후보다. 후보가 있다는 사실이 threshold를 만들지는 않는다.

```text
FK Position Error
FK Orientation Error

Joint Target Error

Unity ↔ Independent FK TCP Error
Isaac ↔ Independent FK TCP Error
Unity ↔ Isaac TCP Error

Trajectory Completion

Joint Limit Violation
Collision
Self Collision
Safety Zone Violation

Command Latency
Simulator Latency
Feedback Latency
RTT

FPS
Memory Growth

Cycle Success Rate
```

비교에는 명령 식별자, 모델 식별자, 프레임 이름, 단위가 함께 있어야 한다. 셋이 필요한 비교에서 하나가 빠지면 그 실행은 비교 PASS가 될 수 없다. 이 문장은 REQ-008을 반복한 것이며 요구사항 문장을 바꾸지 않는다.

FPS와 Memory Growth에는 합격 한도를 적지 않는다. Cycle Success Rate의 목표 횟수는 기존 REQ-018의 500 cycles를 다른 이름으로 바꾸지 않는다. 500 cycles는 `docs/REQUIREMENTS.md`의 초기 Engineering Target이며 이 계획에서 새로 보증하지 않는다.

## 2. Engineering targets

다음 값은 Engineering Target이다. PASS Evidence가 아니다. 각 값의 상태는 같다.

```text
TARGET — NOT YET VALIDATED
```

| 항목 | Engineering Target | 상태 |
| --- | --- | --- |
| FK Position Error | < 0.5 mm | TARGET — NOT YET VALIDATED |
| FK Orientation Error | < 0.1 deg | TARGET — NOT YET VALIDATED |
| Joint Target Error | < 0.1 deg | TARGET — NOT YET VALIDATED |
| Unity ↔ Isaac TCP Error | < 1.0 mm | TARGET — NOT YET VALIDATED |
| Trajectory Completion | = 100% | TARGET — NOT YET VALIDATED |
| Joint Limit Violation | = 0 | TARGET — NOT YET VALIDATED |
| Unexpected Collision | = 0 | TARGET — NOT YET VALIDATED |

FK 위치와 자세, 관절 목표, Unity와 Isaac의 TCP, 기대하지 않은 충돌의 숫자는 `docs/REQUIREMENTS.md` 4절에 이미 초기 목표로 있다. 그 절은 보증값이 아니라고 적혀 있다. 이 표는 그 숫자를 다시 측정된 합격으로 올리지 않는다.

Trajectory Completion과 Joint Limit Violation은 이 계획에 적은 목표다. REQUIREMENTS.md의 목표 표에는 같은 행이 없다. 요구사항 파일은 수정하지 않았다. 두 항목을 요구사항 숫자에 넣을지는 Open Issue다.

단위는 비교 전에 Canonical 단위로 맞춘다. 저장 각은 radian이고, 위 표의 degree는 목표를 읽는 단위다. Evidence에 적을 때는 실제 기록 단위를 함께 쓴다.

## 3. Latency

Latency의 PASS threshold는 정하지 않는다. 먼저 다음을 측정한다.

```text
Mean
P50
P95
P99
Max
```

대상 후보는 Command Latency, Simulator Latency, Feedback Latency, RTT다. 표본 수와 시계를 함께 남긴다. 통계를 계산할 수 없으면 미측정이다.

기존의 20 ms 숫자를 PASS threshold로 사용하지 않는다. 이 저장소의 기준선에는 그 한도가 없으며, 다른 경험의 숫자를 여기로 가져오지 않는다.

## 4. Prior myCobot experience

다른 프로젝트에서 다룬 다음 종류의 수치는 참고 경험이다.

- Isaac joint command error
- Python FK ↔ Isaac TCP
- Unity TCP error
- Python ↔ C# FK consistency

그 숫자는 이 문서에 옮기지 않는다. RoboTwin의 PASS Evidence로 재사용하지 않는다. UR10e 또는 이후 모델을 쓰는 새 환경과 새 기준선에서 다시 측정한다.

## 5. What P-001 is expected to measure

P-001의 목적은 `docs/ARCHITECTURE.md` 5절과 같다. 핀된 공개 로봇 모델에 대해 독립 FK의 정량 기준선을 만드는 것이다.

P-000B는 그 측정을 실행하지 않는다. P-000B 시점에는 UR10e의 source, repository, commit, model version을 고르지 않았다. P-001A가 이후 official nominal UR10e를 핀했다. 기록은 `robot_description/SOURCE_MANIFEST.md`다. 그 핀 이후에도 이 계획의 FK 측정은 실행되지 않았다. 그 전에 만든 FK 숫자는 이 계획의 PASS가 될 수 없다.

## 6. Physics and safety claims

중력, 질량, 관성, 페이로드, 마찰, 충돌은 REQ-009의 측정 후보다. 출처가 없는 물리 파라미터는 PASS 근거가 되지 못한다.

Safety Zone 위반은 측정 후보이며, NORMAL, WARNING, PROHIBITED는 시뮬레이션 정책 모델이다. 산업 안전 규격의 인증 결과가 아니다.

## 7. Bolt use case boundary

`BOLT_FASTEN`과 `BOLT_LOOSEN`의 Pre-Execution 결과는 `SAFE_TO_ATTEMPT` 또는 `REJECTED`다. REQUIRED 목록은 REQ-026이다. 그 결과는 `FASTENED`, `LOOSENED`, `FAILED`, `FAULT`가 아니다.

Execution 판정은 Rotation Angle, Rotation Speed, Torque Feedback, Contact State, Tool State, Controller State를 사용할 수 있다. Torque 숫자는 `SIMULATION TEST PARAMETER` 또는 `PROJECT TEST PARAMETER`다. 실제 장비 specification이 아니다. 이 계획은 Torque 숫자를 만들지 않는다.

Virtual Fastening Process와 Virtual Torque Sensor의 기록은 simulated/virtual로 식별한다. 실제 센서 측정값으로 적지 않는다.

대표 정상 흐름과 대표 Failure Demo는 REQ-026과 REQ-027의 문서 요구다. 이 계획은 그 흐름을 실행하지 않고 PASS Evidence를 만들지 않는다.

## 검증 근거

이 계획의 숫자를 기존 문서와 대조한 기록이다. 새 측정은 하지 않았다.

- `docs/REQUIREMENTS.md:746-771` — 4절 초기 Engineering Target. FK Position Error `< 0.5 mm`, FK Orientation Error `< 0.1 deg`, Joint Target Error `< 0.1 deg`, Cross-Simulator TCP Position Error `< 1.0 mm`, Unexpected Collision `0`, Reliability Target `500 cycles`. 상태 열은 `보증값 아님`이다. Trajectory Completion과 Joint Limit Violation 행은 이 표에 없다.
- `docs/REQUIREMENTS.md:181-195` — REQ-017. Latency PASS 임계값은 없고 Mean, P50, P95, P99, Max만 있다. 20 ms 한도는 이 파일에 없다.
- `docs/REQUIREMENTS.md:197-207` — REQ-018. 반복 목표 횟수는 `500 cycles`다.
- `docs/PROJECT_CHARTER.md:67-73` — 차터 그림은 Unity, Isaac, ROS2, PMAC를 대등한 어댑터로 둔다. 이 계획 문서는 그 그림을 바꾸지 않는다.
- ROS REP-103, https://reps.openrobotics.org/rep-0103/ , 조회 2026-10-23 — SI 단위, 오른손 좌표계, 각의 기본 단위는 radian. 이 계획의 degree 표기는 목표를 읽는 단위이고 저장 단위를 바꾸지 않는다.
