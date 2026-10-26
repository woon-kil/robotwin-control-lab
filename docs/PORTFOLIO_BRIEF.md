# RoboTwin Control Lab

면접 직전 2~3분 복습용이다. 숫자의 원값은 README와 `evidence/`에 있다. 이 문서는 요구사항과 architecture 계약을 개정하지 않는다.

## 20-second introduction

산업용 로봇 명령을 바로 Controller로 보내지 않고 Digital Twin에서 충돌과 실행 가능성을 먼저 검증한 뒤, 승인된 명령만 실행시키는 제어 플랫폼을 만들었습니다. 실행 중에는 Target과 Actual을 추적하고, Unity, Isaac, ROS 2, Virtual Controller를 독립적인 evidence로 검증했습니다. Controller abstraction은 Official Power PMAC Simulator의 software virtual Motor[1]까지 확장했고, 그 결과는 robot joint 검증과 별도입니다.

## Problem

```text
Robot command
    ↓
Can this command be attempted?
    ↓
Could it collide?
    ↓
Should the controller receive it?
    ↓
Did actual state follow target?
    ↓
Did the system converge?
```

`SAFE_TO_ATTEMPT`는 실행 전 PROJECT TEST 통과다. 체결 성공과 certified safety가 아니다.

## Architecture

`RobotCommand` → Pre-Execution Validator → `REJECT` 또는 `SAFE_TO_ATTEMPT` → Controller Abstraction → Target / Actual → Digital Twin Monitor → Operator HMI.

Employment MVP motion backend는 `VirtualPmacController`다. ROS 2 Adapter는 그 controller로의 transport다. `PowerPmacControllerAdapter`는 Official Power PMAC Simulator backend이며 ExecutionGate가 호출하지 않는다. Unity와 Isaac Sim은 같은 nominal UR10e의 독립 검증이며 controller backend가 아니다. 정본은 `docs/ARCHITECTURE.md`, 구현 범위는 `docs/PORTFOLIO_ARCHITECTURE.md`다.

Golden Reference는 official nominal UR10e, commit `7ce98d8c3512b9d0239da93b75de68ea64f61bd6`이다.

## Three strongest results

1. Unity ↔ Isaac TCP position difference `0.29056228373221366 mm`. README 반올림은 0.291 mm. Real robot accuracy가 아니다. 출처는 `evidence/P-004/fix-attempt2/summary.md`.
2. Collision command `E-fixed-1`은 100/100 `REJECT`. Unsafe submission은 0이다.
3. 500-cycle deterministic project workload는 500/500 correct다. Fault recovery는 primary 500과 별도로 10/10이다.

함께 말할 수 있는 transport 수치는 ROS 2 P95 `1.0901190034928732 ms`다. Project test transport latency이며 hard real-time latency가 아니다.

Power PMAC Simulator extension의 별도 수치는 Motor[1] 100/100 motion, reconnect 5/5, false submission 0, tracking error `0 PMAC motor unit`이다. P-023 regression baseline은 `197 passed in 54.94s`다. 상세는 `docs/power-pmac-validation.md`다.

## Strongest failure story

P-008 `E-fixed-1`.

```text
Expectation
canonical 5개가 모두 SAFE_TO_ATTEMPT

Unexpected Result
E-fixed-1 REJECT, wrist_3_link ↔ bolt_panel, sample 44/100

Independent Verification
geometry, FK, transform, AABB, independent collision

Root Cause
TRUE_COLLISION
Validator defect = NO
Acceptance assumption defect = YES

Decision
collision rule을 느슨하게 만들지 않음
틀린 acceptance만 수정

Revalidation
P-010 Execute blocked
P-011 100/100 REJECT
unsafe submission = 0
```

## Why this project matters

안전 검증 기준을 낮춰 테스트를 통과시키지 않았다. 예상과 다른 REJECT를 독립적으로 확인한 뒤, validator가 아니라 가정이 틀렸다고 기록했다. 그 판정은 HMI 차단과 500-cycle 반복까지 이어진다.

## Limitations

```text
No real robot validation
No real Power PMAC hardware
Official Power PMAC Simulator only
Motor[1] is not a robot joint
No industrial safety certification
UR10e model is nominal reference
Tracking metrics are project virtual-controller results
500-cycle result is deterministic project workload,
not field reliability or MTBF evidence
```

NOT VERIFIED AGAINST REAL POWER PMAC.

## Likely interview questions

**REJECT를 없애기 위해 tolerance를 바꿨는가.** 바꾸지 않았다. FIX1의 validator defect는 NO다. FIX2는 acceptance만 고쳤다.

**0.291 mm는 실제 UR10e 정확도인가.** 아니다. Unity와 Isaac의 nominal model TCP position difference다.

**500/500은 현장 신뢰성인가.** 아니다. 500-cycle deterministic project workload다. Timeout과 non-finite는 0이다.

**SAFE_TO_ATTEMPT는 안전 인증인가.** 아니다. 실행 전 검사 통과이며, 체결 성공이 아니다.

**Power PMAC hardware를 검증했는가.** 아니다. Official Power PMAC Simulator의 software virtual Motor[1]을 검증했다. Real hardware, encoder, EtherCAT, industrial reliability는 검증하지 않았다. ACK는 completion이 아니다. 설명은 `docs/interview-notes.md`다.

**Target과 Actual은 왜 다른가.** 승인된 이동 중 controller target과 actual joint를 따로 기록한다. 거절된 명령은 tracking을 시작하지 않는다. Fault는 `COMPLETED`와 `SETTLED`로 표시하지 않는다.
