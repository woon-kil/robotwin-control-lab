# Evidence Rules

RoboTwin Control Lab — P-000B Evidence Contract

| 항목 | 값 |
| --- | --- |
| 문서 | Evidence Rules |
| 기준선 | `62b9a624083cb6ac584e4eb6fd6a69c8c56197ee` |
| 현재 Gate | P-000B |
| 구현 | NOT STARTED |
| 작성일 | 2026-10-23 |

이후 게이트는 이 규칙을 따른다. P-000B 자체는 구현, 테스트, 측정을 하지 않으므로 이 게이트의 PASS Evidence 파일을 만들지 않는다.

## 1. Required flow

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
   ↓
NEXT GATE
```

`Robot moved successfully`처럼 동작했다는 서술만으로 PASS할 수 없다. PASS는 요구사항, 테스트, 측정값, 기준, Evidence가 연결되고 검토를 통과했을 때 쓴다.

다음 게이트는 앞 게이트의 판정이 기록된 뒤에 시작한다. 기록이 없다는 이유로 판정을 PASS로 바꾸지 않는다.

## 2. Minimum metadata

이후 Evidence에는 가능한 항목을 남긴다.

```text
Gate ID
Test ID

Timestamp

Git Commit

Environment
Software Version

Robot Model Version

Input
Expected Result
Actual Result

Metric
Threshold

PASS / FAIL

Raw Artifact Reference
```

없는 값은 비워 둔다. 추정해서 채우지 않는다. 비어 있는 Threshold는 합격 한도가 아니다. 비어 있는 Robot Model Version으로 모델 검증을 PASS로 닫지 않는다.

Expected Result가 없는 실행은 측정 기록일 수 있다. 그것을 PASS로 승격하지 않는다. Actual Result만 있고 기준이 없으면 NFR-004의 미확정이다.

Raw Artifact Reference는 원자료의 위치다. 원자료가 없으면 참조를 만든 것처럼 적지 않는다.

## 3. Immutability

PASS에 사용한 Evidence를 나중에 조용히 덮어쓰지 않는다.

재실행 결과가 다르면 새 Evidence로 남긴다. 이전 PASS 파일을 새 숫자로 교체하지 않는다.

실패 기록도 삭제하지 않는다. 실패, 수정, 다시 실행, PASS가 서로 다른 기록으로 남아 추적되어야 한다. 실패 파일을 고쳐서 처음부터 PASS였던 것처럼 만들지 않는다.

같은 입력이라도 Git Commit, 소프트웨어 버전, 로봇 모델 버전이 다르면 다른 실행이다.

## 4. What cannot be evidence

다음은 이 저장소의 PASS Evidence가 아니다.

- 시뮬레이터 화면에서 로봇이 움직였다는 설명
- 명령을 보냈다는 로그만 있는 기록
- 측정하지 않은 지연을 0으로 둔 기록
- 출처 없는 질량, 관성, 페이로드로 만든 물리 합격
- 다른 프로젝트의 myCobot 계열 측정값
- 20 ms처럼 이 기준선에 없는 지연 한도
- `TARGET — NOT YET VALIDATED`라고 표시된 Engineering Target 자체

Engineering Target은 `docs/VALIDATION_PLAN.md`에 있다. 그 표는 앞으로 비교할 후보 기준이며, 이미 검증되었다는 증거가 아니다.

## 5. Failure visibility

오류, 타임아웃, 임포터 불일치, 기구학 실패, 기대하지 않은 충돌, Feedback 누락, NOT_EVALUATED는 기록에 남긴다. 예외를 삼키고 다음 주기를 성공으로 잇지 않는다. 테스트를 통과시키려고 기대값을 결과 쪽으로 옮기지 않는다.

Required validator가 FAIL이거나 NOT_EVALUATED이면 전체 PASS를 쓰지 않는다. required 목록이 없으면 전체 PASS를 쓰지 않는다. 이 집계 규칙은 `docs/ROBOT_INTERFACE.md` 8절과 같다.
