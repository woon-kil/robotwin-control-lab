# Coordinate Convention

RoboTwin Control Lab — P-000B Coordinate Contract

| 항목 | 값 |
| --- | --- |
| 문서 | Coordinate Convention |
| 기준선 | `62b9a624083cb6ac584e4eb6fd6a69c8c56197ee` |
| 현재 Gate | P-000B |
| 구현 | NOT STARTED |
| 작성일 | 2026-10-23 |

좌표계는 Cross-Simulator Validation의 핵심 실패 원인이다. 이 계약은 P-001 전에 잠근다. 로봇 모델의 수치와 시뮬레이터 변환 행렬은 잠그지 않는다.

## 1. Source used for the rule set

ROS REP-103을 참고한다. 조회일은 2026-10-23이다.

- 문서: Standard Units of Measure and Coordinate Conventions
- URL: https://reps.openrobotics.org/rep-0103/

그 문서의 해당 규칙은 다음과 같다.

- 단위는 SI다. 길이는 meter, 각은 radian, 시간은 second다.
- 좌표계는 오른손이다.
- 몸체 프레임의 축은 x forward, y left, z up이다.
- 카메라 optical 접미 프레임의 축은 z forward, x right, y down이다.
- 회전 표현의 우선순위는 quaternion이다. Euler 각은 규약이 많아 권장되지 않는다.

아래 Canonical convention은 그 규칙을 이 프로젝트의 계약으로 다시 적은 것이다. REP-103이 정하지 않는 Unity 변환과 Isaac 변환은 추정하지 않는다.

## 2. Canonical convention

Canonical Robot Core의 기본 단위와 손잡이는 다음이다.

- SI units
- distance: meter
- angle: radian
- time: second
- right-handed coordinate system

몸체 프레임은 REP-103의 몸체 규칙을 따른다.

- +x forward
- +y left
- +z up

카메라 optical 프레임을 둘 때는 REP-103의 optical 규칙을 따른다.

- +z forward
- +x right
- +y down

이 축 의미는 프레임의 종류에 대한 계약이다. 특정 로봇의 베이스가 셀의 어느 벽을 향하는지는 숫자가 아니며, 이 게이트에서 정하지 않는다. 관절 축은 나중에 핀되는 모델의 출처를 따른다. UR10e의 축을 여기서 추정하지 않는다.

## 3. Frame hierarchy

최소 논리 프레임은 다음이다.

```text
world
  │
  └── robot_base
         │
         └── joint/link chain
                    │
                    └── tool0
                           │
                           └── tcp
```

의미는 이름과 부모 관계까지다.

- `world`: 셀 기준의 오른손 프레임. +z는 위다. 수평 +x가 셀의 어느 방향인지는 수치로 정하지 않는다.
- `robot_base`: 로봇 몸체 프레임. 축 의미는 2절의 몸체 규칙이다. `T_world_robot_base`의 숫자는 없다.
- `joint/link chain`: Canonical 모델이 나중에 제공할 관절과 링크의 사슬이다. 링크 길이와 관절 원점은 이 문서에 없다.
- `tool0`: 도구 부착 프레임의 이름이다.
- `tcp`: MOVE_TCP와 TCP 검사에 쓰는 제어점 프레임의 이름이다. `T_tool0_tcp`의 숫자는 없다.
- `bolt_tool_tcp`: 대표 Bolt Tool이 붙었을 때의 제어점 이름이다. 개념 사슬은 `flange` → bolt tool → `bolt_tool_tcp`다. `flange`는 로봇 플랜지이고 `tool0`은 도구 부착 프레임이다. `bolt_tool_tcp`를 `flange` 또는 `tool0`과 같은 프레임으로 두지 않는다. `T_flange_bolt_tool_tcp`와 공구 길이, 자세, 충돌 geometry의 숫자는 없다.

Vision 단계에서 다음 이름이 추가될 수 있다.

```text
world
robot_base
camera
camera_optical
tool0
tcp
object
target
```

`camera_optical`의 축 의미는 2절의 optical 규칙이다. `camera`에서 `camera_optical`로 가는 숫자 변환은 없다. `object`와 `target`도 이름만 있다.

실제 수치 transform은 정의하지 않는다.

## 4. Transform notation

문서 전체의 표기는 `T_A_B` 하나다.

`T_A_B`는 프레임 B에 표현된 점을 프레임 A의 좌표로 보내는 강체 변환이다.

- source frame: B
- destination frame: A
- 점은 동차 열벡터 `(x, y, z, 1)`이다.
- 곱셈 순서: 열벡터의 왼쪽에 행렬을 곱한다. 오른쪽 행렬이 먼저 적용된다.

```text
p_A = T_A_B * p_B

T_A_C = T_A_B * T_B_C
```

따라서 이름이 있는 예는 다음을 뜻한다.

- `T_world_robot_base`: source `robot_base`, destination `world`
- `T_robot_base_tcp`: source `tcp`, destination `robot_base`
- `T_world_camera`: source `camera`, destination `world`

사슬은 다음과 같이 쓴다.

```text
T_world_tcp = T_world_robot_base * T_robot_base_tool0 * T_tool0_tcp
```

자유 벡터도 같은 회전을 쓴다. `T_A_B`의 왼쪽 위 3×3을 `R_A_B`라고 하면 `v_A = R_A_B * v_B`다.

이 표기는 부모를 A, 자식을 B로 둘 때 자식 좌표를 부모 좌표로 보내는 읽기와 맞춘다. 반대 방향의 행렬을 같은 이름으로 쓰지 않는다. 역변환이 필요하면 `T_B_A`라고 적고, `T_A_B`의 역이라고 따로 말한다.

예시에 숫자를 넣지 않는다. 세 예의 행렬 값은 아직 없다.

## 5. Orientation

저장과 전송의 기본 표현은 Quaternion이다.

- 규약: Hamilton
- 성분 순서: `(x, y, z, w)`
- 항등 회전: `(0, 0, 0, 1)`
- 오른손 좌표계의 회전이며, 노름은 1이다
- 이 quaternion에서 뽑은 행렬이 `R_A_B`다

성분 순서는 ROS `geometry_msgs/Quaternion`의 필드 순서와 같게 잠근다. 메시지 파일을 이 게이트에서 만들지는 않는다. `(w, x, y, z)` 순서로 저장하지 않는다.

`q`와 `-q`는 같은 회전이다. 어느 부호를 대표값으로 둘지는 정하지 않았다. 비교 코드가 생기기 전에 부호 규칙을 추정으로 넣지 않는다.

Euler angle은 HMI의 사람이 읽는 표시에만 쓸 수 있다. Canonical 상태, 명령, Evidence의 저장 값이 될 수 없다.

Euler를 표시하면 그 표시마다 다음을 적는다.

- axis order
- intrinsic 또는 extrinsic
- degree 또는 radian

표시의 기본 축 순서는 잠그지 않는다. REP-103이 Euler 규약이 여러 개라서 권장하지 않기 때문이다. 표시 단위가 degree여도 Canonical 저장 단위는 radian이다.

## 6. Simulator coordinate adapters

Unity와 Isaac의 좌표 차이는 adapter 책임이다.

```text
Canonical
   │
   ├─ UnityCoordinateAdapter
   │
   └─ IsaacCoordinateAdapter
```

Core의 Kinematics Engine은 시뮬레이터 좌표 규칙을 직접 처리하지 않는다. 변환은 adapter 안에서 Canonical convention으로 들어오고 Canonical convention으로 나간다.

실제 변환 행렬은 작성하지 않는다. Unity의 손잡이, Isaac 또는 USD 스테이지의 업축, 단위 배율은 해당 시뮬레이터 검증 게이트에서 실측한 뒤에만 숫자가 된다. 이 문서에 그 숫자를 추정해 넣지 않는다.

어댑터가 변환을 적용하면 Evidence에 프레임 이름, 단위, 사용한 규약의 식별자를 남긴다. 규약 식별자가 없는 TCP 비교는 비교 증거가 아니다.
