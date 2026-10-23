# P-001B Independent FK

```text
Gate
P-001B

upstream SHA
7ce98d8c3512b9d0239da93b75de68ea64f61bd6

Result
PASS
```

이 PASS는 Canonical model이 재현되고, 독립 FK가 결정적으로 실행되며, 구조 검사가 통과했다는 뜻이다. 외부 reference와 맞다는 정확도 합격은 아니다. Unity, Isaac, 실제 로봇과 비교하지 않았다.

## Canonical model

```text
source repository
UniversalRobots/Universal_Robots_ROS2_Description

source commit
7ce98d8c3512b9d0239da93b75de68ea64f61bd6

robot type
ur10e

source manifest
robot_description/SOURCE_MANIFEST.md

model identifier
ur10e-nominal-7ce98d8c3512b9d0239da93b75de68ea64f61bd6

base_frame
base_link

tcp_frame
tool0
```

`base_frame`은 source의 `base_link`다. `base`는 같은 프레임이 아니다. `tcp_frame`은 source의 `tool0`이다. `T_tool0_tcp`와 `bolt_tool_tcp`는 숫자가 없어 곱하지 않았다.

관절 원점의 rpy는 URDF 고정축 회전 Rz(yaw) Ry(pitch) Rx(roll)이다. `T_parent_child(q) = T_origin * T_joint(q)`이고, `T_base_link_tool0`은 그 행렬을 base에서 tool0까지 곱한 값이다.

## Joint order

```text
shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
```

이 순서는 xacro의 actuated 직렬 체인이다. dictionary 순서나 시뮬레이터 import 순서를 쓰지 않는다.

## Tests

```text
test count
11

test method
고정 joint vector. 난수 없음.
각 vector를 forward_kinematics에 두 번 넣어 같은 Pose인지 확인한다.
구조 검사는 tests/unit이다.

test command
PYTHONPATH=. python3 -m pytest tests/unit -q
```

위치 단위는 meter, 관절각은 radian, 자세 저장은 quaternion (x, y, z, w)이다. Euler는 저장하지 않는다.

Joint limit 밖의 입력도 FK가 clamp하지 않는다. 한계는 모델에 radian으로 보존되고 `limit_violations`가 따로 검사한다. source의 `!degrees`는 로드할 때 `degrees * pi / 180`으로 바뀐다.

## Known limitations

- official nominal UR10e model 기반이다.
- robot-specific calibration이 아니다.
- 실제 UR10e 측정값과 비교하지 않았다.
- Unity와 비교하지 않았다.
- Isaac Sim과 비교하지 않았다.
- physical robot accuracy를 주장하지 않는다.
- `FK Position Error < 0.5 mm`는 이번 결과가 아니다.
