# P-001A Source Selection

```text
Gate
P-001A

Timestamp
2026-10-23T00:04:38Z
```

`jazzy` 브랜치 팁을 이 시각에 조회했다. 아래 확인은 그 커밋의 파일을 읽은 결과다.

```text
Selected Repository
UniversalRobots/Universal_Robots_ROS2_Description

Selected Branch
jazzy

Pinned Commit
7ce98d8c3512b9d0239da93b75de68ea64f61bd6

Robot
ur10e

Package
ur_description
```

```text
Why selected
공식 UniversalRobots 조직의 ROS2 description 저장소다.
ROS distribution baseline이 Jazzy이므로 검토 브랜치는 jazzy다.
브랜치는 움직이므로 조회 시점의 커밋 SHA를 핀한다.
비공식 URDF 저장소는 사용하지 않는다.
선택한 설명은 official nominal UR10e description이다.
특정 실제 장비의 calibration model로 쓰지 않는다.
```

```text
Entry Point verified
YES
urdf/ur.urdf.xacro
macro ur_robot in urdf/ur_macro.xacro

UR10e config verified
YES
config/ur10e/default_kinematics.yaml
config/ur10e/joint_limits.yaml
config/ur10e/physical_parameters.yaml
config/ur10e/visual_parameters.yaml

Joint inventory verified
YES
actuated joint 6개. 이름, 순서, type, parent, child, limit은 핀된 xacro와 joint_limits.yaml에서 추출.

Frame inventory verified
YES
base, base_link, shoulder_link, upper_arm_link, forearm_link,
wrist_1_link, wrist_2_link, wrist_3_link, flange, tool0.
base와 base_link는 별도 link.

License reviewed
YES
package.xml license 태그는 BSD-3-Clause, file LICENSE.
Graphical Documentation 주석은 UR10e를 그 목록에 넣지 않는다.
meshes/ur10e에는 LICENSE.txt가 없다.
제약 없음으로 적지 않았다.
```

로컬 복사와 SHA-256은 `robot_description/SOURCE_MANIFEST.md`와 `robot_description/upstream/7ce98d8c3512b9d0239da93b75de68ea64f61bd6/SHA256SUMS.txt`에 있다. 파일 23개의 `shasum -a 256 -c`는 통과했다.

```text
Result:
PASS
```

Independent FK, Unity import, Isaac import, P-001B는 시작하지 않았다.
