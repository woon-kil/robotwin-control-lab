# UR10e Source Manifest

Golden Reference Robot:

Official nominal UR10e description. `UniversalRobots/Universal_Robots_ROS2_Description` 패키지 `ur_description` 3.6.0이 싣고 있는 `ur_type=ur10e` 기본 설명이다.

특정 실제 UR10e에서 추출한 calibration model이 아니다. 컨트롤러에서 뽑은 robot-specific calibration 파일도 아니다. 실제 장비별 calibration이 필요하면 이후 별도 integration에서 다룬다.

이 모델은 RoboTwin의 최종 portfolio robot이 아니다. 역할은 다음뿐이다.

```text
UR10e official nominal model
        ↓
Canonical Robot Model
        ↓
Independent FK
        ↓
Unity / Isaac comparison
```

Generic Long-Reach Manipulator는 이 게이트의 대상이 아니다.

## Source

```text
Source Repository
UniversalRobots/Universal_Robots_ROS2_Description

Source Owner
UniversalRobots

Branch
jazzy

Pinned Commit
7ce98d8c3512b9d0239da93b75de68ea64f61bd6

Commit subject
3.6.0

Author date
2026-10-22T12:34:10+02:00

Commit date
2026-10-22T17:55:46+02:00

ROS Distribution
Jazzy

Package
ur_description

Package version
3.6.0

Robot Type
ur10e

Retrieval Date
2026-10-23T00:04:38Z
```

조회 시점의 `refs/heads/jazzy`는 `7ce98d8c3512b9d0239da93b75de68ea64f61bd6`였다. 브랜치 이름만으로 핀하지 않는다. 커밋 URL:

https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/commit/7ce98d8c3512b9d0239da93b75de68ea64f61bd6

저장소 루트가 패키지다. `ur_description/` 하위 디렉터리는 이 커밋에 없다. 아래 경로는 저장소 루트 기준이다.

## Entry point

```text
Entry Point
urdf/ur.urdf.xacro

Macro
ur_robot in urdf/ur_macro.xacro
```

`ur.urdf.xacro`의 `ur_type` 기본값은 `ur5x`다. 이 값은 타입을 빠뜨리면 실패하도록 둔 sentinel이다. UR10e 설명은 `ur_type=ur10e`일 때 같은 파일의 기본 인자로 다음 네 파일을 읽는다.

`tf_prefix` 기본값은 빈 문자열이다. 아래 joint 이름은 그 기본값 기준이다.

`urdf/ur_mocked.urdf.xacro`와 `ros2_control` mock hardware는 이 Golden Reference의 entry point가 아니다.

## Parameters

```text
Kinematics Parameters
config/ur10e/default_kinematics.yaml

Joint Limits
config/ur10e/joint_limits.yaml

Physical Parameters
config/ur10e/physical_parameters.yaml

Visual Parameters
config/ur10e/visual_parameters.yaml
```

`urdf/inc/ur_common.xacro`의 `read_model_data`가 이 네 파일을 `xacro.load_yaml`로 읽고, joint limit 속성을 다음처럼 연결한다.

```text
min_position  → lower
max_position  → upper
max_velocity  → velocity
```

위치와 속도는 YAML에서 `!degrees`로 적혀 있다. 이 문서는 그 표기를 라디안으로 바꿔 적지 않는다. `wrist_3_joint`는 `has_position_limits: true`이므로 `ur_common.xacro`가 joint type을 `revolute`로 둔다.

### Kinematics source values

단위는 파일에 적힌 값 그대로다. 위치는 미터, 각도는 라디안 리터럴이다. 이 표는 FK 계산 결과가 아니다.

| segment | x | y | z | roll | pitch | yaw |
| --- | --- | --- | --- | --- | --- | --- |
| shoulder | 0 | 0 | 0.1807 | 0 | 0 | 0 |
| upper_arm | 0 | 0 | 0 | 1.570796327 | 0 | 0 |
| forearm | -0.6127 | 0 | 0 | 0 | 0 | 0 |
| wrist_1 | -0.57155 | 0 | 0.17415 | 0 | 0 | 0 |
| wrist_2 | 0 | -0.11985 | -2.458164590756244e-11 | 1.570796327 | 0 | 0 |
| wrist_3 | 0 | 0.11655 | -2.390480459346185e-11 | 1.570796326589793 | 3.141592653589793 | 3.141592653589793 |

같은 파일의 `hash` 필드는 `calib_5119701370761913513`이다. 이 문자열은 upstream YAML에 있는 필드다. 특정 실제 장비의 calibration 기록으로 해석하지 않는다.

`ur_macro.xacro` 머리 주석은 기본 kinematic 파라미터가 특정 로봇과 대응하지 않으며, 장비별 변환은 컨트롤러에서 추출한 YAML이 따로 필요하다고 적는다. 이번 핀은 그 추출 파일을 사용하지 않는다.

## Include chain

```text
urdf/ur.urdf.xacro
      ↓ include
urdf/ur_macro.xacro
      ↓ include
urdf/inc/ur_common.xacro
      ↓ xacro.load_yaml
config/ur10e/joint_limits.yaml
config/ur10e/default_kinematics.yaml
config/ur10e/physical_parameters.yaml
config/ur10e/visual_parameters.yaml
      ↓ mesh package path
meshes/ur10e/visual/*.dae
meshes/ur10e/collision/*.stl
```

`ur_macro.xacro`가 직접 include하는 xacro는 `urdf/inc/ur_common.xacro` 하나다. 같은 `urdf/inc/`에 `ur_joint_control.xacro`, `ur_sensors.xacro`, `ur_transmissions.xacro`가 upstream에 있으나 `ur.urdf.xacro`의 include 경로에는 없다. 이번 로컬 복사에도 넣지 않았다.

`smart_panel` 기본 인자는 빈 문자열이다. `ur10e/visual_parameters.yaml`에는 `smart_panel` 키가 없다. `ur_common.xacro`는 키가 없으면 Smart Panel을 끄도록 되어 있다. `meshes/common/smart_panel`은 이번 UR10e 기본 설명의 mesh dependency가 아니다.

## Mesh dependency

`visual_parameters.yaml`이 가리키고, 핀된 트리에 파일도 있는 경로다.

| link key | visual | collision |
| --- | --- | --- |
| base | meshes/ur10e/visual/base.dae | meshes/ur10e/collision/base.stl |
| shoulder | meshes/ur10e/visual/shoulder.dae | meshes/ur10e/collision/shoulder.stl |
| upper_arm | meshes/ur10e/visual/upperarm.dae | meshes/ur10e/collision/upperarm.stl |
| forearm | meshes/ur10e/visual/forearm.dae | meshes/ur10e/collision/forearm.stl |
| wrist_1 | meshes/ur10e/visual/wrist1.dae | meshes/ur10e/collision/wrist1.stl |
| wrist_2 | meshes/ur10e/visual/wrist2.dae | meshes/ur10e/collision/wrist2.stl |
| wrist_3 | meshes/ur10e/visual/wrist3.dae | meshes/ur10e/collision/wrist3.stl |

패키지 이름은 모두 `ur_description`이다. Unity import와 Isaac import는 하지 않았다.

## Actuated joints

순서와 parent/child는 `ur_macro.xacro`의 직렬 체인이다. limit 숫자는 `joint_limits.yaml` 원문이다. `tf_prefix`는 기본값인 빈 문자열이다.

| order | joint name | type | parent link | child link | lower | upper | velocity |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | shoulder_pan_joint | revolute | base_link_inertia | shoulder_link | !degrees -360.0 | !degrees 360.0 | !degrees 120.0 |
| 2 | shoulder_lift_joint | revolute | shoulder_link | upper_arm_link | !degrees -360.0 | !degrees 360.0 | !degrees 120.0 |
| 3 | elbow_joint | revolute | upper_arm_link | forearm_link | !degrees -180.0 | !degrees 180.0 | !degrees 180.0 |
| 4 | wrist_1_joint | revolute | forearm_link | wrist_1_link | !degrees -360.0 | !degrees 360.0 | !degrees 180.0 |
| 5 | wrist_2_joint | revolute | wrist_1_link | wrist_2_link | !degrees -360.0 | !degrees 360.0 | !degrees 180.0 |
| 6 | wrist_3_joint | revolute | wrist_2_link | wrist_3_link | !degrees -360.0 | !degrees 360.0 | !degrees 180.0 |

`elbow_joint`의 ±180 deg는 upstream 주석이 MoveIt 계획 문제를 피하려고 실제 기구 한계의 절반으로 줄인 값이라고 적는다. 그 주석을 검증하거나 한계를 다시 쓰지 않았다.

가속도 한계는 여섯 joint 모두 `has_acceleration_limits: false`다. upstream 주석은 공개된 가속도 한계가 없다고 적는다. effort는 이번 joint contract의 필수 항목이 아니며, 소스에는 `max_effort`가 있다. shoulder_pan과 shoulder_lift는 330.0, elbow는 150.0, wrist 세 joint는 54.0이다.

고정 joint `base_joint`, `base_link-base_fixed_joint`, `base_link-base_link_inertia`, `wrist_3-flange`, `flange-tool0`, `wrist_3_link-ft_frame`은 actuated joint가 아니다.

## Frame inventory

요청한 이름은 모두 이 커밋에 서로 다른 link로 있다. `base`와 `base_link`는 같은 frame이 아니다.

| name | present | relation in this commit |
| --- | --- | --- |
| base | yes | link. `base_link-base_fixed_joint`의 child. origin rpy `0 0 ${pi}` |
| base_link | yes | link. 위 joint의 parent. xacro 주석은 REP-103 정렬이라고 적는다 |
| shoulder_link | yes | `shoulder_pan_joint`의 child |
| upper_arm_link | yes | `shoulder_lift_joint`의 child |
| forearm_link | yes | `elbow_joint`의 child |
| wrist_1_link | yes | `wrist_1_joint`의 child |
| wrist_2_link | yes | `wrist_2_joint`의 child |
| wrist_3_link | yes | `wrist_3_joint`의 child |
| flange | yes | `wrist_3-flange`의 child. parent는 `wrist_3_link` |
| tool0 | yes | `flange-tool0`의 child. parent는 `flange` |

같은 설명에 추가로 있는 link: `world` (`ur.urdf.xacro`, `base_joint`의 parent), `base_link_inertia` (`shoulder_pan_joint`의 parent), `ft_frame` (`wrist_3_link-ft_frame`의 child). `base_link`에서 `base_link_inertia`로 가는 고정 joint origin도 rpy `0 0 ${pi}`다.

`smart_panel` link는 인자가 켜질 때만 생긴다. UR10e 기본 visual YAML은 그 키를 주지 않는다.

## Nominal model boundary

이번 Golden Reference는 official nominal UR10e description이다.

사용하지 않는 표현:

```text
exact real UR10e calibration
factory-calibrated physical robot model
identical to a specific real UR10e
```

## Physics boundary

`physical_parameters.yaml`은 핀된 커밋에 있다. 아래는 소스에 적힌 값이며, 검증된 물리 거동이 아니다.

| key | source value | source comment |
| --- | --- | --- |
| shoulder_offset | 0.1762 | measured from model |
| elbow_offset | 0.0393 | measured from model |
| base_mass | 4.0 | This mass might be incorrect |
| shoulder_mass | 7.369 | |
| upper_arm_mass | 13.051 | |
| forearm_mass | 3.989 | |
| wrist_1_mass | 2.1 | |
| wrist_2_mass | 1.98 | |
| wrist_3_mass | 0.615 | |

`shoulder_radius`, `upper_arm_radius`, `elbow_radius`, `forearm_radius`, `wrist_radius` 줄은 소스 주석이 `FROM UR5 DON'T USE`라고 적는다. 그 줄은 사용하지 않는다.

이번 게이트에서 physics accuracy, payload simulation, inertia tuning, deflection, Isaac PhysX 검증은 하지 않았다.

## License

```text
Code / description license
package.xml의 license 태그는 BSD-3-Clause이고 file 속성은 LICENSE다.
LICENSE는 소스 재배포 시 저작권 고지와 조건과 면책 조항을 유지할 것,
바이너리 재배포 시 같은 고지를 문서 또는 동봉물에 실을 것,
저작권자 이름과 기여자 이름으로 파생물을 보증하거나 홍보하지 말 것을 조건으로 둔다.
소프트웨어는 "AS IS"로 제공되고 상품성, 특정 목적 적합성, 손해 배상 책임을 부인한다.
```

```text
Mesh / graphical documentation caveat
package.xml 주석은 UR8LONG, UR15, UR18, UR20, UR30, UR10g-1750, UR17g-1300,
UR18g-950, common Smart Panel mesh를 Graphical Documentation으로 부르며
Universal Robots의 Terms and Conditions for Use of Graphical Documentation을 가리킨다.
그 주석의 이름 목록에 UR10e는 없다.
meshes/ur10e 아래에는 LICENSE.txt가 없다.
meshes/common/smart_panel/LICENSE.txt는 그 Graphical Documentation 조건 본문이다.
UR10e 기본 visual 설정은 Smart Panel을 켜지 않으며, 그 mesh는 이번 복사에 넣지 않았다.
UR10e mesh에 별도 LICENSE.txt가 없다는 사실을 제약 없음으로 해석하지 않는다.
```

## Integrity

```text
Pinned upstream commit
7ce98d8c3512b9d0239da93b75de68ea64f61bd6

Local source location
robot_description/upstream/7ce98d8c3512b9d0239da93b75de68ea64f61bd6/

SHA-256
같은 디렉터리의 SHA256SUMS.txt. 파일 23개. shasum -a 256 -c 통과.
```

로컬 복사는 위 entry point, macro, `ur_common.xacro`, UR10e config 네 파일, `LICENSE`, `package.xml`, `meshes/ur10e` visual/collision이다. xacro와 YAML과 mesh를 수정하지 않았다. 다른 기종 mesh와 `urdf/inc`의 ros2_control 파일은 복사하지 않았다.

## Rejected sources

비공식 URDF 저장소는 Golden Reference로 쓰지 않았다. 검토한 공식 소스는 `UniversalRobots/Universal_Robots_ROS2_Description`의 `jazzy`뿐이고, 핀은 그 브랜치의 조회 시점 커밋이다.
