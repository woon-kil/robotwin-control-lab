# P-002 environment

```text
OS
Mac OS X 26.6.2

Unity Editor version
2022.3.62f2

URDF Importer version
0.5.2-preview
90f353e4352aae4df52fa2c05e49b804631d2a63

Import method
Unity.Robotics.UrdfImporter.UrdfRobotExtensions.Create
chosenAxis zAxis
convexMethod unity

Source model
robot_description/generated/7ce98d8c3512b9d0239da93b75de68ea64f61bd6/ur10e.urdf

Source commit
7ce98d8c3512b9d0239da93b75de68ea64f61bd6

source repository
UniversalRobots/Universal_Robots_ROS2_Description
robot
ur10e
source manifest
robot_description/SOURCE_MANIFEST.md
physics timestep
0.0199999996
gravity
0 -9.81000042 0

test runner
/Applications/Unity/Hub/Editor/2022.3.62f2/Unity.app/Contents/MacOS/Unity -batchmode -nographics -projectPath unity/UR10eJointBaseline -runTests -testPlatform EditMode -testCategory Ur10eJointBaseline -testResults evidence/P-002/unity-test-results.xml
```

## Unit boundary

Canonical joint command와 feedback은 radian이다. `ArticulationDrive.target`은 angular motion에서 degree다. 변환은 `Ur10eJointAdapter.SetJointTargetsRadians`에서 `radian * 180 / pi`로만 한다. `ArticulationBody.jointPosition[0]`은 radian이라 feedback에 추가 변환을 하지 않는다.

근거: https://docs.unity3d.com/2022.3/Documentation/ScriptReference/ArticulationDrive-target.html
근거: https://docs.unity3d.com/2022.3/Documentation/ScriptReference/ArticulationBody-jointPosition.html

## Coordinate boundary

어댑터는 TCP pose를 canonical base_link로 다시 바꾸지 않는다. 기록하는 tool0은 Unity world pose다. 위치 단위는 meter, quaternion은 Unity `(x, y, z, w)`다.

링크 배치와 관절 축은 URDF Importer가 import 때 적용한다. 벡터 `Ros2Unity`은 `(-rosY, rosZ, rosX)`다. quaternion은 `(rosY, -rosZ, -rosX, rosW)`다. origin Euler degree는 `(pitch, -yaw, -roll)`이다. revolute twist는 parent anchor의 X축이고, importer가 `SetFromToRotation(+X, -Ros2Unity(axis))`로 anchor를 돌린다.

`chosenAxis=zAxis`는 importer의 mesh up-axis 설정이다. source mesh의 up axis가 Z이기 때문에 그 값을 넘겼다. 이 설정은 mesh visual/collision 보정이며 canonical joint radian을 바꾸지 않는다.

## PROJECT TEST PARAMETER

```text
drive stiffness
1000000
drive damping
2000
force limit
imported URDF effort, unchanged
colliders disabled
7
root was immovable before test
false
settled frames
20
velocity epsilon rad/s
0.001
position delta epsilon rad
1.0000000000000001E-05
timeout seconds
8
```

Importer가 남긴 stiffness/damping은 import_report.json의 imported_stiffness, imported_damping이다. runner가 그 값을 산업 사양으로 쓰지 않고, 측정 중에만 위 PROJECT TEST PARAMETER를 drive에 넣는다. force limit은 바꾸지 않는다.
