# P-004 diagnostics

TCP는 UR10e `tool0`이다. `bolt_tool_tcp`가 아니다. 비교 프레임은 `base_link`다.

`T_base_tool0 = inverse(T_world_base) * T_world_tool0`.

## 관측

Unity raw는 `unity_world`다. base_link position은 `(0, 0, 0)`, quaternion xyzw는 `(0, 0, 0, 1)`이다. 변환은 URDF Importer 0.5.2 `90f353e4352aae4df52fa2c05e49b804631d2a63`의 `BuiltInExtensions.Unity2Ros`다.

```text
position (x, y, z)_canonical = (z, -x, y)_unity
quaternion xyzw_canonical = (-z, x, -y, w)_unity
```

`chosenAxis zAxis`는 visual과 collision mesh만 돌린다. link transform에는 곱하지 않았다. 출처는 같은 패키지의 `UrdfRobotExtensions.CorrectAxis`다.

Isaac raw는 `isaac_world`다. pose API는 `UsdGeom.XformCache.GetLocalToWorldTransform`다. `RigidPrim`은 만들지 않았다. quaternion은 API의 wxyz를 canonical xyzw로 재배열만 했다. base_link world position은 `(0, 0, 0)`, quaternion wxyz는 `(1, 0, 0, 0)`이다.

## 배제한 큰 오류

다섯 configuration에서 Isaac tool0 위치는 약 1 m 근처이고, 자세 오차는 0.07 deg 아래다. meter와 millimeter가 바뀌거나, world와 base가 다르거나, 축이 90 deg 또는 180 deg 돌아간 상태는 아니다. tool0 local translate는 0이고, local quaternion wxyz `(0.5, 0.5, 0.5, 0.5)`는 URDF `flange-tool0` rpy `(pi/2, 0, pi/2)`와 같다.

## 남은 차이

Python FK는 각 시뮬레이터의 actual joint를 입력으로 한다.

```text
Python ↔ Unity position max     0.004052324538793909 mm
Python ↔ Unity orientation max  0.0006247173259805493 deg

Python ↔ Isaac position max     1.4705603411282602 mm
Python ↔ Isaac orientation max  0.0666726093171082 deg

Unity ↔ Isaac position max      3.1548989314166063 mm
Unity ↔ Isaac orientation max   0.14790147537513967 deg
```

A-zero에서 Isaac tool0에서 Python FK(Isaac actual joint)를 뺀 값은 약 `(0.058, 0.066, -1.361)` mm다. 다른 configuration도 z 성분이 가장 크다. 이 크기는 축 치환으로 설명되지 않는다. 자동 분류는 `UNKNOWN`이다.

Unity actual joint의 FK와 Unity TCP는 0.005 mm 아래다. Isaac XformCache pose는 같은 방식으로 계산한 FK보다 0.5 mm를 넘는다. Unity와 Isaac TCP 차이는 1.0 mm를 넘는다. target은 완화하지 않았다.

A-zero fresh rerun의 joint, tool0 position, tool0 orientation 차이는 Unity와 Isaac 모두 0이다.

## End-to-end

Target joint의 Python FK와 simulator TCP 차이는 kinematic 비교와 별도다. joint tracking error가 포함된다. 이 숫자로 Python FK를 수정하지 않았다.
