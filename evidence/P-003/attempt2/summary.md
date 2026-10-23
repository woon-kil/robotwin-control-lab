# P-003 Isaac UR10e joint control

```text
Gate
P-003

Result
FAIL

Robot
UR10e Golden Reference

Source commit
7ce98d8c3512b9d0239da93b75de68ea64f61bd6

Isaac Sim
6.2.0-rc.14+release.52805.02189a84.gl

Articulation root
/ur10e/Geometry/world/base_link/base_link_inertia

Configurations
5

Global max joint error
1.6746309041976928 rad
95.9492830527047 deg

Engineering target
< 0.1 deg

Timeouts
1

Known limitations
official nominal UR10e model
not robot-specific calibration
not compared with actual UR10e
not yet compared with Unity
not yet compared with Python FK
no physical robot accuracy claim
Isaac controller/drive is simulation test configuration
```

이 숫자는 Isaac PhysX articulation feedback이다. Python FK 정확도나 Unity와의 우열이 아니다.
