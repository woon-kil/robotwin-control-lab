# P-004 Cross-simulator TCP

```text
Gate
P-004

Result
FAIL

Robot
UR10e Golden Reference

Source commit
7ce98d8c3512b9d0239da93b75de68ea64f61bd6

Configurations
5

Python ↔ Unity
Max Position:
0.0040523245387939092 mm
Max Orientation:
0.00062471732598054931 deg

Python ↔ Isaac
Max Position:
1.4705603411282602 mm
Max Orientation:
0.066672609317108195 deg

Unity ↔ Isaac
Max Position:
3.1548989314166063 mm
Max Orientation:
0.14790147537513967 deg

Deterministic rerun:
PASS

Regression:
PASS

Known limitations:
official nominal UR10e model
not robot-specific calibration
not compared with actual UR10e
no physical robot accuracy claim
TCP is tool0, not bolt_tool_tcp
Unity ↔ Isaac orientation has no acceptance target in REQUIREMENTS or VALIDATION_PLAN
```

이 숫자는 동일 nominal UR10e 모델에 대한 독립 Python FK와 Unity/Isaac TCP transform consistency다. 실제 로봇 정확도가 아니다.
