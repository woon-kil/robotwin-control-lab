# P-002 Unity UR10e joint control

```text
Gate
P-002

Result
FAIL

Robot
UR10e Golden Reference

Source commit
7ce98d8c3512b9d0239da93b75de68ea64f61bd6

Configurations
5

Global max joint error
0.014747420325875282 rad
0.8449649433780988 deg

Engineering target
< 0.1 deg

Unity version
2022.3.62f2

URDF Importer
0.5.2-preview
90f353e4352aae4df52fa2c05e49b804631d2a63
```

측정값과 Engineering Target은 별개다. 이 숫자는 Unity articulation feedback이다. Python FK와 맞다는 판정은 아니다.

## Excluded P-001B configurations

D-near-upper와 D-near-lower는 ±360 deg 또는 ±180 deg 한계의 1% 안쪽이다. 이번 Gate의 drive는 PROJECT TEST PARAMETER라 limit 근처의 winding을 넣지 않았다.

B-positive-2, C-mixed-2, E-fixed-2, E-fixed-4는 같은 이유의 더 큰 이동이다. zero, positive, mixed, fixed, additional은 A-zero, B-positive-1, C-mixed-1, E-fixed-1, E-fixed-3으로 포함했다.

## Known limitations

- official nominal UR10e model 기반
- robot-specific calibration이 아님
- 실제 UR10e 측정값과 비교하지 않음
- Isaac Sim과 비교하지 않음
- Python FK와 최종 정확도 비교를 하지 않음
- physical robot accuracy를 주장하지 않음
- tool0 pose는 unity_world이며 canonical base_link로 변환하지 않음
- drive stiffness/damping과 collider disable은 PROJECT TEST PARAMETER

## Failures

```text
A-zero FAIL max_abs_error_rad=0.014747420325875282
B-positive-1 FAIL max_abs_error_rad=0.0064703345298767201
C-mixed-1 FAIL max_abs_error_rad=0.011876249313354514
E-fixed-1 FAIL max_abs_error_rad=0.011951950192451488
E-fixed-3 FAIL max_abs_error_rad=0.014625303000211715
```
