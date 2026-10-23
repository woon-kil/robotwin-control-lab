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
0.0026187931653112173 rad
0.15004579579003846 deg

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

## Previous measurement

attempt1은 stiffness 100000, damping 10000, timeout 5s, velocity epsilon 0.01 rad/s였다. 결과는 FAIL이고 global max joint error는 0.014747420325875282 rad다. 오차는 shoulder_lift에 몰려 있고 shoulder_pan, wrist_2, wrist_3은 그보다 작았다. force limit은 URDF effort 그대로였다. damping이 남은 토크를 상쇄해 0.1 deg 밖에서 멈추거나 timeout이 났다. 이번 측정은 stiffness 1000000, damping 2000, timeout 8s, velocity epsilon 0.001 rad/s다. 무한 강성은 쓰지 않았다.

## Cause

가장 큰 오차는 shoulder_lift다. shoulder_pan과 wrist_2, wrist_3은 그보다 작다. drive force limit은 URDF effort를 유지했다. stiffness를 유한한 값으로 올린 뒤에도 shoulder_lift는 gravity torque와 그 effort limit 사이에서 0.1 deg 안으로 들어오지 않았고, velocity epsilon을 연속으로 만족하지 못해 timeout이 됐다. force limit을 올리거나 stiffness를 무한대로 두지 않았다.

## Failures

```text
A-zero FAIL max_abs_error_rad=0.0026187931653112173
B-positive-1 FAIL max_abs_error_rad=0.0011127412319183461
C-mixed-1 FAIL max_abs_error_rad=0.0020964682102203591
E-fixed-1 FAIL max_abs_error_rad=0.002110925316810619
E-fixed-3 FAIL max_abs_error_rad=0.002595802322030067
```
