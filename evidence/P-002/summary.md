# P-002 Unity UR10e joint control

```text
Gate
P-002

Result
PASS

Robot
UR10e Golden Reference

Source commit
7ce98d8c3512b9d0239da93b75de68ea64f61bd6

Configurations
5

Global max joint error
0.00061524647753685713 rad
0.035251026523152325 deg

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
- drive stiffness/damping, collider disable, solver iterations는 PROJECT TEST PARAMETER
- 이 측정은 실제 UR10e controller를 재현하지 않음

## Attempt history

```text
Attempt 1
FAIL
0.014747420325875282 rad
Attempt 2A
FAIL
0.0026187931653112173 rad
Attempt 2 final
PASS
0.00061524647753685713 rad
0.035251026523152325 deg
```

Attempt 1은 stiffness 100000, damping 10000, timeout 5s, velocity epsilon 0.01 rad/s, solver iterations 6이다. Attempt 2A는 stiffness 1000000, damping 2000, timeout 8s, velocity epsilon 0.001 rad/s, solver iterations 6이다. Attempt 2 final은 Attempt 2A와 같은 drive, timeout, gravity ON이고 solver position iterations만 24다.

## Cause

A-zero shoulder_lift는 gravity ON에서 약 0.4초 뒤에 위치가 멈추지만 오차는 0.150 deg에 남았다. 그때 drive force는 약 156 Nm이고 URDF effort 330 Nm에 닿지 않았다. stiffness 1000000/2000000/3000000과 damping 500/2000/8000은 그 바닥을 0.134 deg 아래로 내리지 못했다. solver position iterations 6, 12, 24에서는 같은 A-zero 오차가 0.150 deg, 0.053 deg, 0.016 deg였다. iterations 24에서 reported joint velocity도 0.001 rad/s 아래였다. timestep 0.005는 오차를 0.018 deg까지 줄였지만 reported velocity가 0.001 rad/s보다 커서 settle 조건을 통과하지 못했다. velocity iterations 4는 오차를 줄이지 못했다. force limit, 무한 stiffness, gravity OFF는 최종 조건으로 쓰지 않았다. 상세 숫자는 evidence/P-002/attempt2/diagnostics.md다.
