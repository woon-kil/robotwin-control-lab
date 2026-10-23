# P-002 Fix Attempt 2 diagnostics

이 문서는 Attempt 2 final을 고르기 전에 측정한 진단이다. G-OFF, root movable, collider enabled, stiffness sweep, timestep/solver probe는 `DIAGNOSTIC ONLY`다. 최종 PASS 조건이 아니다.

최종 조건은 gravity ON, actual articulation feedback, stiffness 1000000, damping 2000, URDF effort force limit, `base_link` immovable, solver position iterations 24, solver velocity iterations 1이다. 이 상태는 실제 UR10e controller를 재현하지 않는다.

## 측정 조건

```text
fixed timestep
0.019999999552965164 s

gravity
(0, -9.81, 0) Unity Y-up

settling threshold
position delta < 1e-5 rad

velocity threshold
reported |jointVelocity| < 0.001 rad/s

consecutive frames
20

timeout
8 s
```

Attempt 2A와 진단의 drive는 stiffness 1000000, damping 2000이다. force limit은 runner가 바꾸지 않는다. 샘플된 Unity `forceLimit`은 imported URDF effort와 같다.

```text
joint                  URDF effort / forceLimit
shoulder_pan_joint     330
shoulder_lift_joint    330
elbow_joint            150
wrist_1_joint          54
wrist_2_joint          54
wrist_3_joint          54
```

`ArticulationBody.driveForce[0]`는 읽혔다. `jointForce[0]`는 샘플에서 0이었다. jointForce를 토크 근거로 쓰지 않는다.

## G-ON — CURRENT CONDITION

gravity ON, root fixed, colliders disabled. 5개 configuration 모두 401 step에서 timeout이다. 위치는 약 0.4 s 이후 변하지 않았다.

A-zero `shoulder_lift_joint`:

```text
target rad            0
actual rad            0.00261879
error                 0.150046 deg
reported velocity     -0.0149255 rad/s
drive stiffness       1000000
drive damping         2000
force limit           330
drive force           -156.083 Nm
|force| / limit       0.47298
child mass            13.051 kg
inertia xyz           0.0278114, 0.769762, 0.770677
elapsed               8.02 s
timed out             true
```

같은 시각의 다른 joint error:

```text
shoulder_pan_joint    +0.000055 deg    drive force +0.000578 Nm
elbow_joint           +0.008520 deg    drive force -56.079 Nm    limit 150
wrist_1_joint         +0.049125 deg    drive force +0.024601 Nm    limit 54
wrist_2_joint         +0.000484 deg    drive force -0.015751 Nm
wrist_3_joint         -0.000012 deg    drive force -0.000207 Nm
```

`shoulder_lift_trace.json`에서 A-zero shoulder_lift error는 t=0.40 s의 0.14902 deg 이후 8 s까지 0.15005 deg에 머문다. 같은 구간의 reported velocity는 -0.01493 rad/s로 남는다. 위치 변화로 환산한 속도는 그 값과 맞지 않는다. timeout은 위치가 움직여서가 아니라 reported `jointVelocity`가 0.001 rad/s 아래로 내려가지 않아서 발생했다.

## G-OFF — DIAGNOSTIC ONLY

같은 target, root fixed, colliders disabled, gravity (0, 0, 0).

```text
A-zero        timeout false   max error 0 deg
B-positive-1  timeout false   max error 3.38554e-06 rad
C-mixed-1     timeout false   max error 5.06043e-06 rad
E-fixed-1     timeout false   max error 1.01805e-05 rad
E-fixed-3     timeout true    max error 0.000219964 rad
```

E-fixed-3의 위치 오차는 0.0126 deg다. timeout은 잔여 reported velocity 때문이다. gravity를 끄면 shoulder_lift의 0.150 deg 오차와 -156 Nm drive force가 사라진다. G-OFF는 최종 PASS로 쓰지 않았다.

## Root와 collider — DIAGNOSTIC ONLY

Imported prefab의 root는 `base_link`이고 immovable은 false였다. Attempt 2A runner는 측정 전에 immovable을 true로 바꿨다. 그래서 0.150 deg는 떠 있는 base의 오차가 아니다.

A-zero, gravity ON, root movable, colliders disabled:

```text
root displacement     0.822573 m
max joint error       5.78358e-06 rad
timed out             false
steps                 20
```

Base가 떨어지면 joint 상대 오차가 줄어 보인다. Golden Reference joint 시험의 base는 고정해야 한다. Attempt 2 final은 `base_link` articulation의 `m_Immovable`을 prefab에 1로 저장했다. 최종 측정의 `root_was_immovable_before_test`는 true다.

A-zero, gravity ON, root fixed, colliders enabled:

```text
max error             0.00261879 rad
drive force           -156.083 Nm
timed out             true
```

Collider를 켜도 A-zero 오차는 collider를 끈 측정과 같다. 이번 Gate는 collision validation이 아니다. 최종 시험도 collider 7개를 끈다.

## Stiffness / damping — DIAGNOSTIC ONLY

A-zero, gravity ON, root fixed, 8 s. drive force는 모든 칸에서 약 -156.08 Nm이다. limit 330에 닿지 않는다.

```text
stiffness   damping   error deg   reported velocity   position settled step
1000000     500       0.140244    -0.014911           48
1000000     2000      0.150046    -0.014925           49
1000000     8000      0.189251    -0.014967           46
2000000     500       0.135507    -0.015018           48
2000000     2000      0.140409    -0.015021           48
2000000     8000      0.160016    -0.015034           44
3000000     500       0.133931    -0.015054           48
3000000     2000      0.137200    -0.015056           48
3000000     8000      0.150271    -0.015061           43
```

Stiffness를 3배로 올려도 오차 바닥은 0.134 deg 아래가 아니다. Damping을 올리면 오차가 커진다. 이 행렬은 최종 parameter로 채택하지 않았다. 무한 stiffness는 쓰지 않았다.

## Solver — DIAGNOSTIC ONLY

A-zero, stiffness 1000000, damping 2000, gravity ON, 2 s.

```text
dt      position iterations   velocity iterations   error deg   reported velocity   drive force
0.02    6                     1                     0.150035    -0.014926           -156.077
0.01    6                     1                     0.045507    -0.007201           -156.117
0.005   6                     1                     0.017906    -0.003180           -156.129
0.02    12                    1                     0.053105    -0.004624           -156.257
0.02    24                    1                     0.016375    -0.000613           -156.101
0.02    6                     4                     0.125421    -0.040336           -156.090
0.01    12                    4                     0.018625    -0.006061           -156.120
```

오차는 drive force가 아니라 solver position iterations와 timestep에 따라 줄어든다. drive force는 156 Nm 근처로 유지되고 limit의 47%다.

iterations 12는 위치 오차 0.053 deg로 0.1 deg 아래지만 reported velocity 0.004624 rad/s가 settle 기준 0.001보다 크다. iterations 24는 오차 0.016375 deg이고 reported velocity 0.000613 rad/s다. timestep 0.005도 오차는 0.018 deg지만 velocity가 0.001보다 커서 기존 settle 규칙을 통과하지 못한다. velocity iterations 4는 오차와 reported velocity를 줄이지 못했다.

## 질문

1. shoulder_lift 오차가 가장 컸던 이유는 gravity load다. G-ON A-zero에서 이 joint만 0.150 deg와 -156 Nm이었고, pan과 wrist_2, wrist_3은 0.001 deg 이하였다. G-OFF에서는 그 오차와 토크가 사라졌다. force limit saturation은 아니다.

2. Gravity OFF에서 A-zero 오차는 0 deg가 됐다. 다른 configuration의 max error는 1.02e-5 rad 이하다. E-fixed-3만 reported velocity 때문에 timeout이었고, 위치 오차는 0.000220 rad였다.

3. Root를 고정하지 않으면 base가 0.823 m 움직이고 joint 오차는 5.8e-6 rad까지 줄어 보인다. 0.150 deg 실패 측정은 이미 runtime에서 root를 고정한 값이다. prefab의 `base_link` immovable은 그 고정을 자산에 명시한 것이다. 그 변경만으로 0.150 deg가 0.1 deg 아래로 내려가진 않았다.

4. forceLimit saturation 증거는 없다. shoulder_lift |drive force| / 330 = 0.473이다. elbow는 56 / 150이다. wrist drive force는 1 Nm 아래다.

5. Stiffness와 damping을 최종 수정으로 바꾸지 않았다. 3×3 행렬에서 오차 바닥이 0.134 deg보다 낮아지지 않았고, damping을 키우면 오차가 커졌다.

6. 최종 parameter는 solver position iterations 24다. 6, 12, 24의 측정에서 24만 위치 오차와 reported velocity 기준을 같이 넘었다. stiffness 1000000, damping 2000, URDF force limit, gravity ON, timeout 8 s, settle 규칙은 Attempt 2A와 같다. 최종 5개 configuration의 global max는 0.0006152464775368571 rad, 0.035251026523152325 deg다. timeout은 0이다. 최종 max를 만든 joint는 A-zero의 `wrist_1_joint`다. shoulder_lift A-zero 오차는 0.016375 deg로 줄었다.

7. 아니다. 최종 상태는 실제 UR10e controller를 재현하지 않는다. Unity Golden Reference simulation의 solver iteration 조정이다.
