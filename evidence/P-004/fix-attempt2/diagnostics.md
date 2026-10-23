# P-004 Fix Attempt 2 diagnostics

기준선은 `evidence/P-004/fix-attempt1`이다. 그 파일은 바꾸지 않았다. Transform consistency는 그 attempt에서 이미 PASS다.

## Attribution

`p_isaac - p_unity`와 `FK(q_isaac) - FK(q_unity)`의 벡터 차이의 norm은 다섯 configuration에서 최대 0.003836883818283858 mm다.

```text
                sim TCP mm    FK(q) mm     residual mm
A-zero          1.242938      1.242907     0.002771
B-positive-1    1.017114      1.017852     0.002958
C-mixed-1       1.193059      1.192225     0.003263
E-fixed-1       1.252646      1.251944     0.003359
E-fixed-3       1.239093      1.237569     0.003837
```

남은 1.252646 mm는 settled joint difference다.

단일 joint를 Isaac actual에서 Unity actual로 바꾼 FK 변위가 가장 큰 joint는 다섯 configuration 모두 `shoulder_lift_joint`다. A-zero에서 그 기여는 1.1050 mm이고, finite-difference Jacobian에 `Δq`를 곱한 값과 같다. elbow는 0.1446 mm다.

A-zero `shoulder_lift_joint`:

```text
Unity actual     0.0002858018851839006 rad
Isaac actual     0.0012141603510826826 rad
Unity - Isaac   -0.000928358465898782 rad
                -0.05319102197123997 deg
```

## Equilibrium

A-zero, gravity ON, stiffness 100000, damping 1000:

```text
gravity compensation shoulder_lift   -121.28190612792969 Nm
max effort shoulder_lift              330 Nm
K * q_error                           100000 * 0.0012141603510826826 = 121.416 Nm
get_dof_efforts                       0, 0, 0, 0, 0, 0
```

Implicit drive force는 `get_dof_efforts`에 나타나지 않았다. Holding torque는 gravity compensation이고, 330 Nm limit 아래다.

Gravity magnitude 0에서는 여섯 joint error가 모두 0이고 20 step에서 settle했다. Terminal velocity는 0이다.

Solver position/velocity attribute는 articulation root에 authoring되어 있지 않다. `set_solver_iteration_counts`는 root joint attribute의 typeName이 비어 있어 실패했다. Stage에서 값이 읽히는 prim은 없었다. Iteration 숫자는 적지 않는다.

Stiffness만 올리고 damping 1000은 유지했다.

```text
stiffness    shoulder_lift error rad    elbow error rad    steps    timeout    max |velocity|
100000       0.0012141603510826826      0.0003736567741725594   43   false    0
200000       0.0005065396544523537      0.00016371588571928442  43   false    0
400000       0.0001482409716118127      4.86294120491948e-05    43   false    0
```

200000은 settle step 수와 timeout을 바꾸지 않고 shoulder_lift 잔차를 줄인 가장 작은 측정 step이다. 400000은 더 줄었지만 최종 조건으로 쓰지 않았다. Damping과 timestep은 바꾸지 않았다.

## 최종 재측정

Isaac importer stiffness 200000, damping 1000, articulation root relocation 유지, gravity ON.

```text
Python ↔ Unity position     0.0040523245387939092 mm
Python ↔ Unity orientation  0.00062471732598054931 deg
Python ↔ Isaac position     0.00056994037914351162 mm
Python ↔ Isaac orientation  5.5449295521625546e-05 deg
Unity ↔ Isaac position      0.29056228373221366 mm
Unity ↔ Isaac orientation   0.019650595706166018 deg
```

Unity joint target error 최대는 A-zero `wrist_1_joint` 0.0006152464775368571 rad, 0.035251026523152325 deg다.
Isaac joint target error 최대는 A-zero `shoulder_lift_joint` 0.0005065396544523537 rad, 0.029022584356134966 deg다.
Timeout은 Unity 0, Isaac 0이다. A-zero fresh rerun의 joint, TCP position, TCP orientation 차이는 양쪽 모두 0이다.
