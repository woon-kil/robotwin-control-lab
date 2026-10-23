# P-003 Fix Attempt 1 diagnostics

측정 호스트는 p620이다. Isaac Sim VERSION 파일은 `6.2.0-rc.14+release.52805.02189a84.gl`. 이 문서는 `state_trace.json`과 재측정 `joint_results.json`에 있는 값만 적는다.

## 질문

Isaac articulation의 joint feedback과 link/tool0 world pose가 같은 physical state인가.

## Articulation root

`base_link_inertia` path는 `/ur10e/Geometry/world/base_link/base_link_inertia`.

- type: Xform
- parent: `/ur10e/Geometry/world/base_link`
- schemas: IsaacLinkAPI, NewtonArticulationRootAPI, NewtonMassAPI, PhysicsArticulationRootAPI, PhysicsMassAPI, PhysicsRigidBodyAPI
- RigidBodyAPI: 있음
- ArticulationRootAPI: 있음

`world`와 `base_link`는 Xform이고 RigidBodyAPI와 ArticulationRootAPI가 없다. local translate는 둘 다 `(0, 0, 0)`.

Fixed joint는 `/ur10e/Physics/tn__base_linkbase_link_inertia_kc0` 하나다. body0는 `/ur10e/Geometry/world/base_link`, body1는 `base_link_inertia`. body0는 rigid body가 아니다. 이 관계는 기록한다. tool0 z 누적의 원인은 아니다.

`base`라는 이름의 Geometry prim은 두 개다.

- `/ur10e/Geometry/world/base_link/base_link_inertia/base`
- `/ur10e/Geometry/world/base_link/base`

base pose는 이 둘을 고르지 않고 `base_link`를 읽는다.

## tool0 identity

path는 `.../wrist_3_link/flange/tool0`.

- type: Xform
- parent: flange
- schemas before RigidPrim: 없음
- RigidBodyAPI: 없음
- local translate: `(0, 0, 0)`
- local quaternion wxyz: `(0.4999999999999997, 0.5000000000000001, 0.5000000000000001, 0.5000000000000001)`

flange도 Xform이고 RigidBodyAPI가 없다. local translate는 `(0, 0, 0)`. flange와 tool0 사이 USD 거리는 다섯 configuration 모두 `0.0`이다. configuration마다 바뀌는 fixed transform이 아니다.

## 어디서 무너지는가

`mode_b_continuous`는 RigidPrim(tool0)를 만들지 않았다. observe 뒤 `tool0_has_rigid_body_after_observe`는 false다.

A-zero의 USD adjacent distance:

```text
base_link_inertia -> shoulder_link   0.18069994449616023 m
shoulder_link -> upper_arm_link      3.331384502767203e-08 m
upper_arm_link -> forearm_link       0.6127000451087954 m
forearm_link -> wrist_1_link         0.5974928041840155 m
wrist_1_link -> wrist_2_link         0.11984998305043866 m
wrist_2_link -> wrist_3_link         0.11654996871948617 m
wrist_3_link -> flange               0.0 m
flange -> tool0                      0.0 m
```

재측정 `link_distance_span_max_m`은 `2.3841857632600494e-07` m. 한 link부터 체인이 찢어진 것이 아니다. world부터 tool0까지 같이 움직인다.

A-zero articulation root position은 약 `8.028719093999825e-06` m. root linear velocity는 `[-0.009205913171172142, -0.0004456978349480778, -0.00020922422118019313]` m/s. 이 속도는 40 m 이동의 원인이 아니다.

## Hypothesis

| ID | 판정 | 근거 |
| --- | --- | --- |
| H1 | 배제 | fresh USD tool0 z와 continuous USD tool0 z가 같다. A-zero `0.05750801414253482`. E-fixed-3 fresh `0.05124803632934593`, continuous `0.05124436318525982`. |
| H2 | joint teleport는 배제 | XformCache tool0 xyz는 configuration마다 바뀐다. |
| H3 | articulation residual은 배제 | root linear speed는 0.01 m/s 근처다. RigidPrim이 만든 body만 중력 속도를 쌓는다. |
| H4 | 확정 | tool0 Xform chain과 RigidPrim이 만든 body는 다른 상태다. |
| H5 | 배제 | path는 flange 아래 imported tool0다. |
| H6 | 확정 | `RigidPrim.__init__`는 RigidBodyAPI가 없으면 schema를 적용한다. |
| H7 | 배제 | pose는 settle 뒤에 읽었다. 하강은 그 다음 physics update에서 생긴다. |
| H8 | z 누적 원인으로는 배제 | fixed joint와 hierarchy는 arm을 붙잡고 있다. root 이동은 마이크로미터다. |

## Mode A와 Mode B

Mode B continuous tool0 USD z:

```text
A-zero       0.05750801414253482
B-positive-1 -0.4754112665479662
C-mixed-1     0.3779478966444301
E-fixed-1     0.14369448988255593
E-fixed-3     0.05124436318525982
```

Mode B fresh tool0 USD z:

```text
A-zero       0.05750801414253482
B-positive-1 -0.4754113549532315
C-mixed-1     0.3779531419934758
E-fixed-1     0.14370396750839465
E-fixed-3     0.05124803632934593
```

누적 하강은 fresh state에서도 생기지 않는다. cross-test contamination이 아니다.

Mode A는 다섯 joint command의 재실행이 아니다. E-fixed-3 이후 `RigidPrim(tool0)`를 한 번 만들고, joint command 없이 `simulation_app.update()` 43회를 네 번 더 했다. 그 rigid z:

```text
0.051248036324977875
-2.526602029800415
-10.142973899841309
-22.79786491394043
-40.491275787353516
```

같은 구간의 USD z도 같이 내려간다. `tool0_has_rigid_body_after_observe`는 true다. 기존 `evidence/P-003/joint_results.json`의 E-fixed-3 z `-40.485015869140625`와 같은 자유낙하다.

## Observation API

tool0는 rigid body가 아니다. `isaacsim.core.experimental.prims.RigidPrim.get_world_poses`를 쓰지 않는다. 그 생성자는 없는 RigidBodyAPI를 만들고, 그 body는 articulation link가 아니라서 중력으로 떨어진다. 실행 중 RigidPrim 생성은 physics view도 무효로 만든다.

재측정 pose API는 `UsdGeom.XformCache.GetLocalToWorldTransform`이다. quaternion은 `Gf.Quatd`의 real 다음 imaginary, 즉 wxyz다. frame은 `isaac_world`다.

## Sanity

`tool0_base_distance_max_m`은 `1.3166888748578596`. limit `2.0` m는 PROJECT SANITY CHECK다. UR10e 공식 reach 1300 mm보다 위이고, 측정된 약 1.32 m를 받아들이며 40 m는 거부한다. FK accuracy target이 아니다.

근거: https://www.universal-robots.com/manuals/EN/HTML/SW5_26/Content/prod-usr-man/complianceUR10e/H_g5_sections/appendix_g5/tech_spec_sheet.htm

adjacent span limit `0.001` m도 structural integrity 검사다. 측정 span은 `2.3841857632600494e-07` m.

## 재측정

`cumulative_world_drift`는 false. timeouts는 0. global max joint error는 `0.0015286624431610218` rad, `0.08758590629328365` deg. A-zero fresh rerun의 joint 차, tool0 position 차, tool0 orientation 차는 모두 `0.0`이다.

drive stiffness와 damping은 바꾸지 않았다.
