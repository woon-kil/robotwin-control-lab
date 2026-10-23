# P-004 Fix Attempt 1 diagnostics

비교 입력은 Isaac `get_dof_positions`의 full-precision actual joint다. Python intermediate frame은 같은 `chain_frames` chain이다. `forward_kinematics`의 joint motion 식은 바꾸지 않았다. Pose는 `UsdGeom.XformCache.GetLocalToWorldTransform`이고 `RigidPrim`은 만들지 않았다.

## A-zero post-settle, root API on base_link_inertia

이 표는 relocation 전의 runtime chain이다. 단위는 cumulative error.

```text
Frame             Pos Error mm    Ori Error deg
------------------------------------------------
base_link_inertia 0.014071         0.064685
shoulder_link     0.212208         0.064649
upper_arm_link    0.212225         0.064890
forearm_link      0.735862         0.040669
wrist_1_link      1.373145         0.040850
wrist_2_link      1.359500         0.064920
wrist_3_link      1.363292         0.064920
flange            1.363292         0.064920
tool0             1.363292         0.064920
```

A-zero tool0 residual은 dx +0.058 mm, dy +0.066 mm, dz -1.361 mm이다.

Orientation error는 `base_link_inertia`에서 약 0.065 deg로 시작하고 이후 link의 local orientation error는 0.003 deg 아래다. Position error는 그 회전이 팔 길이로 증폭된 값이다. `tool0` local error는 0이다.

같은 첫 frame이 다섯 configuration의 pre-step, post-settle, injected settled joints에서 반복된다. pre-step tool0도 이미 1.610 mm다. Authored local과 Python local at q=0의 orientation error는 0 deg다. 차이는 URDF→USD authoring이 아니라 USD→PhysX runtime에서 생긴다.

## Joint origin

`base_link-base_link_inertia`

```text
URDF origin xyz = 0 0 0
URDF origin rpy = 0 0 pi
Python local quaternion xyzw = 0 0 1 ~0
Isaac authored local quaternion wxyz = ~0 0 0 1
PhysX FixedJoint localRot0 = w ~0, z 1
PhysX FixedJoint localRot1 = identity
body0 = /ur10e/Geometry/world/base_link          rigid_body_api false
body1 = /ur10e/Geometry/world/base_link/base_link_inertia
```

`base_link`는 inertia가 없는 ghost link다. Importer는 ArticulationRootAPI를 첫 rigid body인 `base_link_inertia`에 둔다. `apply_fix_base`는 body0가 non-rigid인 이 fixed joint를 이미 있는 world anchor로 보고 relocation을 건너뛴다. 설치된 `fix_articulation_root_for_fixed_base`는 그 API를 fixed joint prim으로 옮긴다.

Inertial origin of `base_link_inertia`는 xyz 0, rpy 0이다. 0.014 mm / 0.065 deg residual은 COM offset이 아니다. visual과 collision mesh는 비교 frame으로 쓰지 않았다.

## Residual direction

post-settle tool0, relocation 전:

```text
A-zero        dx +0.058  dy +0.066  dz -1.361 mm
B-positive-1  dx -0.395  dy -0.130  dz -0.820 mm
C-mixed-1     dx +0.296  dy +0.229  dz -1.201 mm
E-fixed-1     dx +0.158  dy +0.071  dz -1.450 mm
E-fixed-3     dx +0.049  dy +0.067  dz -1.343 mm
```

방향은 configuration에 따라 바뀐다. 원인은 root orientation이 팔의 pose로 전달되는 Pattern B다. tool0 local axis에 고정된 오차는 아니다.

## Relocation 후

Articulation root는 `/ur10e/Physics/tn__base_linkbase_link_inertia_kc0`이다. relocated count는 1이다. 같은 actual joint의 tool0 cumulative position error는 A-zero post-settle 0.0004 mm, 다섯 configuration의 TCP runner 최대는 0.00075148570272387397 mm다.

A-zero forearm local orientation은 0.0214 deg이고 wrist_2 local orientation 0.0223 deg와 상쇄되어 tool0 orientation error는 0.0000 deg다. 이 값은 1.47 mm의 시작점이 아니다.

## 수정 후 cross-simulator

Python ↔ Isaac kinematic maximum은 0.00075148570272387397 mm, 3.9087544105696663e-05 deg다.

Unity ↔ Isaac maximum은 1.2526456740680483 mm다. 이 벡터는 FK(Unity actual joints)와 FK(Isaac actual joints)의 차이와 같다. A-zero shoulder_lift actual은 Unity 0.0002858018851839006 rad, Isaac 0.0012141603510826826 rad다. Drive stiffness, damping, timestep은 변경하지 않았다.

Acceptance의 Unity ↔ Isaac position < 1.0 mm는 만족하지 않는다. P-004는 FAIL이다.
