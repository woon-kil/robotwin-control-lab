# P-003 Isaac UR10e joint control

```text
Gate
P-003

Result
PASS

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
0.0015286624431610218 rad
0.087585906293283652 deg

Engineering target
< 0.1 deg

Timeouts
0

Tool0 observation
UsdGeom.XformCache.GetLocalToWorldTransform Gf.Quatd real then imaginary

Tool0 base distance max
1.3166888748578596 m

Cumulative world drift
False

Fixed tool0
A-zero 1.1840670108795746 0.29081696210666502 0.05750801414253482
B-positive-1 0.86082869771210258 0.45060029699989362 -0.47541126654796623
C-mixed-1 0.95159471031483123 0.82784497693584203 0.37794789664443013
E-fixed-1 1.2241501799973511 0.40062919054793938 0.14369448988255593
E-fixed-3 1.1715254784921376 0.30238359951834159 0.051244363185259822

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

```text
Initial P-003
Joint error:
0.08758590629328365 deg PASS

BUT

tool0:
physically invalid cumulative world displacement
A-zero z=0.057508014142513275
B-positive-1 z=-2.5203418731689453
C-mixed-1 z=-10.136713981628418
E-fixed-1 z=-22.79160499572754
E-fixed-3 z=-40.485015869140625

Overall:
FAIL

Fix Attempt 1
Root cause:
tool0 is an Xform with no RigidBodyAPI. RigidPrim.__init__ applies RigidBodyAPI, and that new body falls under gravity.

Modification:
Read tool0 with UsdGeom.XformCache. Do not construct RigidPrim on a prim that has no RigidBodyAPI.

Joint error:
0.087585906293283652 deg

Tool0 integrity:
PASS

Overall:
PASS
```
