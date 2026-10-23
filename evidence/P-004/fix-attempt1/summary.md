# P-004 Fix Attempt 1

```text
Initial P-004
Result FAIL

Python ↔ Unity:
0.004052 mm

Python ↔ Isaac:
1.470560 mm

Unity ↔ Isaac:
3.154899 mm

Stored initial maxima
Python ↔ Unity position 0.0040523245387939092 mm
Python ↔ Unity orientation 0.00062471732598054931 deg
Python ↔ Isaac position 1.4705603411282602 mm
Python ↔ Isaac orientation 0.066672609317108195 deg
Unity ↔ Isaac position 3.1548989314166063 mm
Unity ↔ Isaac orientation 0.14790147537513967 deg

Root cause:
ArticulationRootAPI stayed on the rigid body base_link_inertia.
URDFImporter apply_fix_base saw the existing fixed joint from non-rigid
base_link to base_link_inertia and skipped relocation. Authored USD local
transforms matched Python FK at q=0. After the first physics step the
base_link_inertia orientation differed by about 0.065 deg, and that rotation
was levered along the arm. The same frame led on A-zero, B-positive-1,
C-mixed-1, E-fixed-1, and E-fixed-3, for pre-step, post-settle, and
injected settled joints.

First divergent frame:
base_link_inertia

Fix:
isaacsim.asset.importer.utils.impl.asset_utils.fix_articulation_root_for_fixed_base
moves ArticulationRootAPI onto the world-anchoring fixed joint
/ur10e/Physics/tn__base_linkbase_link_inertia_kc0.
Python FK parameters, URDF geometry, drives, timestep, and tolerances
were not changed.

Fixed measurement
Result FAIL

Python ↔ Unity position 0.0040523245387939092 mm
Python ↔ Unity orientation 0.00062471732598054931 deg
Python ↔ Isaac position 0.00075148570272387397 mm
Python ↔ Isaac orientation 3.9087544105696663e-05 deg
Unity ↔ Isaac position 1.2526456740680483 mm
Unity ↔ Isaac orientation 0.060103776762272014 deg

Unity ↔ Isaac still exceeds 1.0 mm. That remainder matches
FK(Unity actual joints) versus FK(Isaac actual joints).
The largest settled joint difference is shoulder_lift_joint,
about 0.00093 rad on A-zero. Drive stiffness and damping were not changed.

A-zero fresh rerun
Unity joint difference 0 rad
Unity tool0 position difference 0 m
Unity tool0 orientation difference 0 deg
Isaac joint difference 0 rad
Isaac tool0 position difference 0 m
Isaac tool0 orientation difference 0 deg

Empirical offset used: NO
Tolerance changed: NO
Regression: PASS
P-005 started: NO
```
