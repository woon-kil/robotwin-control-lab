# P-004 Fix Attempt 2

```text
Baseline Unity ↔ Isaac
1.2526456740680483 mm

Cause
Settled shoulder_lift_joint difference under gravity.
A-zero Unity - Isaac = -0.000928358465898782 rad = -0.05319102197123997 deg.
FK(q_unity) and FK(q_isaac) reproduce the simulator TCP difference within 0.003836883818283858 mm.

Parameter
Isaac URDFImporterConfig override_joint_stiffness
Before: 100000 Nm/rad
After: 200000 Nm/rad
Damping stayed 1000.
Reason: gravity compensation 121.28190612792969 Nm equals K*q at 100000, and gravity OFF removes the joint error. The 330 Nm effort limit is not reached. Solver iteration count is not authored and could not be set on the root joint.
Measured effect: A-zero shoulder_lift error 0.0012141603510826826 rad → 0.0005065396544523537 rad. Five-configuration Unity ↔ Isaac position 0.29056228373221366 mm.
Side effect: damping ratio is lower because damping was not raised. All five configurations and both reruns settled with timeout false and finite joint values.

Result
PASS

Python ↔ Unity
0.0040523245387939092 mm
0.00062471732598054931 deg

Python ↔ Isaac
0.00056994037914351162 mm
5.5449295521625546e-05 deg

Unity ↔ Isaac
0.29056228373221366 mm
0.019650595706166018 deg

Unity max joint error
0.035251026523152325 deg

Isaac max joint error
0.029022584356134966 deg

Timeouts
0

Deterministic rerun
PASS

Empirical TCP offset
NO

Tolerance changed
NO

P-005 started
NO
```
