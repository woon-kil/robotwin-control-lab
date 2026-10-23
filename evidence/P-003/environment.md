# P-003 environment

```text
OS
Ubuntu 24.04.4 LTS

Kernel
Linux-6.8.0-147-generic-x86_64-with-glibc2.39

GPU
NVIDIA GeForce RTX 3090, 595.99.02, 24576 MiB

Python
3.12.14 (tags/v3.12.14-dirty:2abcf90, Sep  4 2026, 14:51:53) [GCC 11.4.0]

Isaac Sim version
6.2.0-rc.14+release.52805.02189a84.gl

Isaac root
/home/woonkil/isaacsim

Execution mode
headless

Physics device
cpu

Stage up axis
Z

Physics timestep
0.016666666666666666

Gravity
{"scene_path": "/PhysicsScene", "created_by_runner": true, "direction_xyz": [0.0, 0.0, -1.0], "magnitude": 9.81}

Solver position iterations


Solver velocity iterations


Solver iteration source
physxArticulation:solverPositionIterationCount is not authored

Source URDF
robot_description/generated/7ce98d8c3512b9d0239da93b75de68ea64f61bd6/ur10e.urdf

Source commit
7ce98d8c3512b9d0239da93b75de68ea64f61bd6

PROJECT TEST PARAMETER
drive stiffness Nm/rad
100000
drive damping Nm*s/rad
1000
stiffness readback
[100000.0, 100000.0, 100000.0, 100000.0, 100000.0, 100000.0]
damping readback
[1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]
max effort readback
[330.0, 330.0, 150.0, 54.0, 54.0, 54.0]
settled steps
20
velocity epsilon rad/s
0.001
position delta epsilon rad
1.0000000000000001e-05
timeout seconds
8

Initial condition
teleport dof positions to the commanded configuration and zero dof velocities, then set position targets
```

Drive stiffness와 damping은 URDFImporterConfig에 Nm/rad, Nm*s/rad로 적힌 PROJECT TEST PARAMETER다. UR10e controller capability가 아니다.
USD drive:angular:physics:stiffness와 damping은 import_report.json의 joint_mapping에 있다. drive_stiffness_readback은 Articulation.get_dof_gains 값이다.
solver iteration attribute가 stage에 없으면 스키마 기본값을 사용한 것처럼 적지 않는다.
Quaternion은 RigidPrim.get_world_poses 문서의 wxyz를 사용한다. tensor 원본 xyzw를 그대로 적지 않는다.
tool0 frame은 isaac_world다. canonical base_link로 변환하지 않았다.
