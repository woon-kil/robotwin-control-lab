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


Physics timestep


Gravity
{}

Solver position iterations


Solver velocity iterations


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
[]
damping readback
[]
max effort readback
[]
settled steps
20
velocity epsilon rad/s
0.001
position delta epsilon rad
1.0000000000000001e-05
timeout seconds
8
```

Drive stiffness와 damping은 URDFImporterConfig에 Nm/rad, Nm*s/rad로 적힌 PROJECT TEST PARAMETER다. UR10e controller capability가 아니다.
Quaternion은 RigidPrim.get_world_poses 문서의 wxyz를 사용한다. tensor 원본 xyzw를 그대로 적지 않는다.
tool0 frame은 isaac_world다. canonical base_link로 변환하지 않았다.
