# P-005 environment

claim: UR10e-based reference workcell
geometry_class: PROJECT TEST GEOMETRY
isaac_version: 6.2.0-rc.14+release.52805.02189a84.gl
execution_mode: headless
physics_device: cpu
os: Linux-6.8.0-147-generic-x86_64-with-glibc2.39
gpu: NVIDIA GeForce RTX 3090, 595.99.02, 24576 MiB
physics_dt_s: 0.016666666666666666
gravity: {'prim': '/PhysicsScene', 'direction': [0.0, 0.0, -1.0], 'magnitude_m_s2': 9.8100004196167, 'enabled': True}
drive_stiffness_nm_per_rad: 200000.0
drive_damping: 1000.0
articulation_root: /ur10e/Physics/tn__base_linkbase_link_inertia_kc0
placement_method: usd_translate_on_/ur10e
up_axis: Z
handedness: right
tool0_observation_api: UsdGeom.XformCache.GetLocalToWorldTransform Gf.Quatd real then imaginary
contact_api: omni.physx.get_physx_simulation_interface().get_contact_report
raycast_api: omni.physx.get_physx_scene_query_interface().raycast_closest
self_collision_engine: not implemented
unity_workcell: not duplicated in this gate. Isaac is the physics baseline.
