# Verification record

6 November 2026

- Source: Recovery Studio Site version 58, commit `9edda0a7a7fcd8700558ed00c44bbd2e29d9196e`.
- 39 COLLADA files: 129,400 triangles. Finite coordinates, valid indices, valid material bindings, metre units and Z-up metadata verified.
- 7 URDF files: valid fixed joint trees with a single `base_link` root, unique link names and resolvable package mesh references.
- 7 SDF files: 6 static model definitions and 1 preview world. Local resource references resolved structurally.
- 6 model manifest entries: bounds and triangle counts match exported geometry.
- Both coordinate conversion matrices have determinant +1.
- Launch Python syntax and package XML/RViz YAML parsing passed.
- All five input GLBs contain triangle geometry and normals, with no texture images or vertex colors.
- Exported COLLADA geometry was independently parsed and rendered to `MODEL_PREVIEW.png` for visual inspection. That image is not a Gazebo render.

## Actual ROS 2 / Gazebo verification

ROS 2 Jazzy and Gazebo Harmonic were installed after the initial structural checks. Results:

- `colcon build` and package installation passed.
- Native `check_urdf` parsed all seven descriptions.
- Seven actual ROS display launches (RViz disabled) published all 84 expected static transforms; translations and rotations matched the URDFs.
- Native `gz sdf -k` validated six models and the preview world.
- Gazebo's native mesh importer loaded all 39 COLLADA files, with 129,400 triangles and materials. Imported bounds agreed with the exported file coordinates within 1e-6 metres. This checks import consistency, not physical measurement accuracy.
- Gazebo Rendering 8.2.3 / Ogre2 rendered the complete 17-part ME120 assembly to `runtime_evidence/ME120_GAZEBO_RENDER.png` using headless EGL and Mesa llvmpipe.

See `runtime_evidence/RUNTIME_TEST_REPORT.md`, `runtime_evidence/runtime_results.json` and the bundled logs. Native probe source and reproducible checks are under `tools/runtime/` and `tools/headless_render/`.

## Environment limits and untested behavior

The Gazebo simulation server could not reach model creation or time stepping because network-interface enumeration and a ZeroMQ address-resolution operation were denied. RViz's GUI could not be tested because the virtual display server could not create listening sockets. These are environment blockers, not passing simulation or GUI tests.

Core ROS, RViz and Gazebo packages configured successfully. The overall APT transaction reported a failure in an optional RDMA dependency's group-creation step, leaving some image/demo dependencies unconfigured. No security settings were changed to work around this. The tested core components ran despite that limitation.

This release contains no physics or sensor-stream implementation. Buoyancy, thrusters, tether, docking forces, real device drivers and RoboSense/Hikvision/ZED sensor output have not been implemented or tested.

Re-run structural checks with:

```bash
python3 tools/validate_models.py
```
