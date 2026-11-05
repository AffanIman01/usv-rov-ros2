# Initial runtime test results (historical baseline)

6 November 2026 — ROS 2 Jazzy / Gazebo Harmonic libraries.

These records describe the initial 129,400-triangle export. They remain here for provenance. The later ME120 visibility correction, current render and mesh checks are documented in `me120_visibility_fix/REPORT.md`.

| Test | Result | Evidence |
|---|---|---|
| colcon build/install | Passed | One ROS package built and installed. |
| Native URDF parser | Passed | All seven URDF descriptions parsed. |
| Actual ROS launch and static TF | Passed | Seven launches, 84 expected transforms received and compared with URDF values. |
| Native SDFormat validation | Passed | Six model files and preview world valid; standalone validator required SDF_PATH configuration. |
| Native Gazebo mesh loader | Passed | 39 COLLADA meshes, 129,400 triangles, materials present; imported bounds matched exported coordinates within 1 micrometre. This is import consistency, not hardware dimensional accuracy. |
| Headless Gazebo server | Environment blocked | Network-interface enumeration failed; documented loopback configuration then encountered a denied ZeroMQ address-resolution operation. No model entities or simulation iterations were reached. |
| Gazebo Ogre2 offscreen rendering | Passed | Complete ME120 assembly: 17 parts at manifest offsets, 68,992 source triangles, 960 × 720 RGB capture. See `ME120_GAZEBO_RENDER.png`. |
| RViz GUI | Environment blocked | No desktop display; Xvfb could not create local/unix listening sockets. No RViz window was inspected. |

No geometry, URDF, SDF, launch or build files were changed for these checks. Documentation was updated after testing. Test harness corrections were confined to native API compatibility and resource-path environment configuration. No runtime security settings were changed.

The actual rendered image uses Gazebo Rendering 8.2.3 / Ogre2 with headless EGL (`EGL_MESA_device_software`), OpenGL 4.5 Core Profile, Mesa 25.2.8 and llvmpipe (LLVM 20.1.2, 256 bits). It ran without a display server or physical GPU and exited with code 0. The image was visually inspected. This independent renderer test does not imply that the Gazebo simulation server or camera/lidar sensor plugins ran.

## Installation scope

Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic (gz-sim 8.15.0). Core ROS, robot-state-publisher, RViz and Gazebo components configured successfully and the tested commands executed. Exact package versions are recorded in `installed_versions.tsv`.

The overall APT install returned code 100: `libibverbs1` could not create its RDMA group because the container denied access to the audit interface. Some optional image/demo dependency packages remain unconfigured. The tests above used the configured core components. No restriction was bypassed; the installation should not be described as a fully clean desktop setup.

Logs and machine-readable results are in this directory. These tests do not validate marine physics, collision response, sensor output, docking or hardware control.
