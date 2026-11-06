# Small-pool stability and crash follow-up

6 November 2026. Previous public version: `4d639e96fb81dc548be65d429568b2cfddecf57f`. Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.15.0 and Gazebo Math 7.7.0.

The user reported sudden sinking, stale odometry and a Ruby/Gazebo app close. Their simulator log was not available. Investigation reproduced a real physics crash and also identified an intentional shutdown path in the launch file. These are confirmed defects; the user's exact app-close mechanism still requires their log.

## Reproduced physics failure

With the previous plugin, sustained forward thrust into the small pool wall produced an impossible buoyancy torque at 26.674 simulated seconds. The hull had almost zero roll. The analytic submerged-centroid calculation divided by tiny plane-normal components and suffered catastrophic cancellation. It returned a finite but physically impossible centre, resulting in about 2.26 × 10^18 N·m of roll torque. The following physics step generated enormous coordinates, and the collision library aborted on an AABB assertion. [Original failure excerpt](wall-before.log).

The buoyancy implementation now clips the six box faces against the water plane, closes the cut face, and calculates volume and centre by summing positive tetrahedra around an interior point. Intersections stay inside the box. Fully dry/wet boxes use exact results. Centroid and volume bounds are checked. This changes the numerical geometry calculation; it does not lock the vessel pose, clamp its applied physical forces, or replace the collision engine.

For the identical recorded pose:

| Measurement | Previous implementation | Geometric clipping |
| --- | ---: | ---: |
| Upward force | 3,021.48 N | 3,021.48 N |
| Roll torque | 2.259 × 10^18 N·m | 0 N·m |
| Pitch torque | −991.25 N·m | −8.42 N·m |

[Captured-pose comparison](centroid-regression.json). Regression checks also cover near-axis perturbations, conservation of volume and first moment, exact half-volume cuts and continuity as roll crosses zero. An independent code review found no blocking geometry issue.

## Default world and fault visibility

Both launch helpers and `sim.launch.py` now default to the small 20 × 12 m pool. It needs no terrain download. Sydney Regatta remains optional with `--world sydney_regatta`.

Previously, any monitor exit requested shutdown of Gazebo and the bridge. A detected sinking fault could therefore close the application and cause later keyboard feedback to become stale. A monitor fault now reports its error and leaves Gazebo open for inspection; passive monitoring has stopped at that point. Actual simulator/bridge exits still stop the other processes and print the component and exit code. Normal Ctrl+C teardown does not produce a new crash diagnosis.

Every launch saves a separate log under `.pool_ws/runs/`. The preceding `.pool_ws/last-launch.log` is preserved before a new run overwrites it. `bash tools/run_sim.sh --diagnose` reads the latest log and reports observed markers without restarting or commanding the boat. Stale odometry alone cannot distinguish a paused world, lost feedback or a crash.

## Verification

| Check | Result |
| --- | --- |
| Colcon build and installed meshes | Pass; both packages build, USV URDF and 17 visual meshes match source. |
| CTest | All 8 suites pass, including buoyancy geometry, real startup ordering and launch exit policy. [Log](unit-tests.log). |
| Native physics | All 16 expectations pass. [Results and input hashes](native/results.json). |
| Continuous wall contact, 180 simulated seconds | No crash; body-origin z stays 0.152785–0.154927 m. Maximum buoyancy torque 21.31 N·m. |
| Mixed forward/reverse/turn commands, 180 simulated seconds | No crash; body-origin z stays 0.152240–0.158414 m. Pose, velocity and wrench bounds hold at every step. |
| Log preservation and default selection | Verified pool selection without terrain download; previous log survives unchanged and a separate new run log is saved. |
| Full headless ROS/Gazebo launch | Preflight passes; this host still blocks network discovery (`getifaddrs: Operation not permitted`). Server/bridge exit255, so this is not a live integration pass. [Log](full-launch.log), [result](launch-result.json). |
| VMware GUI / actual ROS keyboard command delivery | Not tested here. |

The native harness runs the installed DART Physics system and production buoyancy library, including real plugin startup ordering. It applies the production thrust calculation and configured drag equation directly; it does not run the controller transport callback or built-in Hydrodynamics plugin. No host restrictions or system libraries were bypassed or modified. Mass, inertia, damping and thruster limits remain simulation estimates.
