# ME120 calm-water marine simulation

This first dynamics stage adds flotation, water resistance and smooth keyboard-controlled twin-thruster movement, a moving mast lift and navigation lights to the existing USV visuals. The default environment is the small 20 × 12 m, 2 m deep pool for initial VMware testing. Water is calm. The existing VRX Sydney Regatta terrain, buildings and docks remain optional with `--world sydney_regatta`. The ROV remains unchanged in the description package and is not deployed in this world.

These are explicitly **estimated simulation dynamics**, not calibrated ME120 performance. The model uses a 120 kg mass, a provisional centre of gravity and inertia, two box displacement proxies and diagonal drag. It does not include added-mass effects, waves, currents, real sensors, hardware drivers, tether forces or docking.

## Run on Ubuntu 24.04 / ROS 2 Jazzy

If dependencies are missing, install them once:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-sim-vendor \
  ros-jazzy-gz-plugin-vendor ros-jazzy-gz-transport-vendor ros-jazzy-gz-msgs-vendor \
  python3-colcon-common-extensions python3-tk build-essential cmake
```

From the repository root, start the simulation:

```bash
bash tools/run_sim.sh
```

In a second terminal, use the same checkout:

```bash
bash tools/run_sim.sh --keyboard
```

Keep the **ME120 control window** focused. Hold **W/S** for forward/reverse and **A/D** for left/right turn. **W+A** and **W+D** combine throttle and steering; releasing the steering key retains forward throttle. Speed and yaw targets ramp independently, including controlled reversals. Sliders adjust the target limits. **U/J** raise/lower the lift, including while driving; release to hold its measured position. **Space** cuts thrust and holds the lift immediately, and **Q/Esc** closes the controller. Switching windows also stops commands. Release held keys before resuming after Space or a control-timer interruption.

The panel publishes at 20 Hz, with estimated surge acceleration/deceleration of 0.5/0.8 m/s² and yaw acceleration/deceleration of 0.8/1.2 rad/s². Separate drive and lift watchdogs expire missing commands after 0.4 wall seconds. The vessel coasts under drag after thrust is cut. Do not run two command publishers concurrently.

`--keyboard-terminal` retains the older terminal fallback: it uses key repeat and 0.35 s expiry, cannot reliably track multiple held keys and does not control the lift. The new window requires the Ubuntu desktop session and `python3-tk`.

The pool requires no terrain download. On the first optional Sydney launch, the helper downloads the pinned 129 MB terrain archive and verifies its checksum and extracted files. Later launches use this cache offline. See [world sources and licenses](worlds/SOURCES.md).

The helper builds into `.pool_ws` inside this checkout and verifies the installed USV visual files. It does not delete or replace other workspaces. Stop the old simulation or RViz preview launch before starting. Stop this simulation with Ctrl+C in the simulation terminal. Gazebo's play/pause control pauses the world; reset clears the controller's saved command.

For VMware graphics issues, try `bash tools/run_sim.sh --software`. Software rendering can be slower. `--render-engine ogre` selects the alternative renderer; `--headless` runs only the physics server. `--check-only` builds, verifies USV visual assets and loads all three custom plugin classes without starting Gazebo. `--world pool` explicitly selects the default small offline test world; `--world sydney_regatta` selects the larger environment. `--self-test` starts a separate headless instance and checks ROS commands against measured movement; stop the interactive simulation first.

## ROS interface

| Topic | Type | Direction |
|---|---|---|
| `/sim/me120/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo velocity target |
| `/sim/me120/odometry` | `nav_msgs/msg/Odometry` | Gazebo → ROS ground-truth pose/velocity |
| `/sim/me120/lift/velocity` | `std_msgs/msg/Float64` | ROS → Gazebo: −1 raise, +1 lower, 0 hold |
| `/sim/me120/lift/position` | `std_msgs/msg/Float64` | Gazebo → ROS: actual downward joint travel, metres |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS simulation time |

The controller uses only `linear.x` (m/s) and `angular.z` (rad/s). All command fields must be finite. Targets are limited to 1.0 m/s and 0.6 rad/s. Each estimated thruster is limited to ±40 N, so targets are requests, not guaranteed achieved speeds. Physical forces and moments drive Gazebo's physics engine. Odometry is simulator ground truth, not a GNSS or IMU sensor model.

The Gazebo command topic is `/model/me120/cmd_vel`. The launch file sets a dedicated per-user Gazebo partition unless `GZ_PARTITION` is already set. It bridges simulation topics only. No network connection to the real USV or ROV is configured.

## Parameters and verification

`models/me120_dynamic/model.sdf` contains all prototype mass, inertia, hull proxies, damping coefficients, thruster positions, gains and limits. Its `SurfaceBuoyancy` plugin defines the water boundary at z = 0, matching the visual surface in both supplied worlds. The original 17 visual meshes retain their raised poses. Eight carrier/mast/rack visuals are now attached to a separate moving link; the remaining visuals stay fixed to the vessel. Tune physics parameters using measured vessel properties before using this simulation to predict real behaviour.

The package supplies a small box-based `SurfaceBuoyancy` system because the installed Harmonic graded-buoyancy implementation produced an overturning pitch moment during testing. The upstream [rotated-collision buoyancy issue](https://github.com/gazebosim/gz-sim/issues/3892) describes this failure. This package transforms the water plane into each box's frame before computing submerged volume and its centroid, preserving pitch and roll restoring forces. It clips each box geometrically against the water plane and sums tetrahedra to find displaced volume and its centre. Gazebo Physics integrates the resulting forces; the plugin does not set the boat's pose or lock roll/pitch. Do not enable a second buoyancy system on the same model.

## Mast lift and navigation lights

The reference is Recovery Studio version 60, source commit `893cd2a47913549e87ab3442d583f918d8934e36`. Its `dist/lift.mjs` defines a 0.418 m reference stroke and a 65% lower stop, giving **0.2717 m travel**. The model uses a real downward prismatic joint: q=0 is raised, q=0.2717 is fully lowered. The moving carrier, mounting ring, mast, ZED/RoboSense bracket and service loops move together. The ROV and E-Reel remain separate from this mast mechanism.

`LiftController` applies bounded joint force with gravity compensation and position/velocity feedback. It integrates a rate-limited target while U/J is held, then captures the measured position on release or command expiry. The speed limit is 0.1045 m/s, matching the Studio animation's 0.25 fraction/s; acceleration 0.25 m/s² and effort 150 N are provisional. Hard joint travel limits bound the mechanism. These are visualization/prototyping estimates, not measured actuator specifications.

Total vessel mass stays 120 kg: 115 kg base plus a provisional 5 kg moving assembly. Parallel-axis compensation preserves the previous raised centre of gravity (z=0.1 m) and diagonal inertia (35,70,90 kg·m²). Lowering shifts the combined centre of gravity downward naturally. Hull buoyancy proxies and thruster limits are unchanged.

Always-on emissive navigation lenses mark **port/left red (+Y)** and **starboard/right green (−Y)** when facing forward along +X. They follow the mast. Their appearance is a visual reference, without modeled electrical circuits or certified optical sectors. The static RViz description stays a fixed visual preview; use this Gazebo simulation for lift motion.

## Stability fixes and diagnostics

The original buoyancy implementation looked up its world during `Configure`. Gazebo emits that callback before setting the model's world parent, so the plugin permanently disabled itself during a real server startup. The implementation now resolves the world in `PreUpdate`, after the entity hierarchy is complete. The lifecycle regression test reproduces the server's `LoadSdfPlugins` event sequence; it failed before the correction and passes afterward. On the pool floor, contact friction can also prevent the limited thruster force from moving the sunken boat.

A later sustained wall-contact test reproduced a second failure: the analytic submerged-centroid formula became numerically unstable for an almost level hull, producing an impossible torque before the physics engine aborted. All partially submerged boxes now use bounded geometric clipping instead. Fully dry/wet boxes use exact volume and centre calculations. This changes the buoyancy calculation, not the boat pose or the collision engine.

The launch explicitly selects Gazebo 8 (Harmonic), which matches Jazzy's bridge and the built plugins. It checks the installed custom plugin filenames/classes before starting, then a passive ROS monitor reports advancing time and stable odometry or explains missing feedback/sinking. This monitor does not command movement and does not prove thruster delivery. Its height check is relative to the first odometry sample. If it reports a fault and exits, Gazebo now stays open for inspection; monitoring has stopped, so stop the launch with Ctrl+C when finished. Earlier versions shut down the whole simulation on monitor failure, which could itself cause the later stale odometry display. Actual simulator/bridge exits still trigger coordinated shutdown and report the component and exit code.

The control window distinguishes command targets from measured speed. `ROS subscribers: 0` means the command bridge has not connected; stale or missing odometry means the simulation feedback is unavailable. Both terminals must use the same `ROS_DOMAIN_ID`, printed at startup. Holding W must change the target shown in the control window; switching focus to Gazebo stops this controller.

Each launch keeps a timestamped log in `.pool_ws/runs/` and also writes `.pool_ws/last-launch.log`. The old last-launch log is preserved before a restart. `bash tools/run_sim.sh --diagnose` reads the last log, shows observed failure markers and its final 100 lines, and does not start Gazebo or send commands. Include this output when reporting sinking, a Ruby crash dialog or stale feedback; the stale status alone cannot distinguish these causes. The automatic `--self-test` saves `.pool_ws/last-test.json` and `.pool_ws/last-test.log`; it sends simulation commands in a separate ROS domain/Gazebo partition and returns failure if movement or feedback checks fail.

See `runtime_evidence/controls_lift_lights/REPORT.md` at the repository root for the latest controls/lift/light tests and remaining limitations. `runtime_evidence/pool_stability_followup/REPORT.md` records the preceding stability tests. The earlier `runtime_evidence/sydney_startup_fix/REPORT.md` records the startup-order correction. The previous pool report describes historical tests that did not reproduce the server's plugin-configuration order. The original static display commands remain available through `usv_rov_description`.
