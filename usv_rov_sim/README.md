# ME120 calm-water marine simulation

This first dynamics stage adds flotation, water resistance and keyboard-controlled twin-thruster movement to the existing USV visuals. The default environment adapts the existing VRX Sydney Regatta world, with its OpenRobotics terrain, buildings and docks. Water is calm. The smaller 20 × 12 m, 2 m deep pool remains an optional test environment. The ROV remains unchanged in the description package and is not deployed in this world.

These are explicitly **estimated simulation dynamics**, not calibrated ME120 performance. The model uses a 120 kg mass, a provisional centre of gravity and inertia, two box displacement proxies and diagonal drag. It does not include added-mass effects, waves, currents, real sensors, hardware drivers, tether forces or docking.

## Run on Ubuntu 24.04 / ROS 2 Jazzy

If dependencies are missing, install them once:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-sim-vendor \
  ros-jazzy-gz-plugin-vendor ros-jazzy-gz-transport-vendor ros-jazzy-gz-msgs-vendor \
  python3-colcon-common-extensions build-essential cmake
```

From the repository root, start the simulation:

```bash
bash tools/run_sim.sh
```

In a second terminal, use the same checkout:

```bash
bash tools/run_sim.sh --keyboard
```

Keep the keyboard terminal focused. Hold or repeat **W/S** for forward/reverse and **A/D** for left/right turn. **Space** cuts thrust, **+/-** adjusts target speed, and **Q** exits the keyboard node. Release also expires the motion command after 0.35 seconds. A separate controller watchdog cuts thrust if commands disappear for 0.4 wall seconds. The vessel coasts under drag after thrust is cut; it is not teleported to a stop.

On the first Sydney launch, the helper downloads the pinned 129 MB terrain archive and verifies its checksum and extracted files. Later launches use this cache offline. See [world sources and licenses](worlds/SOURCES.md).

The helper builds into `.pool_ws` inside this checkout and verifies the installed USV visual files. It does not delete or replace other workspaces. Stop the old simulation or RViz preview launch before starting. Stop this simulation with Ctrl+C in the simulation terminal. Gazebo's play/pause control pauses the world; reset clears the controller's saved command.

For VMware graphics issues, try `bash tools/run_sim.sh --software`. Software rendering can be slower. `--render-engine ogre` selects the alternative renderer; `--headless` runs only the physics server. `--check-only` builds, verifies USV visual assets and loads both custom plugin classes without starting Gazebo. `--world pool` selects the smaller offline test world. `--self-test` starts a separate headless instance and checks ROS commands against measured movement; stop the interactive simulation first.

## ROS interface

| Topic | Type | Direction |
|---|---|---|
| `/sim/me120/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo velocity target |
| `/sim/me120/odometry` | `nav_msgs/msg/Odometry` | Gazebo → ROS ground-truth pose/velocity |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS simulation time |

The controller uses only `linear.x` (m/s) and `angular.z` (rad/s). All command fields must be finite. Targets are limited to 1.0 m/s and 0.6 rad/s. Each estimated thruster is limited to ±40 N, so targets are requests, not guaranteed achieved speeds. Physical forces and moments drive Gazebo's physics engine. Odometry is simulator ground truth, not a GNSS or IMU sensor model.

The Gazebo command topic is `/model/me120/cmd_vel`. The launch file sets a dedicated per-user Gazebo partition unless `GZ_PARTITION` is already set. It bridges simulation topics only. No network connection to the real USV or ROV is configured.

## Parameters and verification

`models/me120_dynamic/model.sdf` contains all prototype mass, inertia, hull proxies, damping coefficients, thruster positions, gains and limits. Its `SurfaceBuoyancy` plugin defines the water boundary at z = 0, matching the visual surface in both supplied worlds. The original 17 visual meshes and their relative poses are reused without changes. Tune physics parameters using measured vessel properties before using this simulation to predict real behaviour.

The package supplies a small box-based `SurfaceBuoyancy` system because the installed Harmonic graded-buoyancy implementation produced an overturning pitch moment during testing. The upstream [rotated-collision buoyancy issue](https://github.com/gazebosim/gz-sim/issues/3892) describes this failure. This package transforms the water plane into each box's frame before computing submerged volume and its centroid, preserving pitch and roll restoring forces. It uses Gazebo Math's clipping functions and Gazebo Physics for integration; it does not set the boat's pose or lock roll/pitch. Do not enable a second buoyancy system on the same model.

## Startup fix and diagnostics

The original buoyancy implementation looked up its world during `Configure`. Gazebo emits that callback before setting the model's world parent, so the plugin permanently disabled itself during a real server startup. The implementation now resolves the world in `PreUpdate`, after the entity hierarchy is complete. The lifecycle regression test reproduces the server's `LoadSdfPlugins` event sequence; it failed before the correction and passes afterward. On the pool floor, contact friction can also prevent the limited thruster force from moving the sunken boat.

The launch explicitly selects Gazebo 8 (Harmonic), which matches Jazzy's bridge and the built plugins. It checks the installed custom plugin filenames/classes before starting, then a passive ROS monitor reports advancing time and stable odometry or explains missing feedback/sinking. This monitor does not command movement and does not prove thruster delivery. Its height check is relative to the first odometry sample.

The keyboard display distinguishes command targets from measured speed. `ROS subscribers: 0` means the command bridge has not connected; stale or missing odometry means the simulation feedback is unavailable. Both terminals must use the same `ROS_DOMAIN_ID`, printed at startup. Holding W must change the target shown in the keyboard terminal; clicking inside Gazebo instead sends keys to Gazebo.

Launch logs are saved in `.pool_ws/last-launch.log`. The automatic `--self-test` saves `.pool_ws/last-test.json` and `.pool_ws/last-test.log`; it sends simulation commands in a separate ROS domain/Gazebo partition and returns failure if movement or feedback checks fail.

See the repository root `runtime_evidence/sydney_startup_fix/REPORT.md` for current tests and remaining limitations. The previous pool report describes historical tests that did not reproduce the server's plugin-configuration order. The original static display commands remain available through `usv_rov_description`.
