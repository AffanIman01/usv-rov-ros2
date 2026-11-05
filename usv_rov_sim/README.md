# ME120 calm-water pool prototype

This first dynamics stage adds flotation, water resistance and keyboard-controlled twin-thruster movement to the existing USV visuals. The pool is 20 × 12 m and 2 m deep. Water is calm. The ROV remains unchanged in the description package and is not deployed in this world.

These are explicitly **estimated simulation dynamics**, not calibrated ME120 performance. The model uses a 120 kg mass, a provisional centre of gravity and inertia, two box displacement proxies and diagonal drag. It does not include added-mass effects, waves, currents, real sensors, hardware drivers, tether forces or docking.

## Run on Ubuntu 24.04 / ROS 2 Jazzy

If dependencies are missing, install them once:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-sim-vendor \
  ros-jazzy-gz-plugin-vendor ros-jazzy-gz-transport-vendor ros-jazzy-gz-msgs-vendor \
  python3-colcon-common-extensions build-essential cmake
```

From the repository root, start the pool:

```bash
bash tools/run_pool.sh
```

In a second terminal, use the same checkout:

```bash
bash tools/run_pool.sh --keyboard
```

Keep the keyboard terminal focused. Hold or repeat **W/S** for forward/reverse and **A/D** for left/right turn. **Space** cuts thrust, **+/-** adjusts target speed, and **Q** exits the keyboard node. Release also expires the motion command after 0.35 seconds. A separate controller watchdog cuts thrust if commands disappear for 0.4 wall seconds. The vessel coasts under drag after thrust is cut; it is not teleported to a stop.

The helper builds into `.pool_ws` inside this checkout and verifies the installed USV visual files. It does not delete or replace other workspaces. Stop the old RViz preview launch before using the pool. Stop this simulation with Ctrl+C in the pool terminal. Gazebo's play/pause control pauses the world; reset clears the controller's saved command.

For VMware graphics issues, try `bash tools/run_pool.sh --software`. Software rendering can be slower. `--render-engine ogre` selects the alternative renderer; `--headless` runs only the physics server. `--check-only` builds and verifies assets without starting Gazebo.

## ROS interface

| Topic | Type | Direction |
|---|---|---|
| `/sim/me120/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo velocity target |
| `/sim/me120/odometry` | `nav_msgs/msg/Odometry` | Gazebo → ROS ground-truth pose/velocity |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS simulation time |

The controller uses only `linear.x` (m/s) and `angular.z` (rad/s). All command fields must be finite. Targets are limited to 1.0 m/s and 0.6 rad/s. Each estimated thruster is limited to ±40 N, so targets are requests, not guaranteed achieved speeds. Physical forces and moments drive Gazebo's physics engine. Odometry is simulator ground truth, not a GNSS or IMU sensor model.

The Gazebo command topic is `/model/me120/cmd_vel`. The launch file sets a dedicated per-user Gazebo partition unless `GZ_PARTITION` is already set. It bridges simulation topics only. No network connection to the real USV or ROV is configured.

## Parameters and verification

`models/me120_dynamic/model.sdf` contains all prototype mass, inertia, hull proxies, damping coefficients, thruster positions, gains and limits. Its `SurfaceBuoyancy` plugin defines the water boundary at z = 0, matching the visual surface in `worlds/pool.sdf`. The original 17 visual meshes and their relative poses are reused without changes. Tune physics parameters using measured vessel properties before using this simulation to predict real behaviour.

The package supplies a small box-based `SurfaceBuoyancy` system because the installed Harmonic graded-buoyancy implementation produced an overturning pitch moment during testing. The upstream [rotated-collision buoyancy issue](https://github.com/gazebosim/gz-sim/issues/3892) describes this failure. This package transforms the water plane into each box's frame before computing submerged volume and its centroid, preserving pitch and roll restoring forces. It uses Gazebo Math's clipping functions and Gazebo Physics for integration; it does not set the boat's pose or lock roll/pitch. Do not enable a second buoyancy system on the same model.

See the repository root `runtime_evidence/pool_simulation/REPORT.md` for actual tests and remaining limitations. The original static display commands remain available through `usv_rov_description`.
