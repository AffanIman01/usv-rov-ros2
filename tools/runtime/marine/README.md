# Native calm-water physics checks

This harness loads the production pool SDF, creates actual Gazebo entities, and advances Gazebo's installed DART Physics system and the project's compiled `SurfaceBuoyancy` system. It does not create a Gazebo Server, a ROS graph, a transport subscriber or a rendering window.

The buoyancy plugin is configured through the actual `LoadSdfPlugins` event while `SdfEntityCreator` constructs the world. This reproduces Gazebo's startup order: model plugins are configured before the model is attached to its world. Each production case requires that ordering. An earlier harness configured buoyancy only after world creation and missed the startup defect that disabled flotation. The built-in buoyancy negative control remains a separate world plugin configured after its world exists.

The production `ComputeThrust` header supplies the two thrust forces. The harness reads all control limits, thruster positions and drag coefficients from the production model. It applies the same diagonal no-current damping equation as the configured Hydrodynamics system. **Hydrodynamics itself is not executed here:** its transport initialization is blocked by this host's network restrictions. This distinction is recorded in every result summary. No installed library or security restriction is modified.

From the repository root, after building `usv_rov_sim`:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S tools/runtime/marine -B /tmp/usv-pool-checks \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/usv-pool-checks -j2
python3 tools/runtime/marine/run_checks.py "$PWD" \
  --probe /tmp/usv-pool-checks/vehicle_probe \
  --plugin-dir "$PWD/.pool_ws/install/usv_rov_sim/lib" \
  --output /tmp/usv-pool-check-results
```

The tests check 60 seconds of unforced equilibrium; recovery from roll, pitch and heave perturbations; forward/reverse movement; both yaw directions; passive slowing after power cut; and restoring torque signs. Perturbation tests also assert that the initial disturbance actually occurred. Tests return nonzero on failure and preserve measured positions, angles and velocities.

Negative controls substitute the installed Harmonic graded Buoyancy plugin. In the tested Gazebo 8.15.0 installation, it produces a destabilizing pitch torque for the rotated hull boxes and the vessel capsizes. The project SurfaceBuoyancy implementation correctly rotates the water-plane normal into each collision frame and restores pitch. This control documents why the package does not use the problematic built-in surface buoyancy behavior. A later upstream version that corrects this behavior may cause the expected-failure negative controls to become unexpected passes; inspect those results before changing the expectations.

These are synthetic simulation checks using provisional mass, inertia, hull proxies and damping. They are not validation against ME120 measurements or proof of complete GUI/keyboard/bridge operation.

## Full ROS / Gazebo integration check

On a machine where ROS discovery and Gazebo transport are permitted, build and source both packages, then run:

```bash
source /opt/ros/jazzy/setup.bash
source .pool_ws/install/local_setup.bash
python3 tools/runtime/marine/ros_bridge_smoke.py \
  --log /tmp/usv-pool-launch.log --output /tmp/usv-pool-bridge-results.json
```

This separate check starts the actual headless server with the production buoyancy, Hydrodynamics, thruster and odometry plugins, plus `ros_gz_bridge`. It sends ROS velocity targets and observes ROS odometry and simulation time. It checks forward motion, the left-turn sign, explicit stop, command-stream timeout and flotation stability. It uses a separate ROS domain and Gazebo partition, has wall-clock timeouts and cleans up its own launch processes. The manual GitHub `Pool simulation` workflow is configured to run this check on Ubuntu 24.04 with ROS 2 Jazzy. The initial hosted attempt stopped before a runner was assigned; see `runtime_evidence/pool_simulation/github-ci.json`. It does not test the Gazebo GUI or a VMware graphics driver.
