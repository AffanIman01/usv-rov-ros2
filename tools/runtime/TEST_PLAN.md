# Actual ROS/Gazebo smoke tests

Prepared for parent execution after installation. These are not recorded successes.

1. Source `/opt/ros/jazzy/setup.bash` and use system Python, not a Python virtual environment.
2. Run colcon on `usv_rov_description` in a separate build/install/log directory. Source that install's setup file.
3. Run `check_urdf` on all seven URDF files and retain full logs.
4. Run `check_static_tf.py PACKAGE_SHARE --log-dir LOGS`. This launches actual `robot_state_publisher` through the supplied ROS launch file for each model; it compares every published fixed transform to the URDF and cleans up each launch process. It uses local discovery and separate ROS domains 71–77.
5. Export `GZ_SIM_RESOURCE_PATH` containing the package share parent and `PACKAGE_SHARE/models`, preserving any prior entries. Run `gz sdf -k` on all model SDF files and the preview world.
6. Run `/usr/bin/python3 build_mesh_expectations.py PACKAGE_SHARE/meshes mesh_expectations.tsv`, then build the native tests with `cmake -S native -B native/build` and `cmake --build native/build` after sourcing the installed Gazebo development environment. `check_gazebo_meshes mesh_expectations.tsv` exercises Gazebo's own mesh loader on all 39 DAE resources, requiring 129,400 triangles, nonempty materials, and every imported min/max bound matching the raw mesh-local DAE coordinates within 1 micrometre. This detects unintended scale or up-axis rotation. `check_gazebo_world PACKAGE_SHARE/worlds/model_preview.sdf` starts an actual headless server for 20 iterations and verifies all six named models exist.

`bash run_runtime_checks.sh PACKAGE_SOURCE RESULT_DIRECTORY` orchestrates these steps with system Python, isolated ROS domains, loopback Gazebo transport and full logs. It installs no software. See `results/runtime_results.json` for this environment's actual outcomes; server execution can be blocked by host network restrictions even when the package and imported meshes are valid.

Steps 5–6 require matching Harmonic development packages (`gz-common5`, graphics component; `gz-sim8`). If only CLI binaries are available, use `gz sim -s -r --iterations 20 WORLD` for a limited server startup check, but do not claim visual mesh import or all model entities were verified from process exit alone.

## Rendering is a separate test

A headless server can retain visual URIs without creating a renderer. The native mesh test verifies parsing and resource content, not pixels. If a display or software OpenGL/virtual display is available, launch RViz or the Gazebo GUI and inspect a captured frame plus their logs. If no renderer works, report render testing unavailable even when the runtime and mesh importer pass.

No test above validates hydrodynamics, collision response, sensors, docking forces or hardware control; the package implements none of those.
