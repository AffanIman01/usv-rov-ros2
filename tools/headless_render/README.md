# Actual Gazebo offscreen rendering

The commands below also work for the corrected model. Latest renders and results: `../../runtime_evidence/me120_visibility_fix/REPORT.md`. The initial capture described below is retained as historical evidence.

The native Gazebo Rendering 8.2.3 Ogre2 backend rendered the complete exported ME120 assembly: 17 COLLADA meshes, 68,992 source triangles, and exact per-part offsets from `model_manifest.json`. The process exited with code 0. The 960 × 720 RGB result is in `../../runtime_evidence/ME120_GAZEBO_RENDER.png`.

The backend was headless EGL (`EGL_MESA_device_software`), OpenGL 4.5 Core Profile, Mesa 25.2.8, and llvmpipe (LLVM 20.1.2, 256 bits). No display server or physical GPU was used. This verifies visual import and rendering only, independent of the Gazebo simulation server. It does not test sensor plugins, ROS image topics, physics, or RViz.

## Reproduce

Use ROS 2 Jazzy with its Gazebo rendering/common vendor development packages and Mesa EGL. From this directory, using an extracted path without whitespace:

```bash
source /opt/ros/jazzy/setup.bash
python3 make_parts.py ../../usv_rov_description me120-parts-local.tsv
cmake -S . -B build
cmake --build build -j2
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless GZ_IP=127.0.0.1 \
  timeout 45 ./build/headless_probe me120-parts-local.tsv me120-local.png
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless GZ_IP=127.0.0.1 \
  timeout 45 ./build/headless_probe me120-parts-local.tsv me120-rear.png rear
```

`me120-parts.tsv` preserves the actual input paths from the original test workspace; regenerate the local TSV with the command above. `probe.cpp` creates the Gazebo scene directly, imports each mesh, applies its offset, adds light and a camera, and captures one frame. It tears down scene objects before unloading the renderer. The capture is a real Gazebo rendering-engine image, not a screenshot of the Gazebo GUI.

Evidence logs: `../../runtime_evidence/logs/render.log` and `../../runtime_evidence/logs/ogre2.log`.
