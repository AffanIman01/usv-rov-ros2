# Native pool rendering check

This probe renders the pool's five boxes, water plane, 17 ME120 meshes and four
navigation-light lens panels through Gazebo Rendering 8's Ogre2 backend. It uses
the visual poses, dimensions, colors, emissive materials, transparency and initial
boat placement extracted from the checked-in SDFs. The
start-view camera and lighting match the initial pool world; an `overview`
argument selects a more distant camera. `detail` frames the full boat more
closely, and `mast` frames the sensor rack and navigation lamps from the bow.

This is an offscreen image check at the initial poses. It does not start the
Gazebo server, execute dynamics, open the GUI or exercise ROS topics.

From the repository root on a ROS 2 Jazzy installation with Gazebo Rendering
and Common development packages plus Mesa EGL:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/runtime/pool_render/make_scene.py \
  usv_rov_description usv_rov_sim /tmp/usv-pool-scene.tsv
cmake -S tools/runtime/pool_render -B /tmp/usv-pool-render
cmake --build /tmp/usv-pool-render -j2
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless GZ_IP=127.0.0.1 \
  /tmp/usv-pool-render/headless_probe /tmp/usv-pool-scene.tsv /tmp/usv-pool-start.png
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless GZ_IP=127.0.0.1 \
  /tmp/usv-pool-render/headless_probe /tmp/usv-pool-scene.tsv /tmp/usv-pool-overview.png overview

# Static joint endpoint comparison, using the model's physical joint axis.
python3 tools/runtime/pool_render/make_scene.py \
  usv_rov_description usv_rov_sim /tmp/usv-pool-lowered.tsv --lift-position 0.2717
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless GZ_IP=127.0.0.1 \
  /tmp/usv-pool-render/headless_probe /tmp/usv-pool-scene.tsv /tmp/usv-raised.png detail
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless GZ_IP=127.0.0.1 \
  /tmp/usv-pool-render/headless_probe /tmp/usv-pool-lowered.tsv /tmp/usv-lowered.png detail
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless GZ_IP=127.0.0.1 \
  /tmp/usv-pool-render/headless_probe /tmp/usv-pool-scene.tsv /tmp/usv-navigation-lights.png mast
```

Use package paths without whitespace. The bounded SDF reader rejects unsupported
geometry, rotations and resource paths instead of approximating them silently.
The probe intentionally uses software EGL and Ogre2; it does not verify a
VMware graphics driver or the optional Ogre GUI fallback.

`--lift-position` applies a static offset only to the child of `lift_joint`; it
rejects positions outside the SDF joint limits and any unsupported axis. The
raised/lowered image comparison verifies visual attachment, not actuator
physics. Re-export old scene tables: the table format now includes three
emissive channels before transparency.

The vessel faces ROS +X. Looking toward its bow, port/left is ROS +Y (red) and
starboard/right is ROS -Y (green). A camera facing the boat from the bow sees
those colors on the opposite image sides. Lens panels sit on the existing
Studio-reference lamp housings and move with the mast; original mesh files are
unchanged. The panels illustrate color and placement, not certified visibility
sectors or photometric output.
