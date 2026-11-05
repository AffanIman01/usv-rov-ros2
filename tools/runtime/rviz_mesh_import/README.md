# Native RViz mesh import check

This probe calls the installed `rviz_rendering::loadMeshFromResource` library and reads the Ogre vertex/index buffers it creates. It does not open RViz, create a display, render pixels or start a ROS graph. Ogre's default CPU hardware-buffer manager permits this check without a GPU.

For every mesh it verifies the expected triangle count, opaque materials, depth writing and clockwise face culling. For the 11 corrected ME120 resources, it also requires every triangle to have a reverse-wound partner with opposite vertex normals in the imported buffers. Pair matching rounds coordinates and normal components to six decimal places. Other models retain their original one-sided surfaces.

The helper returns a nonzero exit status for any failed assertion. `check_models.py` compares all 39 imported mesh counts with the COLLADA source data and writes structured results.

## Build and run

Use Ubuntu 24.04 with ROS 2 Jazzy and its `rviz_rendering` development dependencies installed. From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S tools/runtime/rviz_mesh_import -B /tmp/usv-rviz-import \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/usv-rviz-import -j2
python3 tools/runtime/rviz_mesh_import/check_models.py usv_rov_description \
  --probe /tmp/usv-rviz-import/rviz_mesh_probe \
  --results /tmp/usv-rviz-import-results.json \
  > /tmp/usv-rviz-import.log 2>&1
```

To check the cabin alone:

```bash
/tmp/usv-rviz-import/rviz_mesh_probe \
  "$PWD/usv_rov_description/meshes/me120/equipment_cabin.dae" \
  7552 --require-reversed-pairs
```

The corrected cabin must report 7,552 triangles and 7,552 reverse-normal matches. The old cabin has 3,776 triangles and fails the reversed-pair requirement. This is an importer compatibility check, not proof of what files a separate user's running RViz instance has loaded.

Actual test results and negative controls are in `../../../runtime_evidence/rviz_import_followup/REPORT.md`.
