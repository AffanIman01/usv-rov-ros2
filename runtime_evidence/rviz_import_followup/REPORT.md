# Follow-up: native RViz import verification

Verified 6 November 2026, Asia/Kuala_Lumpur, using the model resources published in commit `3dffc98ac06c00a45a6261645823c5395028045c`.

The user's orbit recording still shows the missing yellow cabin surfaces. The video alone cannot establish which Git revision, installed package directory or cached mesh RViz is displaying. A Gazebo render alone also does not verify RViz's different mesh-loading path, so this follow-up exercises RViz's actual installed library.

## Method and result

The probe calls `rviz_rendering::loadMeshFromResource` from installed ROS 2 Jazzy `rviz_rendering` 14.1.24 with Ogre 1.12.10. Ogre's CPU buffer manager makes the imported geometry available without a display server. It inspects the actual vertex and index buffers returned by RViz, including triangle winding and normals, plus material alpha, depth writing and face culling.

All 39 package meshes loaded successfully, totaling **182,316 triangles**. All 11 corrected ME120 resources preserve an opposite-wound triangle with opposite vertex normals for every imported triangle. All imported materials are opaque, write depth and use clockwise culling.

| Cabin asset | Imported triangles | Triangles with reverse-wound, opposite-normal counterpart | Result |
|---|---:|---:|---|
| Baseline before correction | 3,776 | 160 | Fails complete reversed-pair requirement |
| Current corrected asset | 7,552 | 7,552 | Passes |

The corrected cabin's seven submeshes therefore retain the added exterior-facing surfaces through RViz's importer. This test found no further mesh correction needed in that path. It does not establish that the user's VM has loaded the corrected files. Checking the active installed package and restarting RViz remain necessary to distinguish a stale asset from a separate rendering problem.

## Negative controls

The same executable was tested against the baseline cabin from commit `549eb948fdfca41f038ca23bd95020fc2a84a621`:

- Baseline with its correct 3,776-triangle count, without a complete-pair requirement: exit 0.
- Baseline with the complete reversed-pair requirement: exit 1, as expected.
- Corrected cabin with an incorrect 3,776-triangle expectation: exit 1, as expected.
- Corrected cabin with 7,552 triangles and the complete-pair requirement: exit 0.

This checks that missing backfaces and stale count expectations cause actual test failures, rather than merely printing statistics.

## Evidence and limits

The new `tools/preview_model.sh` helper was also run from outside the checkout with `--check-only`. Its isolated colcon build passed, and the checker confirmed that ROS selected the expected installation and that the ME120 URDF and all 17 meshes matched this checkout. The installed cabin reported 7,552 triangles. A deliberately stale cabin copy was rejected; selecting the wrong package directory was rejected; the installed ROV matched its source. See [preview build output](preview-build.log) and [installation check results](installation-checks.json).

The helper starts a new RViz process only after these checks pass. Stop an existing display launch with Ctrl+C first. It does not terminate existing processes, delete previous workspaces or change model assets.

- [All-mesh structured results](results.json)
- [All-mesh execution log](all-meshes.log)
- [Baseline comparison results](baseline-comparison.json)
- [Baseline comparison execution log](baseline-comparison.log)
- [Reproducible native probe and instructions](../../tools/runtime/rviz_mesh_import/README.md)

No model files were changed for this follow-up. This is an actual RViz **import** test, not an RViz GUI screenshot or rendering test. VMware graphics behavior, the user's installed file contents and the user's current process cache were not directly observed. Physics, sensor output and hardware integration are outside this check.
