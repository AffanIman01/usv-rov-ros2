# Sydney Regatta world

The default marine environment is an adaptation of **Virtual RobotX (VRX)'s
Sydney Regatta world**, by the Open Source Robotics Foundation and VRX
contributors. It uses the existing Sydney International Regatta Centre terrain,
buildings, docks and shore collision mesh from OpenRobotics. It is not a new
procedural pool.

## Pinned sources and attribution

| Resource | Source | License |
| --- | --- | --- |
| World, scene settings and geographic origin | [OSRF VRX `sydney_regatta.sdf`](https://github.com/osrf/vrx/blob/7609d1bd90ce7edb29d040a082f949e8b089c864/vrx_gz/worlds/sydney_regatta.sdf), Jazzy commit `7609d1bd90ce7edb29d040a082f949e8b089c864` | Apache-2.0; full text in `licenses/VRX-APACHE-2.0.txt` |
| Default boat location | [VRX `competition.launch.py`](https://github.com/osrf/vrx/blob/7609d1bd90ce7edb29d040a082f949e8b089c864/vrx_gz/launch/competition.launch.py) | Apache-2.0 |
| Sydney Regatta terrain, buildings, docks and shore collision mesh | [OpenRobotics / Sydney Regatta, Fuel version 3](https://fuel.gazebosim.org/1.0/OpenRobotics/models/sydney_regatta/3) | [Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/), credited to OpenRobotics |

The Fuel metadata describes the asset as "The Sydney Regatta Centre world."
It reports version 3 and 128,980,600 bytes. The preparation tool downloads this
exact archive and verifies its SHA-256 and every extracted resource against
`sydney_regatta.asset.json`. This repository does not redistribute its large
binary textures or meshes. The world uses `model://sydney_regatta`, resolved
from this verified local cache, avoiding a new Fuel request during startup.

## Changes for this project

- Keep the upstream terrain pose (`0 0 0.2`), sun, scene, geographic origin and
  core Gazebo systems.
- Spawn the ME120 at the upstream VRX launch location (`x=-532`, `y=162`,
  `yaw=1` rad), with its initial body origin at the estimated equilibrium height
  `z=0.152784564` m. The water surface is `z=0`, as in VRX.
- Use a close camera view, a 2 ms physics step and disabled shadows for initial
  VMware use.
- Replace the animated `coast_waves` visual with a flat, visual-only water plane.
  The ME120's own buoyancy, drag and propulsion systems provide its physics.
- Omit the competition props and VRX-specific wave, wind and competition
  plugins. No full VRX build is required for this calm-water adaptation.

The terrain is unchanged. This launch does **not** provide VRX wave dynamics,
competition tasks or calibrated ME120 hydrodynamics. The ocean is not a solid
surface: correct plugin loading is required for the boat to float.

## Download once, then run offline

From the repository root:

```bash
python3 tools/prepare_sydney_world.py
python3 tools/prepare_sydney_world.py --check-only
```

The first download can take several minutes. The default cache is
`.world_cache/sydney_regatta/` in the checkout. `--cache-dir /path/to/cache`
selects a different parent directory. Add that parent to `GZ_SIM_RESOURCE_PATH`
when launching directly with `ros2 launch` or `gz sim`. The launch helper does
this for the default cache. Once prepared, the world can start offline.

The archive checksum is
`ccf768134ff7e7929f67970fd059c0286569bc22ead9d959d45061a5c1ee967a`.
A damaged cache is preserved in a timestamped sibling directory before its
verified replacement is installed. A failed download never replaces it.

The old `pool.sdf` remains available for small, repeatable physics checks.
