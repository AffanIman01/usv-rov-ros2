#!/usr/bin/env bash
# Build this checkout and open the ME120 simulation, or its keyboard node.
set -euo pipefail
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
pool_ws="$repo_root/.pool_ws"
mode=sim
world=pool
gui=true
software=false
engine=ogre2
while (($#)); do
  case "$1" in
    --keyboard) mode=keyboard ;;
    --check-only) mode=check ;;
    --diagnose) mode=diagnose ;;
    --self-test) mode=test ;;
    --world)
      shift
      world=${1:-}
      if [[ "$world" != sydney_regatta && "$world" != pool ]]; then
        echo "--world requires sydney_regatta or pool." >&2; exit 2
      fi ;;
    --headless) gui=false ;;
    --software) software=true ;;
    --render-engine)
      shift
      engine=${1:-}
      if [[ "$engine" != ogre && "$engine" != ogre2 ]]; then
        echo "--render-engine requires ogre or ogre2." >&2; exit 2
      fi ;;
    --help|-h)
      echo "Usage: bash tools/run_sim.sh [--keyboard | --check-only | --self-test | --diagnose] [--world pool|sydney_regatta] [--headless] [--software] [--render-engine ogre2|ogre]"
      echo "Default: small offline pool. Sydney terrain is optional with --world sydney_regatta."
      echo "Start the simulation in one terminal, then --keyboard in a second terminal."
      exit 0 ;;
    *) echo "Unknown option: $1. Use --help." >&2; exit 2 ;;
  esac
  shift
done
if [[ "$mode" == diagnose ]]; then
  exec /usr/bin/python3 "$repo_root/tools/sim_diagnostics.py" --workspace "$pool_ws"
fi
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -x /usr/bin/colcon ]]; then
  echo "ROS 2 Jazzy and colcon are required on Ubuntu 24.04." >&2
  exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PATH="/usr/bin:/bin:$PATH"
export GZ_PARTITION="${GZ_PARTITION:-usv_rov_sim_$UID}"
export GZ_SIM_RESOURCE_PATH="$repo_root/.world_cache${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
if [[ "$mode" != keyboard ]]; then
  echo "Building ME120 simulation from: $repo_root"
  /usr/bin/colcon --log-base "$pool_ws/log" build \
    --base-paths "$repo_root/usv_rov_description" "$repo_root/usv_rov_sim" \
    --build-base "$pool_ws/build" --install-base "$pool_ws/install" \
    --packages-up-to usv_rov_sim --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
elif [[ ! -f "$pool_ws/install/local_setup.bash" ]]; then
  echo "First run bash tools/run_sim.sh in another terminal to build and start the simulation." >&2
  exit 2
fi
set +u
source "$pool_ws/install/local_setup.bash"
set -u
/usr/bin/python3 "$repo_root/tools/check_installed_model.py" \
  --source "$repo_root/usv_rov_description" --model me120 \
  --expected-share "$pool_ws/install/usv_rov_description/share/usv_rov_description"
if [[ "$mode" == check ]]; then
  /opt/ros/jazzy/bin/ros2 run usv_rov_sim check_sim_plugins
  echo "Build, installed visuals and force-plugin loading verified. This does not run the physics server."
  exit 0
elif [[ "$mode" == keyboard ]]; then
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}; use the same domain as the simulation terminal."
  exec /opt/ros/jazzy/bin/ros2 run usv_rov_sim keyboard_control
fi
if [[ "$world" == sydney_regatta ]]; then
  /usr/bin/python3 "$repo_root/tools/prepare_sydney_world.py" --cache-dir "$repo_root/.world_cache"
fi
if [[ "$mode" == test ]]; then
  exec /usr/bin/python3 "$repo_root/tools/runtime/marine/ros_bridge_smoke.py" \
    --world "$world" --timeout 240 --startup-timeout 120 \
    --log "$pool_ws/last-test.log" --output "$pool_ws/last-test.json"
fi
echo "In a second terminal run: bash '$repo_root/tools/run_sim.sh' --keyboard"
mkdir -p "$pool_ws/runs"
run_stamp=$(date -u +%Y%m%dT%H%M%S)-$$
if [[ -s "$pool_ws/last-launch.log" ]]; then
  cp -- "$pool_ws/last-launch.log" "$pool_ws/runs/before-$run_stamp.log"
fi
run_log="$pool_ws/runs/launch-$run_stamp.log"
echo "World: $world. Each run keeps its own log; previous crash logs are preserved."
echo "Launch log: $run_log"
/opt/ros/jazzy/bin/ros2 launch usv_rov_sim sim.launch.py \
  "world:=$world" "gui:=$gui" "software_rendering:=$software" "render_engine:=$engine" \
  2>&1 | tee "$run_log" "$pool_ws/last-launch.log"
