#!/usr/bin/env bash
# Build this checkout and open its calm-water simulation, or its keyboard node.
set -euo pipefail
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
pool_ws="$repo_root/.pool_ws"
mode=sim
gui=true
software=false
engine=ogre2
while (($#)); do
  case "$1" in
    --keyboard) mode=keyboard ;;
    --check-only) mode=check ;;
    --headless) gui=false ;;
    --software) software=true ;;
    --render-engine)
      shift
      engine=${1:-}
      if [[ "$engine" != ogre && "$engine" != ogre2 ]]; then
        echo "--render-engine requires ogre or ogre2." >&2; exit 2
      fi ;;
    --help|-h)
      echo "Usage: bash tools/run_pool.sh [--keyboard | --check-only] [--headless] [--software] [--render-engine ogre2|ogre]"
      echo "Start the pool in one terminal, then --keyboard in a second terminal."
      exit 0 ;;
    *) echo "Unknown option: $1. Use --help." >&2; exit 2 ;;
  esac
  shift
done
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -x /usr/bin/colcon ]]; then
  echo "ROS 2 Jazzy and colcon are required on Ubuntu 24.04." >&2
  exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PATH="/usr/bin:/bin:$PATH"
export GZ_PARTITION="${GZ_PARTITION:-usv_rov_pool_$UID}"
if [[ "$mode" != keyboard ]]; then
  echo "Building pool simulation from: $repo_root"
  /usr/bin/colcon --log-base "$pool_ws/log" build \
    --base-paths "$repo_root/usv_rov_description" "$repo_root/usv_rov_sim" \
    --build-base "$pool_ws/build" --install-base "$pool_ws/install" \
    --packages-up-to usv_rov_sim --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
elif [[ ! -f "$pool_ws/install/local_setup.bash" ]]; then
  echo "First run bash tools/run_pool.sh in another terminal to build and start the pool." >&2
  exit 2
fi
set +u
source "$pool_ws/install/local_setup.bash"
set -u
/usr/bin/python3 "$repo_root/tools/check_installed_model.py" \
  --source "$repo_root/usv_rov_description" --model me120 \
  --expected-share "$pool_ws/install/usv_rov_description/share/usv_rov_description"
if [[ "$mode" == check ]]; then
  echo "Pool build and installed visual assets verified. This does not run the physics server."
  exit 0
elif [[ "$mode" == keyboard ]]; then
  exec /opt/ros/jazzy/bin/ros2 run usv_rov_sim keyboard_control
fi
echo "In a second terminal run: bash '$repo_root/tools/run_pool.sh' --keyboard"
exec /opt/ros/jazzy/bin/ros2 launch usv_rov_sim pool.launch.py \
  "gui:=$gui" "software_rendering:=$software" "render_engine:=$engine"
