#!/usr/bin/env bash
# Build this checkout into a separate overlay and verify the model before RViz.
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
model=me120
model_set=false
check_only=false
for arg in "$@"; do
  case "$arg" in
    --check-only) check_only=true ;;
    -h|--help)
      echo "Usage: bash tools/preview_model.sh [MODEL] [--check-only]"
      echo "MODEL defaults to me120. --check-only builds and verifies without opening RViz."
      exit 0 ;;
    me120|m2_pro_max|showcase|e_reel|starlink_mini|compute_enclosure|recovery_concept)
      if "$model_set"; then echo "Specify only one model." >&2; exit 2; fi
      model=$arg
      model_set=true ;;
    *) echo "Unknown argument: $arg. Use --help for usage." >&2; exit 2 ;;
  esac
done

for required in /opt/ros/jazzy/setup.bash /usr/bin/colcon /usr/bin/python3; do
  if [[ ! -f "$required" ]]; then
    echo "Missing $required. This preview requires Ubuntu 24.04 with ROS 2 Jazzy and colcon." >&2
    exit 2
  fi
done

preview_ws="$repo_root/.preview_ws"
echo "Building checkout: $repo_root"
echo "Preview workspace: $preview_ws"
echo "Close any earlier display launch before opening this preview."

# ROS setup scripts may reference variables that are not yet defined.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PATH="/usr/bin:/bin:$PATH"
/usr/bin/colcon --log-base "$preview_ws/log" build \
  --base-paths "$repo_root/usv_rov_description" \
  --build-base "$preview_ws/build" \
  --install-base "$preview_ws/install" \
  --packages-select usv_rov_description \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

set +u
source "$preview_ws/install/local_setup.bash"
set -u
/usr/bin/python3 "$repo_root/tools/check_installed_model.py" \
  --source "$repo_root/usv_rov_description" --model "$model" \
  --expected-share "$preview_ws/install/usv_rov_description/share/usv_rov_description"

if "$check_only"; then exit 0; fi
exec /opt/ros/jazzy/bin/ros2 launch usv_rov_description display.launch.py "model:=$model"
