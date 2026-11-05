#!/usr/bin/env bash
# Run only after ROS 2 Jazzy / Harmonic and test dependencies are installed.
# This script does not install, update or remove software.
set -eo pipefail
test_tools_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
package_source="${1:?Usage: run_runtime_checks.sh PACKAGE_SOURCE [RESULT_DIRECTORY]}"
result_dir="${2:-$test_tools_dir/results}"
mkdir -p "$result_dir/logs"
result_dir="$(cd -- "$result_dir" && pwd)"
source /opt/ros/jazzy/setup.bash
export PATH="/usr/bin:/bin:$PATH"
export ROS_LOG_DIR="$result_dir/logs/ros"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=70
export GZ_PARTITION="usv_rov_model_validation_$$"
export GZ_IP="${GZ_IP:-127.0.0.1}"

/usr/bin/colcon --log-base "$result_dir/logs/colcon" build \
  --base-paths "$package_source" \
  --build-base "$result_dir/build" --install-base "$result_dir/install" \
  --packages-select usv_rov_description --event-handlers console_direct+ \
  2>&1 | tee "$result_dir/logs/colcon_build.log"
source "$result_dir/install/setup.bash"
package_share="$(/opt/ros/jazzy/bin/ros2 pkg prefix --share usv_rov_description)"
export GZ_SIM_RESOURCE_PATH="$(dirname "$package_share"):$package_share/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# gz sdf is a standalone SDFormat tool and does not install Gazebo's URI callback.
export SDF_PATH="$(dirname "$package_share"):$package_share/models${SDF_PATH:+:$SDF_PATH}"

for description in "$package_share"/urdf/*.urdf; do
  check_urdf "$description" > "$result_dir/logs/$(basename "$description").check.log" 2>&1
done
/usr/bin/python3 "$test_tools_dir/check_static_tf.py" "$package_share" \
  --log-dir "$result_dir/logs" | tee "$result_dir/logs/static_tf_summary.log"
for model_file in "$package_share"/models/*/model.sdf; do
  gz sdf -k "$model_file" > "$result_dir/logs/$(basename "$(dirname "$model_file")").sdf_check.log" 2>&1
done
gz sdf -k "$package_share/worlds/model_preview.sdf" > "$result_dir/logs/world.sdf_check.log" 2>&1

/usr/bin/python3 "$test_tools_dir/build_mesh_expectations.py" "$package_share/meshes" "$result_dir/mesh_expectations.tsv"
/usr/bin/cmake -S "$test_tools_dir/native" -B "$result_dir/native_build" \
  2>&1 | tee "$result_dir/logs/native_cmake.log"
/usr/bin/cmake --build "$result_dir/native_build" -j2 \
  2>&1 | tee "$result_dir/logs/native_build.log"
"$result_dir/native_build/check_gazebo_meshes" "$result_dir/mesh_expectations.tsv" \
  2>&1 | tee "$result_dir/logs/gazebo_mesh_import.log"
"$result_dir/native_build/check_gazebo_world" "$package_share/worlds/model_preview.sdf" \
  2>&1 | tee "$result_dir/logs/gazebo_world_runtime.log"
printf 'ROS descriptions, TF, SDF, native mesh import and headless world checks passed.\nRenderer and physics-fidelity validation are separate.\n'
