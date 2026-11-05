"""Open static exported models in Gazebo; no vehicle dynamics are provided."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    package_share = Path(get_package_share_directory("usv_rov_description"))
    # The share parent resolves model://usv_rov_description/meshes/...;
    # the models directory resolves separately named model:// model folders.
    search_paths = [str(package_share.parent), str(package_share / "models")]
    existing_paths = os.environ.get("GZ_SIM_RESOURCE_PATH")
    if existing_paths:
        search_paths.append(existing_paths)

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH", value=os.pathsep.join(search_paths),
        ),
        ExecuteProcess(
            cmd=["gz", "sim", str(package_share / "worlds" / "model_preview.sdf")],
            output="screen",
        ),
    ])
