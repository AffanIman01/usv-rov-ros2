"""Launch the prototype calm-water USV world and its simulation-only ROS bridge."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    sim = Path(get_package_share_directory("usv_rov_sim"))
    description = Path(get_package_share_directory("usv_rov_description"))
    resources = [str(description.parent), str(sim / "models")]
    plugins = [str(Path(get_package_prefix("usv_rov_sim")) / "lib")]
    for paths, variable in ((resources, "GZ_SIM_RESOURCE_PATH"), (plugins, "GZ_SIM_SYSTEM_PLUGIN_PATH")):
        if os.environ.get(variable):
            paths.append(os.environ[variable])
    gui = LaunchConfiguration("gui").perform(context) == "true"
    engine = LaunchConfiguration("render_engine").perform(context)
    command = ["gz", "sim", "-r", "-v", "3", "--render-engine", engine]
    if not gui:
        command.append("-s")
    command.append(str(sim / "worlds" / "pool.sdf"))
    actions = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", value=os.pathsep.join(resources)),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", value=os.pathsep.join(plugins)),
        SetEnvironmentVariable("GZ_PARTITION", value=os.environ.get("GZ_PARTITION", f"usv_rov_pool_{os.getuid()}")),
    ]
    if LaunchConfiguration("software_rendering").perform(context) == "true":
        actions.append(SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", value="1"))
    actions.extend([
        ExecuteProcess(cmd=command, output="screen", on_exit=Shutdown(reason="Gazebo exited")),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="usv_pool_bridge",
             parameters=[{"config_file": str(sim / "config" / "bridge.yaml")}],
             output="screen", on_exit=Shutdown(reason="Simulation bridge exited")),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("render_engine", default_value="ogre2", choices=["ogre2", "ogre"]),
        DeclareLaunchArgument("software_rendering", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=_launch),
    ])
