"""Launch ME120 in the small test pool, with Sydney Regatta as an option."""

import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _process_exit(component, *, stop_simulation):
    """Keep monitor faults visible without closing the scene under inspection."""
    def on_exit(event, context):
        if context.is_shutdown:
            # Ctrl+C and coordinated shutdown are not fresh crash reports.
            return []
        code = event.returncode
        prefix = "ERROR: " if code else ""
        message = f"{prefix}{component} exited with code {code}. "
        if not stop_simulation:
            return [LogInfo(msg=message +
                "Gazebo stays open for inspection; passive monitoring has stopped. "
                "Read the monitor diagnostic above and stop the launch with Ctrl+C when finished.")]
        reason = f"{component} exited with code {code}"
        return [
            LogInfo(msg=message +
                "Stopping the remaining simulation processes. Inspect this process's preceding output for the cause."),
            Shutdown(reason=reason),
        ]
    return on_exit


def _launch(context):
    sim = Path(get_package_share_directory("usv_rov_sim"))
    prefix = Path(get_package_prefix("usv_rov_sim"))
    description = Path(get_package_share_directory("usv_rov_description"))
    resources = [str(description.parent), str(sim / "models")]
    plugins = [str(prefix / "lib")]
    env = dict(context.environment)
    for paths, variable in ((resources, "GZ_SIM_RESOURCE_PATH"), (plugins, "GZ_SIM_SYSTEM_PLUGIN_PATH")):
        if env.get(variable):
            paths.append(env[variable])
        env[variable] = os.pathsep.join(paths)
    env["GZ_PARTITION"] = env.get("GZ_PARTITION", f"usv_rov_sim_{os.getuid()}")

    # A visible mesh does not prove that Gazebo loaded either force plugin.
    # Use the exact installed SDF and loader search path before starting physics.
    checker = prefix / "lib" / "usv_rov_sim" / "check_sim_plugins"
    if not checker.is_file():
        raise RuntimeError("Simulation installation is out of date. Run bash tools/run_sim.sh to rebuild this checkout.")
    checked = subprocess.run(
        [str(checker), str(sim / "models" / "me120_dynamic" / "model.sdf")],
        env=env, text=True, capture_output=True, timeout=20, check=False,
    )
    if checked.returncode:
        raise RuntimeError("ME120 force plugins cannot load; simulation was not started.\n" + checked.stdout + checked.stderr)

    selected = LaunchConfiguration("world").perform(context)
    if selected == "sydney_regatta" and not any(
        (Path(path) / "sydney_regatta" / "model.sdf").is_file()
        for path in env["GZ_SIM_RESOURCE_PATH"].split(os.pathsep) if path
    ):
        raise RuntimeError("Sydney Regatta terrain is not prepared. Run bash tools/run_sim.sh --world sydney_regatta; it downloads and verifies the pinned world asset once.")
    gui = LaunchConfiguration("gui").perform(context) == "true"
    engine = LaunchConfiguration("render_engine").perform(context)
    # Jazzy's bridge and these plugins use gz-sim8. The unqualified CLI can
    # otherwise choose a separately installed newer, ABI-incompatible Gazebo.
    command = ["gz", "sim", "--force-version", "8", "-r", "-v", "3", "--render-engine", engine]
    if not gui:
        command.append("-s")
    command.append(str(sim / "worlds" / f"{selected}.sdf"))
    actions = [SetEnvironmentVariable(key, value=env[key]) for key in
               ("GZ_SIM_RESOURCE_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH", "GZ_PARTITION")]
    if LaunchConfiguration("software_rendering").perform(context) == "true":
        actions.append(SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", value="1"))
    actions.extend([
        LogInfo(msg=f"ME120 world: {selected}; Gazebo Harmonic (8); ROS domain: {env.get('ROS_DOMAIN_ID', '0')}; Gazebo partition: {env['GZ_PARTITION']}"),
        LogInfo(msg=checked.stdout.strip()),
        ExecuteProcess(cmd=command, output="screen",
             on_exit=_process_exit("Gazebo", stop_simulation=True)),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="usv_sim_bridge",
             parameters=[{"config_file": str(sim / "config" / ("sydney_bridge.yaml" if selected == "sydney_regatta" else "bridge.yaml"))}],
             output="screen", on_exit=_process_exit("ROS/Gazebo bridge", stop_simulation=True)),
        Node(package="usv_rov_sim", executable="simulation_guard", name="usv_sim_guard",
             output="screen", on_exit=_process_exit("Simulation monitor", stop_simulation=False)),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="pool", choices=["pool", "sydney_regatta"]),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("render_engine", default_value="ogre2", choices=["ogre2", "ogre"]),
        DeclareLaunchArgument("software_rendering", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=_launch),
    ])
