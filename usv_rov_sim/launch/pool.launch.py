"""Compatibility entry point for the small, offline physics test pool."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    common = Path(get_package_share_directory("usv_rov_sim")) / "launch" / "sim.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("render_engine", default_value="ogre2", choices=["ogre2", "ogre"]),
        DeclareLaunchArgument("software_rendering", default_value="false", choices=["true", "false"]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(common)), launch_arguments={
            "world": "pool", "gui": LaunchConfiguration("gui"),
            "render_engine": LaunchConfiguration("render_engine"),
            "software_rendering": LaunchConfiguration("software_rendering"),
        }.items()),
    ])
