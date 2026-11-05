"""Display the exported visual model and its fixed coordinate frames in RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


MODELS = (
    "showcase", "me120", "m2_pro_max", "e_reel", "starlink_mini",
    "compute_enclosure", "recovery_concept",
)


def _launch_nodes(context):
    package_share = Path(get_package_share_directory("usv_rov_description"))
    model = LaunchConfiguration("model").perform(context)
    if model not in MODELS:
        raise ValueError(f"Unknown model {model!r}; choose one of {MODELS}")

    urdf_path = package_share / "urdf" / f"{model}.urdf"
    robot_description = urdf_path.read_text(encoding="utf-8")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(robot_description, value_type=str),
                "use_sim_time": False,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d", str(package_share / "rviz" / "model.rviz"),
                "-f", LaunchConfiguration("fixed_frame"),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value="showcase",
            choices=list(MODELS),
            description="Exported visual model to display; all models use base_link as their root.",
        ),
        DeclareLaunchArgument(
            "fixed_frame",
            default_value="base_link",
            description="RViz fixed frame. The URDF root is base_link for every model.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", choices=["true", "false"],
            description="Start RViz with the visual model configuration.",
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
