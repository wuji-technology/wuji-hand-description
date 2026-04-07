"""WUJI Hand Description RViz Display Launch File.

Launch robot_state_publisher and RViz for visualizing WUJI robot models.

Usage:
    ros2 launch wuji_hand_description display.launch.py robot:=left
    ros2 launch wuji_hand_description display.launch.py robot:=right
"""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_model_paths(robot_name: str):
    """Resolve URDF and RViz paths for installed-package and source-tree use."""
    try:
        package_share = Path(get_package_share_directory("wuji_hand_description"))
        return (
            package_share / "urdf" / f"{robot_name}-ros.urdf",
            package_share / "rviz" / f"{robot_name}.rviz",
            True,
        )
    except PackageNotFoundError:
        repo_root = Path(__file__).resolve().parents[1]
        return (
            repo_root / "urdf" / f"{robot_name}.urdf",
            repo_root / "rviz" / f"{robot_name}.rviz",
            False,
        )


def _load_robot_description(urdf_path: Path, use_package_paths: bool) -> str:
    """Load URDF text, rewriting local mesh paths to absolute paths when needed."""
    if use_package_paths:
        return urdf_path.read_text(encoding="utf-8")

    root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    urdf_dir = urdf_path.parent

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename and not filename.startswith(("package://", "file://", "/")):
            mesh.set("filename", (urdf_dir / filename).resolve().as_uri())

    return ET.tostring(root, encoding="unicode")


def _launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    use_gui = LaunchConfiguration("use_gui")

    if robot_name not in {"left", "right"}:
        raise ValueError(f"Unsupported robot '{robot_name}', expected 'left' or 'right'.")

    urdf_path, rviz_config_file, use_package_paths = _resolve_model_paths(robot_name)
    robot_description = {
        "robot_description": _load_robot_description(urdf_path, use_package_paths)
    }

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(use_gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", str(rviz_config_file)],
        ),
    ]


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot",
            default_value="left",
            description="Robot model to display (left or right)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Use joint_state_publisher_gui",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])
