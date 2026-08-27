from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("diyrobot_ros"))
    robot_description = (share / "urdf" / "so101.urdf").read_text(encoding="utf-8")
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        Node(package="diyrobot_ros", executable="grasp_node", name="grasp_node",
             parameters=[share / "config" / "grasp.yaml"], output="screen"),
    ])
