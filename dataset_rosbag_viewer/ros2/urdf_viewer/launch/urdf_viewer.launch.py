from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Get package pathdefault_model_path")
    pkg_path = get_package_share_directory("urdf_viewer")

    # Paths to xacro and rviz config
    xacro_file = os.path.join(pkg_path, "urdf", "AMR", "efeu.xacro")
    rviz_config_file = os.path.join(pkg_path, "rviz", "SEW_AMR.rviz")

    # Process xacro file to URDF
    doc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": doc.toxml()}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            output="screen"
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[robot_description],
            output="screen"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[robot_description],
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])
