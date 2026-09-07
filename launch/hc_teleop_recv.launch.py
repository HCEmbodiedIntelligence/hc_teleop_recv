"""Start a frontend using a resolved robot-model configuration."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", description="Absolute path to hc_teleop YAML"),
        Node(
            package="hc_teleop_recv", executable="hc_teleop_recv_node",
            name="hc_teleop_recv", output="screen",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
        ),
    ])
