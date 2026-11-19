"""Start openarm_teleop_node with config/teleop.yaml.

  ros2 launch openarm_teleop_cpp teleop.launch.py
  ros2 launch openarm_teleop_cpp teleop.launch.py mode:=sim arms:=right cmd_rate_hz:=0
  ros2 launch openarm_teleop_cpp teleop.launch.py params_file:=/path/to/my.yaml

Start the robot (openarm_bringup) first, and stop the dora dataflow: both bind
UDP 5006 and publish to the same controllers.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("openarm_teleop_cpp"))
    args = [
        DeclareLaunchArgument("params_file", default_value=str(share / "config" / "teleop.yaml"),
                              description="node parameters"),
        DeclareLaunchArgument("mode", default_value="real", description="real | sim"),
        DeclareLaunchArgument("arms", default_value="bimanual", description="bimanual | right | left"),
        DeclareLaunchArgument("cmd_rate_hz", default_value="200.0",
                              description="arm command timer rate, 0 = publish every tick"),
    ]
    node = Node(
        package="openarm_teleop_cpp",
        executable="openarm_teleop_node",
        name="openarm_teleop",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "mode": LaunchConfiguration("mode"),
                "arms": LaunchConfiguration("arms"),
                # cmd_rate_hz:=200 would otherwise arrive as an int.
                "cmd_rate_hz": ParameterValue(LaunchConfiguration("cmd_rate_hz"), value_type=float),
            },
        ],
    )
    return LaunchDescription([*args, node])
