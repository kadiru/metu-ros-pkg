from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    world = Path(get_package_share_directory("roboturk_ros")) / "webots/worlds/ssl_training.wbt"
    simulator = ExecuteProcess(
        cmd=[LaunchConfiguration("webots"), "--mode=realtime", "--stdout", "--stderr", str(world)],
        output="screen",
        additional_env={
            "ROBOTURK_MODE": "ros",
            "ROBOTURK_PRACTICE": LaunchConfiguration("practice"),
            "ROBOTURK_PRACTICE_LIMITS": LaunchConfiguration("practice_limits_file"),
        },
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("webots", default_value="webots"),
            DeclareLaunchArgument(
                "practice", default_value="false", description="Enable practice referee heartbeat"
            ),
            DeclareLaunchArgument(
                "practice_limits_file", default_value="", description="Shared Practice limits JSON"
            ),
            DeclareLaunchArgument("referee_address", default_value="224.5.23.1"),
            DeclareLaunchArgument("referee_port", default_value="10003"),
            DeclareLaunchArgument("referee_interface", default_value="0.0.0.0"),
            Node(
                package="roboturk_ros",
                executable="ssl_referee",
                parameters=[
                    {
                        "use_sim_time": True,
                        "address": LaunchConfiguration("referee_address"),
                        "port": ParameterValue(LaunchConfiguration("referee_port"), value_type=int),
                        "interface": LaunchConfiguration("referee_interface"),
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration("practice")),
            ),
            simulator,
            *[
                Node(
                    package="roboturk_ros",
                    executable="strategy",
                    name=f"{team}_strategy",
                    parameters=[{"team": team, "use_sim_time": True}],
                    output="screen",
                )
                for team in ("blue", "yellow")
            ],
            Node(
                package="roboturk_ros",
                executable="practice_referee",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("practice")),
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=simulator, on_exit=[EmitEvent(event=Shutdown())])
            ),
        ]
    )
