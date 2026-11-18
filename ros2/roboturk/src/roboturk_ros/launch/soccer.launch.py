from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = Path(get_package_share_directory("roboturk_ros")) / "webots/worlds/ssl_training.wbt"
    simulator = ExecuteProcess(
        cmd=[LaunchConfiguration("webots"), "--mode=realtime", "--stdout", "--stderr", str(world)],
        output="screen",
        additional_env={"ROBOTURK_MODE": "ros"},
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("webots", default_value="webots"),
            DeclareLaunchArgument(
                "practice", default_value="false", description="Enable practice referee heartbeat"
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
