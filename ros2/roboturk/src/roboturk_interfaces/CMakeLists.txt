cmake_minimum_required(VERSION 3.8)
project(roboturk_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg" "msg/BallState.msg" "msg/WorldState.msg"
  "msg/RobotCommand.msg" "msg/CommandArray.msg" "msg/RefereeState.msg"
  "msg/MotionLimits.msg"
  "srv/ResetEpisode.srv"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
