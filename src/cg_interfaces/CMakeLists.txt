cmake_minimum_required(VERSION 3.8)
project(cg_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveCmd.srv"
  "srv/GetMap.srv"
  "msg/RobotSensors.msg"
  "srv/Reset.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
