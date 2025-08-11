cmake_minimum_required(VERSION 3.8)
project(manipulation_pipeline_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    action/ActuateTool.action
    action/ExecutePath.action
    action/Grasp.action
    action/MoveToNamedPose.action
    action/MoveToPose.action
    action/Place.action
    srv/SpawnObject.srv
    msg/CartesianLimits.msg
    msg/CollisionPair.msg
    msg/LinearMotion.msg
    msg/MotionParameters.msg
    msg/Progress.msg
    msg/State.msg
    msg/ToolCommand.msg
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
