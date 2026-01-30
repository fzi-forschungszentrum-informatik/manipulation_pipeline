cmake_minimum_required(VERSION 3.8)
project(manipulation_pipeline)

find_package(ament_cmake REQUIRED)

#
# Find dependencies
#
set(ROS_DEPENDENCIES
  geometric_shapes
  manipulation_pipeline_interfaces
  moveit_core
  moveit_ros_planning
  pilz_industrial_motion_planner
  rclcpp
  shape_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
foreach(DEP ${ROS_DEPENDENCIES})
  find_package(${DEP} REQUIRED)
endforeach()

find_package(fmt REQUIRED)

#
# Create executable
#
add_executable(manipulation_pipeline)
target_sources(manipulation_pipeline
  PRIVATE
    src/action.cpp
    src/actions/change_attached_object.cpp
    src/actions/dwell.cpp
    src/actions/execute_trajectory.cpp
    src/collision_object_manager.cpp
    src/execution_pipeline.cpp
    src/goal_handle.cpp
    src/manipulation_pipeline.cpp
    src/planner.cpp
    src/planning_context.cpp
    src/planning_interface.cpp
    src/planning_pipeline.cpp
    src/planning_step.cpp
    src/planning_steps/actuate_tool.cpp
    src/planning_steps/execute_path.cpp
    src/planning_steps/grasp.cpp
    src/planning_steps/move_to_named_pose.cpp
    src/planning_steps/move_to_pose.cpp
    src/planning_steps/move_to_configuration.cpp
    src/planning_steps/place.cpp
    src/robot_model.cpp
    src/robot_model_monitor.cpp
    src/visualization_interface.cpp
    src/worker_thread.cpp
)
target_include_directories(manipulation_pipeline
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(manipulation_pipeline
  PUBLIC
    c_std_11
    cxx_std_20
)
ament_target_dependencies(manipulation_pipeline
  ${ROS_DEPENDENCIES}
)
target_link_libraries(manipulation_pipeline
  fmt::fmt
)

#
# Installation
#
install(
  TARGETS
    manipulation_pipeline
  DESTINATION lib/${PROJECT_NAME}
)

#
# Testing
#
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
