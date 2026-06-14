cmake_minimum_required(VERSION 3.8)
project(ur5e_2fg7)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(pick_place_pipeline_node src/pick_place_node.cpp)
ament_target_dependencies(pick_place_pipeline_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  control_msgs
  trajectory_msgs
)

install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  pick_place_pipeline_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
