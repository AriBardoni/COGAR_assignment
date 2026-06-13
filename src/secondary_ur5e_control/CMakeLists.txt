cmake_minimum_required(VERSION 3.8)
project(secondary_ur5e_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)

# Add the pick and place executable
add_executable(pick_and_place_node src/pick_and_place_node.cpp)

# Target include directories
target_include_directories(pick_and_place_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to the executable
ament_target_dependencies(pick_and_place_node
  rclcpp
  moveit_ros_planning_interface
  tf2_ros
)

# Install executable
install(TARGETS pick_and_place_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()