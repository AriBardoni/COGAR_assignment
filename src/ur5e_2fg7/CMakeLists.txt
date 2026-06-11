cmake_minimum_required(VERSION 3.8)
project(ur5e_2fg7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. TROVA LE DIPENDENZE ROS 2 E MOVEIT ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- 2. CONFIGURAZIONE COMPILAZIONE NODO C++ ---
# Definiamo l'eseguibile e dove trovare il codice sorgente
add_executable(pick_place_pipeline_node src/pick_place_node.cpp)

# Colleghiamo le librerie necessarie al nodo
ament_target_dependencies(pick_place_pipeline_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# --- 3. SEZIONE INSTALLAZIONE ---
# Installa le cartelle delle risorse geometriche (Le tue originali)
install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

# Installa l'eseguibile binario appena creato per poterlo lanciare con "ros2 run"
install(TARGETS
  pick_place_pipeline_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- 4. CONFIGURAZIONE TESTING E PACKAGING ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()