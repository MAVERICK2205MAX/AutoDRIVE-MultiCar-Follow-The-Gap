cmake_minimum_required(VERSION 3.8)
project(gap_follow)

# Utilizar C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Advertencias del compilador
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Sistema de construcción
find_package(ament_cmake REQUIRED)

# Dependencias del nodo
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 1. Ejecutable base del tutorial (reactive_node)
add_executable(
  reactive_node
  src/reactive_node.cpp
)
ament_target_dependencies(
  reactive_node
  rclcpp
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)

# 2. Ejecutable multirrobot parametrizado (reactive_node_multicar)
add_executable(
  reactive_node_multicar
  src/reactive_node_multicar.cpp
)
ament_target_dependencies(
  reactive_node_multicar
  rclcpp
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)

# Instalar los ejecutables
install(
  TARGETS
    reactive_node
    reactive_node_multicar
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar carpeta de launch si existe
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Configuración de pruebas
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
