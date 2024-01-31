cmake_minimum_required(VERSION 3.5)
project(zeta2_bringup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Declare a C++ executable
add_executable(zeta_mc_node zeta_mc/src/zeta_mc.cpp zeta_mc/src/zeta_mc_node.cpp)
add_executable(zeta_if_node zeta_if/src/zeta_if.cpp zeta_if/src/zeta_if_node.cpp)

# Include directories
include_directories(
  include
  zeta_if/include
  zeta_mc/include
  serial/include
)

# Add serial library
add_subdirectory(serial)  # Adjust the path as needed

target_link_libraries(zeta_mc_node
  serial  # Link against the serial library
  # other libraries...
)

target_link_libraries(zeta_if_node
  serial  # Link against the serial library
  # other libraries...
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(zeta_if_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
)

ament_target_dependencies(zeta_mc_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
)

# Install targets
install(TARGETS
zeta_mc_node
zeta_if_node
DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/control.py
  scripts/odometry.py
  scripts/odom_mornitor.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files and other directories
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

