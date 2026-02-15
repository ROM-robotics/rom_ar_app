cmake_minimum_required(VERSION 3.8)
project(ar_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python nodes
install(PROGRAMS
  scripts/gesture_controller_node.py
  scripts/lidar_relay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
