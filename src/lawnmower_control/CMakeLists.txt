cmake_minimum_required(VERSION 3.5)
project(odom_yaw)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  odom_yaw/odom_yaw_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
