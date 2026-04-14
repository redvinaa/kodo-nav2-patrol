cmake_minimum_required(VERSION 3.8)
project(patrol_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY maps params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
