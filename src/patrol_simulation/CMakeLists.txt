cmake_minimum_required(VERSION 3.8)
project(patrol_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

