cmake_minimum_required(VERSION 3.8)
project(patrol_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp_v3
  std_srvs
)

include_directories(include)

add_library(pause_resume_controller_bt_node SHARED
  src/control/pause_resume_controller.cpp)
list(APPEND plugin_libs pause_resume_controller_bt_node)

add_library(persistent_sequence_bt_node SHARED
  src/control/persistent_sequence.cpp)
list(APPEND plugin_libs persistent_sequence_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/)
install(FILES bt_nodes.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})

ament_package()
