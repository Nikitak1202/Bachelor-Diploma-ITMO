cmake_minimum_required(VERSION 3.8)
project(omni_behavior)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(library_name omni_target_visible_condition_bt_node)

add_library(${library_name} SHARED
  src/target_visible_condition.cpp
)

ament_target_dependencies(${library_name}
  rclcpp
  behaviortree_cpp_v3
  nav2_behavior_tree
  std_msgs
)

target_compile_definitions(${library_name} PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(rclcpp behaviortree_cpp_v3 nav2_behavior_tree std_msgs)

ament_package()
