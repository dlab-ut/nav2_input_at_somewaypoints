cmake_minimum_required(VERSION 3.8)
project(nav2_input_at_somewaypoints)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

link_libraries(stdc++fs)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  nav_msgs
  nav2_msgs
  nav2_util
  tf2_ros
  nav2_core
  pluginlib
)

add_library(input_at_somewaypoint SHARED src/input_at_somewaypoint.cpp)
ament_target_dependencies(input_at_somewaypoint ${dependencies})

install(
  TARGETS input_at_somewaypoint
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(input_at_somewaypoint)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_waypoint_follower plugins.xml)

ament_package()
