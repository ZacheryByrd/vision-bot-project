cmake_minimum_required(VERSION 3.8)
project(vision_bot_plugins)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Pure cost math, no ROS dependencies: unit tested on its own.
add_library(avoidance_cost SHARED src/avoidance_cost.cpp)
target_include_directories(avoidance_cost PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# The costmap layer plugin (ROS plumbing around the pure functions).
add_library(vision_bot_plugins SHARED src/robot_avoidance_layer.cpp)
target_link_libraries(vision_bot_plugins avoidance_cost)
ament_target_dependencies(vision_bot_plugins
  rclcpp rclcpp_lifecycle nav2_costmap_2d nav2_util pluginlib geometry_msgs
  rcl_interfaces tf2 tf2_ros tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS avoidance_cost vision_bot_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_avoidance_cost test/test_avoidance_cost.cpp)
  target_link_libraries(test_avoidance_cost avoidance_cost)

  # Nav2 loads the plugin into a costmap and activates it without errors.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_plugin_load.py TIMEOUT 120)
endif()

ament_package()
