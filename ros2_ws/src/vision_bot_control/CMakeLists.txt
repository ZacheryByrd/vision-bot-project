cmake_minimum_required(VERSION 3.8)
project(vision_bot_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# The control law, no ROS dependencies: unit tested on its own against the
# Python node's outputs (test/fixtures/reference.json).
add_library(tracking SHARED src/tracking.cpp)
target_include_directories(tracking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS tracking
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nlohmann_json REQUIRED)
  ament_add_gtest(test_tracking test/test_tracking.cpp)
  target_link_libraries(test_tracking tracking nlohmann_json::nlohmann_json)
  target_compile_definitions(test_tracking PRIVATE
    FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
endif()

ament_package()
