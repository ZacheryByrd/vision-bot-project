cmake_minimum_required(VERSION 3.8)
project(vision_bot_perception)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn imgcodecs)

# The detectors, no ROS dependencies: unit tested on their own against the
# Python node's outputs (test/fixtures/reference.json).
add_library(detectors SHARED src/detectors.cpp)
target_include_directories(detectors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(detectors ${OpenCV_LIBS})

install(TARGETS detectors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nlohmann_json REQUIRED)
  ament_add_gtest(test_detectors test/test_detectors.cpp)
  target_link_libraries(test_detectors detectors nlohmann_json::nlohmann_json)
  target_compile_definitions(test_detectors PRIVATE
    FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
    VISION_BOT_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../vision_bot")
endif()

ament_package()
