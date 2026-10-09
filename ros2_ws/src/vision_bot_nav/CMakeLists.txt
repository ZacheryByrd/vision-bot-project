cmake_minimum_required(VERSION 3.8)
project(vision_bot_nav)

find_package(ament_cmake REQUIRED)

# Launch files, robot model, worlds, configs, and saved maps.
install(
  DIRECTORY config launch maps rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# Small helper nodes, run as `ros2 run vision_bot_nav <script>.py`.
install(
  PROGRAMS scripts/idle_stop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
