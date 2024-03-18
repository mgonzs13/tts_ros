cmake_minimum_required(VERSION 3.8)
project(tts_ros)

# find dependencies
find_package(ament_cmake REQUIRED)

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  tts_ros/tts_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tts_node
)

ament_package()
