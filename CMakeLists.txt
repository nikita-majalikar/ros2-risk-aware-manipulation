cmake_minimum_required(VERSION 3.8)
project(risk_aware_manipulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
