cmake_minimum_required(VERSION 3.8)
project(mas248_ur5e)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf rviz config srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
