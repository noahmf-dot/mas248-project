cmake_minimum_required(VERSION 3.8)
project(mas248_description)

find_package(ament_cmake REQUIRED)

# Install all files needed by the ROS 2 launch file.
install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
