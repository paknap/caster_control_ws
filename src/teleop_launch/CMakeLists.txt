cmake_minimum_required(VERSION 3.8)
project(teleop_launch)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/launch_teleop.sh DESTINATION lib/${PROJECT_NAME})
ament_package()
