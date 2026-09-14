cmake_minimum_required(VERSION 3.12)
project(auwo_bringup)

find_package(ament_cmake REQUIRED)

# No nodes here - launch files, configuration, and Isaac Script Editor files.
install(DIRECTORY launch config isaac DESTINATION share/${PROJECT_NAME})

ament_package()