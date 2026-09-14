cmake_minimum_required(VERSION 3.12)
project(auwo_control)

find_package(ament_cmake REQUIRED)

# Every .py in scripts/ is installed as an executable node.
# CONFIGURE_DEPENDS re-scans the folder at build time, so adding a new script
# needs no edit here - just rebuild.
file(GLOB AUWO_SCRIPTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
install(PROGRAMS ${AUWO_SCRIPTS} DESTINATION lib/${PROJECT_NAME})

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

ament_package()