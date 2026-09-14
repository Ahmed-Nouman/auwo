cmake_minimum_required(VERSION 3.8)
project(excavator_interactive_rviz)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

# --- RViz panel plugin (C++) ---
add_library(${PROJECT_NAME} SHARED
  src/preset_poses_panel.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  rviz_common
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

# Export RViz plugin description
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# --- Install C++ library & plugin XML ---
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- Python scripts (interactive markers + description topics) ---
install(PROGRAMS
  scripts/map_accumulator.py
  scripts/slew_scan.py
  scripts/joint_imarkers.py
  scripts/twin_router_node.py
  scripts/physical_tf_follower_node.py
  scripts/robot_description_topics.py
  scripts/self_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Launch files ---
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# --- Register package in ament index ---
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# --- Install package.xml ---
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

