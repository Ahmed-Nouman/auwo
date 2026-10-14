cmake_minimum_required(VERSION 3.16)
project(dumper_description)

find_package(ament_cmake REQUIRED)

# Gazebo model: models/dumper/{model.config,model.sdf,meshes/dumper.obj}
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

ament_package()
