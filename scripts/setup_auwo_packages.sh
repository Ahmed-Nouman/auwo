#!/usr/bin/env bash
#
# setup_auwo_packages.sh — create the AUWO package layout in auwo_ws.
#
# Creates auwo_control, auwo_perception and auwo_bringup with working
# CMakeLists.txt and package.xml, then moves the existing scripts out of
# excavator_interactive_rviz.
#
# Safe to re-run: it never overwrites a package that already exists, and it
# only moves files that are still in the old location.
#
#   bash setup_auwo_packages.sh
#
# WHY A GLOB IN CMakeLists
# The usual pattern lists every script by name in install(PROGRAMS ...), and
# CMake hard-errors if any listed file is missing - and silently skips any file
# you forgot to list. Both failure modes have already cost time. Instead:
#
#     file(GLOB SCRIPTS CONFIGURE_DEPENDS scripts/*.py)
#     install(PROGRAMS ${SCRIPTS} DESTINATION lib/${PROJECT_NAME})
#
# Everything in scripts/ is installed. Add a file, rebuild, done. CONFIGURE_DEPENDS
# makes CMake re-scan the folder at build time so new files are picked up without
# a clean build (needs CMake 3.12+, you have it).
#
# The trade: a stray or half-finished .py in scripts/ also gets installed. That
# is a much cheaper mistake than the two above.

set -e
WS="${1:-$HOME/Desktop/auwo_ws}"
SRC="$WS/src"

[ -d "$SRC" ] || { echo "not a workspace: $SRC"; exit 1; }
echo "workspace: $WS"

# --------------------------------------------------------------- package.xml
write_package_xml () {   # $1 = name, $2 = description, $3.. = exec deps
  local name="$1" desc="$2"; shift 2
  local deps=""
  for d in "$@"; do deps="$deps  <exec_depend>$d</exec_depend>\n"; done
  cat > "$SRC/$name/package.xml" <<EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>$name</name>
  <version>0.1.0</version>
  <description>$desc</description>
  <maintainer email="you@centria.fi">AUWO</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

$(printf "$deps")
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
}

# ------------------------------------------------------------ CMakeLists.txt
write_cmake_scripts () {   # $1 = name  (package that installs python scripts)
  cat > "$SRC/$1/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.12)
project($1)

find_package(ament_cmake REQUIRED)

# Every .py in scripts/ is installed as an executable node.
# CONFIGURE_DEPENDS re-scans the folder at build time, so adding a new script
# needs no edit here - just rebuild.
file(GLOB AUWO_SCRIPTS CONFIGURE_DEPENDS \${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
install(PROGRAMS \${AUWO_SCRIPTS} DESTINATION lib/\${PROJECT_NAME})

# Optional per-package config, installed if the folder exists.
if(EXISTS \${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/\${PROJECT_NAME})
endif()

ament_package()
EOF
}

write_cmake_bringup () {
  cat > "$SRC/auwo_bringup/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.12)
project(auwo_bringup)

find_package(ament_cmake REQUIRED)

# No nodes here - launch files, configuration, and Isaac Script Editor files.
install(DIRECTORY launch config isaac DESTINATION share/${PROJECT_NAME})

ament_package()
EOF
}

# --------------------------------------------------------------------- build
make_pkg () {   # $1 name, $2 desc, $3.. deps
  local name="$1"
  if [ -d "$SRC/$name" ]; then
    echo "  $name already exists, leaving it alone"
    return
  fi
  echo "  creating $name"
  mkdir -p "$SRC/$name/scripts"
  write_package_xml "$@"
  write_cmake_scripts "$name"
}

echo
echo "== packages =="
make_pkg auwo_control    "AUWO actuation: arbiter, teleop, sim bridge" \
         rclpy std_msgs sensor_msgs
make_pkg auwo_perception "AUWO world model: mapping, filtering, monitoring" \
         rclpy std_msgs sensor_msgs geometry_msgs tf2_ros

if [ ! -d "$SRC/auwo_bringup" ]; then
  echo "  creating auwo_bringup"
  mkdir -p "$SRC/auwo_bringup"/{launch,config/sensors,config/rviz,isaac/lidar_configs}
  write_package_xml auwo_bringup "AUWO launch files, configuration and Isaac assets" \
      auwo_control auwo_perception excavator_description robot_state_publisher \
      tf2_ros rviz2 joy
  write_cmake_bringup
  # keep empty dirs in git
  for d in launch config/sensors config/rviz isaac/lidar_configs; do
    touch "$SRC/auwo_bringup/$d/.gitkeep"
  done
else
  echo "  auwo_bringup already exists, leaving it alone"
fi

# ------------------------------------------------------------------ migrate
echo
echo "== migrating from excavator_interactive_rviz =="
OLD="$SRC/excavator_interactive_rviz"
move () {   # $1 file, $2 destination package
  if [ -f "$OLD/scripts/$1" ]; then
    mv "$OLD/scripts/$1" "$SRC/$2/scripts/$1"
    echo "  $1 -> $2"
  fi
}
move lidar_mapper.py        auwo_perception
move dig_zone_monitor.py    auwo_perception
move isaac_command_bridge.py auwo_control

if [ -d "$OLD/config/sensors" ]; then
  mkdir -p "$SRC/auwo_bringup/config/sensors"
  mv "$OLD/config/sensors"/*.yaml "$SRC/auwo_bringup/config/sensors/" 2>/dev/null && \
    echo "  sensor configs -> auwo_bringup/config/sensors" || true
fi

chmod +x "$SRC"/auwo_*/scripts/*.py 2>/dev/null || true

# ------------------------------------------------------------------- report
echo
echo "== result =="
for p in auwo_control auwo_perception auwo_bringup; do
  echo "$p:"
  find "$SRC/$p" -name "*.py" -o -name "*.yaml" | sed 's|^|    |' | sed "s|$SRC/||"
done

cat <<'EOF'

NEXT
  1. Put the downloaded scripts in place:
        auwo_arbiter.py  auwo_joy_teleop.py  keyboard_to_joy.py
            -> src/auwo_control/scripts/
        sensor_rig.py  add_surround_cameras.py
            -> src/auwo_bringup/isaac/
        isaac_twin.launch.py
            -> src/auwo_bringup/launch/
     then:  chmod +x src/auwo_*/scripts/*.py

  2. Remove the moved files from excavator_interactive_rviz/CMakeLists.txt
     (its install(PROGRAMS ...) list will now name files that are gone, and
     CMake hard-errors on that).

  3. Build:
        cd <workspace> && colcon build --symlink-install && source install/setup.bash

  4. Check the nodes are visible:
        ros2 pkg executables auwo_control
        ros2 pkg executables auwo_perception
EOF