---
name: ros2-jazzy-gazebo-harmonic
description: Use ROS 2 Jazzy and Gazebo Harmonic for this workspace. Use when installing ROS/Gazebo packages, writing launch files, bridging topics, debugging ros_gz_* nodes, or when the user asks about ROS distro, Gazebo version, gz sim, or AUWO simulation stack.
---

# ROS 2 Jazzy + Gazebo Harmonic

This workspace uses **ROS 2 Jazzy** and **Gazebo Harmonic** (gz-sim 8.x). Assume `ROS_DISTRO=jazzy`, Ubuntu 24.04 Noble.

## Stack summary

| Component | Version |
|-----------|---------|
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (gz sim 8.x) |
| Ubuntu | 24.04 Noble |

## Installing packages

```bash
# Debian packages: always use jazzy
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-moveit-planners-ompl
```

Never use `ros-humble-*`, `ros-iron-*`, or other distros unless the user explicitly asks for comparison.

## Launch and integration

- **Gazebo**: `gz sim -s` (server) / `gz sim -g` (GUI); use `-r` for run-on-start.
- **Bridge**: `ros_gz_sim`, `ros_gz_bridge` with Harmonic.
- **Transport partition**: set `GZ_PARTITION=auwo_sim` (or similar) so bridge and gz sim share the same partition.
- **Sim time**: bridge nodes run with `use_sim_time:=false` so `/clock` is published; other nodes use `use_sim_time:=true`.

## Official documentation

| Topic | URL |
|-------|-----|
| ROS 2 Jazzy | https://docs.ros.org/en/jazzy/ |
| Gazebo Harmonic | https://gazebosim.org/docs/harmonic/ |
| Gazebo ↔ ROS 2 | https://gazebosim.org/docs/harmonic/ros2_interop |
| ros_gz_bridge (Jazzy) | https://docs.ros.org/en/ros2_packages/jazzy/api/ros_gz_bridge/ |

Prefer these docs when answering ROS/Gazebo questions. Do not default to Humble, Fortress, Garden, or Gazebo Classic.

## Do not use (wrong for this repo)

- **ROS 2**: Humble, Iron, Rolling, Kilted — unless explicitly asked.
- **Gazebo**: Fortress, Garden, Ionic, Classic (`gazebo` binary, Gazebo 11) — this stack uses `gz sim` (Harmonic).
