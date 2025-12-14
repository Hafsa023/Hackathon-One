---
sidebar_position: 4
title: Version Compatibility
description: Software version matrix and compatibility requirements for Physical AI development
---

# Version Compatibility Matrix

This page provides the definitive version compatibility information for all software components used in the Physical AI book.

## Primary Software Stack

| Component | Version | Support Window | Ubuntu Package |
|-----------|---------|----------------|----------------|
| **Ubuntu** | 22.04 LTS | Apr 2022 - Apr 2032 | N/A |
| **ROS 2** | Humble Hawksbill | May 2022 - May 2027 | `ros-humble-desktop` |
| **Gazebo** | Fortress (Ignition) | Sep 2021 - Sep 2026 | `ignition-fortress` |
| **Python** | 3.10+ | Oct 2021 - Oct 2026 | `python3` |
| **CUDA** | 12.0+ | 2023+ | NVIDIA installer |

## NVIDIA Isaac Stack

| Component | Version | Requirements |
|-----------|---------|--------------|
| **Isaac Sim** | 2023.1.1+ | RTX GPU, 32GB+ RAM |
| **Isaac ROS** | 2.0+ | ROS 2 Humble |
| **cuVSLAM** | 2.0+ | Isaac ROS 2.0 |
| **Isaac Perceptor** | 1.0+ | Isaac Sim 2023.1+ |

:::caution Hardware Requirements
Isaac Sim requires an NVIDIA RTX GPU (minimum RTX 3070) and is not fully supported on WSL2. Ubuntu 22.04 native installation is recommended.
:::

## Unity Robotics Stack

| Component | Version | Notes |
|-----------|---------|-------|
| **Unity Editor** | 2022.3 LTS | Long-term support |
| **Unity Robotics Hub** | 0.7+ | ROS 2 support |
| **ROS-TCP-Connector** | 0.7+ | ROS 2 Humble compatible |
| **URDF Importer** | 0.5+ | Via Robotics Hub |
| **Perception Package** | 1.0+ | Synthetic data |

## ROS 2 Humble Packages

### Core Packages

```bash
# Desktop full installation
sudo apt install ros-humble-desktop

# Navigation
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

# Visualization
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-rqt*
```

### Gazebo Integration

```bash
# Gazebo Fortress packages
sudo apt install ros-humble-ros-gz
sudo apt install ros-humble-ros-gz-bridge
sudo apt install ros-humble-ros-gz-sim
```

### Sensor Packages

```bash
# Intel RealSense
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-realsense2-description

# Generic sensors
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-depth-image-proc
```

## Python Dependencies

### Core ROS 2 Development

```txt
# requirements.txt for ROS 2 development
rclpy>=3.0.0
sensor_msgs
geometry_msgs
nav_msgs
std_msgs
tf2_ros
```

### AI/ML Dependencies

```txt
# requirements.txt for AI components
torch>=2.0.0
transformers>=4.30.0
openai-whisper>=20231117
numpy>=1.24.0
opencv-python>=4.8.0
```

## Known Compatibility Issues

### ROS 2 + Gazebo

| Issue | Versions Affected | Solution |
|-------|-------------------|----------|
| Bridge topic mismatch | ros_gz_bridge < 0.244 | Update to latest ros-humble-ros-gz |
| Camera frame rate drops | Gazebo Fortress + RTX 40xx | Set `MESA_GL_VERSION_OVERRIDE=4.5` |

### Isaac Sim + ROS 2

| Issue | Versions Affected | Solution |
|-------|-------------------|----------|
| tf2 timing issues | Isaac Sim < 2023.1.1 | Update Isaac Sim |
| VSLAM initialization | cuVSLAM 1.x | Migrate to cuVSLAM 2.0 |

### Unity + ROS 2

| Issue | Versions Affected | Solution |
|-------|-------------------|----------|
| Message serialization | ROS-TCP-Connector < 0.7 | Update connector |
| URDF import failures | Unity < 2022.3 | Use Unity 2022.3 LTS |

## Recommended Development Environment

### Minimum Specifications

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | Intel i7-10th / AMD Ryzen 7 3700X | Intel i9-13th / AMD Ryzen 9 7900X |
| RAM | 16GB DDR4 | 64GB DDR5 |
| GPU | NVIDIA RTX 3060 | NVIDIA RTX 4080+ |
| Storage | 256GB SSD | 1TB NVMe |
| OS | Ubuntu 22.04 | Ubuntu 22.04 |

### Windows Development (WSL2)

WSL2 can be used for ROS 2 and Gazebo development with limitations:

| Feature | WSL2 Support |
|---------|--------------|
| ROS 2 Humble | ✅ Full |
| Gazebo Fortress | ⚠️ Limited (software rendering) |
| Isaac Sim | ❌ Not supported |
| Unity (Windows native) | ✅ Full |

## Version Verification Commands

```bash
# ROS 2 version
ros2 --version

# Gazebo version
ign gazebo --version

# Python version
python3 --version

# CUDA version
nvcc --version

# Check ROS 2 packages
ros2 pkg list | grep -E "nav2|ros_gz"
```

## Upgrade Paths

### From ROS 2 Foxy to Humble

1. Backup workspace
2. Uninstall Foxy packages
3. Install Humble from apt
4. Rebuild workspace with `colcon build`
5. Update launch files for API changes

### From Gazebo Classic to Fortress

1. Convert `.world` files to `.sdf` format
2. Update plugin configurations
3. Replace `gazebo_ros` with `ros_gz`
4. Update launch files for `ign gazebo`

---

*Last updated: 2025-12-11*
