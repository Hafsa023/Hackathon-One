---
sidebar_position: 1
title: Installation Guide
description: Step-by-step installation instructions for ROS 2, Gazebo, Unity, and NVIDIA Isaac tools
---

# Installation Guide

This appendix provides installation instructions for all tools covered in this book. All instructions target Ubuntu 22.04 LTS unless otherwise noted.

---

## ROS 2 Humble Hawksbill

ROS 2 Humble is the Long-Term Support (LTS) release used throughout this book.

### Prerequisites

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install -y \
  software-properties-common \
  curl \
  gnupg \
  lsb-release
```

### Installation

```bash
# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Desktop (recommended)
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y ros-dev-tools
```

### Environment Setup

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
# Expected: ros2 0.9.x
```

### Additional Packages

```bash
# Navigation
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# Visualization
sudo apt install -y ros-humble-rviz2

# Simulation bridge
sudo apt install -y ros-humble-ros-gz

# Robot description
sudo apt install -y ros-humble-xacro ros-humble-joint-state-publisher-gui
```

---

## Gazebo Fortress

Gazebo Fortress is the LTS simulation platform compatible with ROS 2 Humble.

### Installation

```bash
# Install Gazebo Fortress
sudo apt install -y ros-humble-ros-gz

# This installs:
# - Gazebo Fortress (gz-sim)
# - ros_gz bridge packages
# - ros_gz_sim for spawning

# Verify installation
gz sim --version
# Expected: Gazebo Sim, version 6.x.x
```

### First Launch

```bash
# Launch empty world
gz sim empty.sdf

# Launch with ROS 2 bridge
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```

### Environment Variables

```bash
# Add to ~/.bashrc if using custom models
export GZ_SIM_RESOURCE_PATH=$HOME/gz_models:$GZ_SIM_RESOURCE_PATH
```

---

## Unity 2022.3 LTS

Unity provides high-fidelity rendering for robotics simulation.

### Installation via Unity Hub

1. Download Unity Hub from: https://unity.com/download
2. Install Unity Hub:

```bash
# Make executable and run
chmod +x UnityHub.AppImage
./UnityHub.AppImage
```

3. In Unity Hub:
   - Sign in with Unity ID
   - Go to **Installs** → **Install Editor**
   - Select **Unity 2022.3 LTS**
   - Add modules: Linux Build Support, Documentation

### Robotics Packages

Add packages via Package Manager (Window → Package Manager):

1. Click **+** → **Add package from git URL**
2. Add each URL:

```text
# ROS-TCP-Connector
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector

# URDF Importer
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer

# Perception (for synthetic data)
com.unity.perception
```

### ROS 2 Endpoint Setup

```bash
# Clone ROS-TCP-Endpoint
cd ~/ros2_ws/src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash

# Run endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

---

## NVIDIA Isaac Sim

Isaac Sim requires NVIDIA RTX GPU and provides photorealistic simulation.

### System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | RTX 3070 (8GB) | RTX 4090 (24GB) |
| CPU | 8 cores | 16+ cores |
| RAM | 32GB | 64GB |
| Storage | 50GB SSD | 100GB NVMe |
| OS | Ubuntu 22.04 | Ubuntu 22.04 |
| Driver | 535+ | Latest |

### Installation

1. **Install NVIDIA Driver**

```bash
# Add graphics drivers PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install driver (535 or newer)
sudo apt install -y nvidia-driver-535
sudo reboot
```

2. **Install Omniverse Launcher**

Download from: https://www.nvidia.com/en-us/omniverse/

```bash
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

3. **Install Isaac Sim**

- Open Omniverse Launcher
- Go to **Exchange** → Search "Isaac Sim"
- Click **Install** on Isaac Sim 2023.1.1+
- Launch from **Library**

### Headless Mode

```bash
# Run Isaac Sim headless (for servers)
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./isaac-sim.sh --headless
```

---

## NVIDIA Isaac ROS

Isaac ROS provides GPU-accelerated perception for ROS 2.

### Prerequisites

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
newgrp docker

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Installation

```bash
# Clone Isaac ROS Common
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Run development container
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh

# Inside container, build packages
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

### Available Packages

Clone additional packages as needed:

```bash
# Visual SLAM
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# AprilTag Detection
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git

# Nvblox 3D Mapping
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git

# Object Detection
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git
```

---

## Intel RealSense

RealSense cameras provide depth sensing for robotics.

### SDK Installation

```bash
# Register server public key
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
  sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# Add repository
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
  https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
  sudo tee /etc/apt/sources.list.d/librealsense.list

# Install SDK
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils

# Verify
realsense-viewer
```

### ROS 2 Wrapper

```bash
# Install ROS 2 wrapper
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description

# Launch camera
ros2 launch realsense2_camera rs_launch.py
```

---

## Python Dependencies

Common Python packages for Physical AI development:

```bash
# Create virtual environment (optional but recommended)
python3 -m venv ~/venvs/physical-ai
source ~/venvs/physical-ai/bin/activate

# Core packages
pip install numpy scipy matplotlib

# Machine learning
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Speech recognition
pip install openai-whisper

# LLM integration
pip install openai anthropic

# ROS 2 Python tools
pip install colcon-common-extensions vcstool

# Computer vision
pip install opencv-python opencv-contrib-python

# Jupyter for development
pip install jupyterlab ipywidgets
```

---

## Jetson Setup

For NVIDIA Jetson edge deployment:

### Flash JetPack

1. Download NVIDIA SDK Manager on host PC
2. Connect Jetson via USB
3. Flash JetPack 6.0+ (includes Ubuntu 22.04, CUDA, TensorRT)

### Post-Flash Setup

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 (same as desktop)
# Follow ROS 2 Humble instructions above

# Enable maximum performance
sudo nvpmodel -m 0
sudo jetson_clocks
```

---

## Verification Checklist

Run these commands to verify your installation:

```bash
# ROS 2
ros2 doctor

# Gazebo
gz sim --version

# NVIDIA GPU
nvidia-smi

# RealSense
rs-enumerate-devices

# Python environment
python3 -c "import torch; print(f'PyTorch: {torch.__version__}, CUDA: {torch.cuda.is_available()}')"
```

### Expected Output

```text
✓ ROS 2 Humble installed
✓ Gazebo Fortress 6.x running
✓ NVIDIA driver 535+ with CUDA
✓ RealSense camera detected
✓ PyTorch with CUDA support
```

---

## Troubleshooting

If you encounter issues, see the [Troubleshooting Guide](./troubleshooting.md) for common problems and solutions.
