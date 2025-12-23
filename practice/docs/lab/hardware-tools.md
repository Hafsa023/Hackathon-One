---
sidebar_position: 1
title: PhysicalAI Lab
description: Configure a complete Physical AI development environment with workstations, edge devices, sensors, and robot platforms
---

# PhysicalAI Lab: Hardware, Tools & Architecture

## Learning Objectives

By the end of this chapter, you will be able to:

- Specify workstation hardware for simulation and training
- Select appropriate GPUs for different AI workloads
- Configure NVIDIA Jetson for edge deployment
- Integrate Intel RealSense depth sensors with ROS 2
- Evaluate robot platforms for Physical AI development
- Design cloud-hybrid lab architectures

---

## Digital Twin Workstation Specifications

A Physical AI development workstation must handle three concurrent workloads: simulation rendering, neural network training, and real-time inference testing.

### Workload Requirements

| Workload | CPU | GPU | RAM | Storage |
|----------|-----|-----|-----|---------|
| Isaac Sim | 8+ cores | RTX 3070+ | 32GB | NVMe |
| Model Training | 16+ cores | RTX 4090 | 64GB | 2TB+ |
| ROS 2 Development | 4+ cores | Any | 16GB | SSD |
| Edge Testing | 2+ cores | - | 8GB | 256GB |

### Recommended Configurations

**Entry-Level Development Station**
```text
CPU: AMD Ryzen 7 7800X3D (8 cores, 16 threads)
GPU: NVIDIA RTX 4070 Super (16GB VRAM)
RAM: 32GB DDR5-6000
Storage: 1TB NVMe Gen4 + 2TB HDD
OS: Ubuntu 22.04 LTS
Cost: ~$2,000
```

**Professional Workstation**
```text
CPU: AMD Threadripper PRO 5975WX (32 cores)
GPU: NVIDIA RTX 4090 (24GB VRAM)
RAM: 128GB DDR5 ECC
Storage: 2TB NVMe RAID + 8TB HDD
OS: Ubuntu 22.04 LTS
Cost: ~$8,000
```

**Multi-GPU Training Rig**
```text
CPU: AMD EPYC 9454 (48 cores)
GPU: 2x NVIDIA RTX 4090 or 1x A100 80GB
RAM: 256GB DDR5 ECC
Storage: 4TB NVMe RAID + 20TB NAS
OS: Ubuntu 22.04 LTS
Cost: ~$25,000+
```

![Workstation Architecture](/assets/ch07/workstation-arch.svg)
*Figure 7.1: Physical AI workstation architecture showing compute, storage, and connectivity requirements*

---

## GPU Selection Guide

GPU selection significantly impacts development velocity. Different tasks have different requirements.

### GPU Comparison for Robotics

| GPU | VRAM | FP32 TFLOPS | Tensor TFLOPS | Isaac Sim | Training | Price |
|-----|------|-------------|---------------|-----------|----------|-------|
| RTX 3070 | 8GB | 20.3 | 163 | Basic | Small | $500 |
| RTX 4070 Super | 16GB | 35.5 | 284 | Good | Medium | $600 |
| RTX 4080 | 16GB | 48.7 | 390 | Good | Medium | $1,000 |
| RTX 4090 | 24GB | 82.6 | 661 | Excellent | Large | $1,600 |
| A100 40GB | 40GB | 19.5 | 312 | N/A | Very Large | $10,000 |
| H100 | 80GB | 51.0 | 1,979 | N/A | Massive | $30,000 |

### Selection Criteria

```python
# GPU selection decision tree (pseudocode)
def select_gpu(use_case):
    if use_case == "isaac_sim_development":
        # Need RT cores for ray tracing
        return "RTX 4070 Super or better"

    elif use_case == "model_training":
        if model_size < "1B parameters":
            return "RTX 4090 (24GB)"
        elif model_size < "7B parameters":
            return "A100 40GB or 2x RTX 4090"
        else:
            return "H100 80GB or cloud"

    elif use_case == "edge_inference":
        return "Jetson Orin (see next section)"

    elif use_case == "multi_robot_sim":
        if robot_count <= 10:
            return "RTX 4090"
        else:
            return "Multi-GPU or cloud rendering"
```

### VRAM Requirements by Task

| Task | Minimum VRAM | Recommended |
|------|--------------|-------------|
| Isaac Sim (single robot) | 8GB | 16GB |
| Isaac Sim (warehouse, 10 robots) | 16GB | 24GB |
| Perception models (YOLO, etc.) | 4GB | 8GB |
| VLA model inference | 16GB | 24GB |
| Fine-tuning 7B LLM | 24GB | 40GB+ |
| Training from scratch | 40GB+ | 80GB+ |

---

## Jetson Orin Edge Deployment

NVIDIA Jetson Orin brings datacenter AI performance to edge robotics. It runs Isaac ROS natively.

### Jetson Orin Lineup

| Model | GPU Cores | CPU | RAM | AI TOPS | Power | Price |
|-------|-----------|-----|-----|---------|-------|-------|
| Orin Nano | 1024 | 6-core A78 | 8GB | 40 | 7-15W | $200 |
| Orin NX | 1024 | 8-core A78 | 16GB | 100 | 10-25W | $400 |
| AGX Orin 32GB | 2048 | 12-core A78 | 32GB | 200 | 15-40W | $1,000 |
| AGX Orin 64GB | 2048 | 12-core A78 | 64GB | 275 | 15-60W | $1,600 |

### Jetson Setup for ROS 2

```bash
# Environment: Jetson Orin (JetPack 6.0+), Ubuntu 22.04
# Flash JetPack using SDK Manager on host

# Install ROS 2 Humble
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop

# Install Isaac ROS (pre-built for Jetson)
sudo apt install -y nvidia-isaac-ros-common
```

### Power Management

```python
# Environment: Jetson Orin, Python 3.10
# Configure power mode programmatically

import subprocess

def set_jetson_power_mode(mode: str):
    """
    Set Jetson power mode.

    Modes:
    - "MAXN": Maximum performance (60W on AGX)
    - "30W": Balanced (30W)
    - "15W": Power saving (15W)
    """
    mode_map = {
        "MAXN": 0,
        "30W": 1,
        "15W": 2
    }

    if mode not in mode_map:
        raise ValueError(f"Unknown mode: {mode}")

    subprocess.run(
        ["sudo", "nvpmodel", "-m", str(mode_map[mode])],
        check=True
    )
    print(f"Power mode set to {mode}")

# Example: Set maximum performance for training
set_jetson_power_mode("MAXN")
```

![Edge Deployment](/assets/ch07/edge-deployment.svg)
*Figure 7.2: Jetson Orin edge deployment architecture with sensor inputs and ROS 2 integration*

---

## Intel RealSense Integration

Intel RealSense depth cameras provide affordable, accurate depth sensing for robotics applications.

### RealSense Camera Comparison

| Model | Depth Tech | Range | Resolution | FPS | Use Case |
|-------|------------|-------|------------|-----|----------|
| D435i | Stereo IR | 0.3-3m | 1280x720 | 90 | Indoor navigation |
| D455 | Stereo IR | 0.6-6m | 1280x720 | 90 | Outdoor, longer range |
| D405 | Stereo IR | 0.07-0.5m | 1280x720 | 90 | Close-range manipulation |
| L515 | LiDAR | 0.25-9m | 1024x768 | 30 | High precision mapping |

### ROS 2 Installation

```bash
# Environment: Ubuntu 22.04, ROS 2 Humble

# Install librealsense2
sudo apt install -y ros-humble-librealsense2*
sudo apt install -y ros-humble-realsense2-camera
sudo apt install -y ros-humble-realsense2-description

# Add udev rules (required for USB access)
sudo cp /opt/ros/humble/share/librealsense2/config/99-realsense-libusb.rules \
  /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify camera detection
rs-enumerate-devices
```

### Launch Configuration

```python
# Environment: ROS 2 Humble, Python 3.10
# realsense_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': True,
                'enable_accel': True,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'pointcloud.enable': True,
                'align_depth.enable': True,
            }],
            output='screen'
        )
    ])
```

### Depth Processing Example

```python
# Environment: ROS 2 Humble, Python 3.10
# Dependencies: rclpy, sensor_msgs, cv_bridge, numpy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        self.bridge = CvBridge()

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.get_logger().info('Depth processor initialized')

    def depth_callback(self, msg):
        # Convert to numpy array (depth in millimeters)
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        depth_meters = depth_image.astype(np.float32) / 1000.0

        # Calculate statistics
        valid_mask = depth_meters > 0
        if np.any(valid_mask):
            min_depth = np.min(depth_meters[valid_mask])
            mean_depth = np.mean(depth_meters[valid_mask])
            self.get_logger().info(
                f'Depth - Min: {min_depth:.2f}m, Mean: {mean_depth:.2f}m'
            )

def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

![Sensor Integration](/assets/ch07/sensor-integration.svg)
*Figure 7.3: RealSense sensor integration showing depth, color, and IMU data flow to ROS 2*

---

## Robot Platforms

Selecting the right robot platform depends on research goals, budget, and deployment environment.

### Humanoid and Quadruped Options

| Platform | Type | DOF | Sensors | SDK | Price |
|----------|------|-----|---------|-----|-------|
| Unitree Go2 | Quadruped | 12 | LiDAR, Cameras, IMU | ROS 2 | $1,600 |
| Unitree H1 | Humanoid | 19 | Cameras, IMU | ROS 2 | $90,000 |
| Boston Dynamics Spot | Quadruped | 12 | Cameras, LiDAR | Python SDK | $75,000 |
| Agility Digit | Humanoid | 30+ | LiDAR, Cameras | ROS | Contact |
| Open-source (Custom) | Various | Variable | Custom | ROS 2 | $5,000+ |

### Unitree Go2 ROS 2 Setup

```bash
# Environment: Ubuntu 22.04, ROS 2 Humble
# Unitree Go2 EDU version required for SDK access

# Clone Unitree ROS 2 package
cd ~/ros2_ws/src
git clone https://github.com/unitreerobotics/unitree_ros2.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select unitree_ros2
source install/setup.bash

# Connect to Go2 (WiFi)
# Default IP: 192.168.123.161
ros2 launch unitree_ros2 go2_bringup.launch.py
```

---

## Cloud vs Local Lab Architecture

Hybrid architectures balance cost, latency, and scalability.

### Architecture Comparison

| Aspect | Local Only | Cloud Only | Hybrid |
|--------|------------|------------|--------|
| Latency | &lt;1ms | 50-200ms | Varies |
| Cost (initial) | High | Low | Medium |
| Cost (ongoing) | Low | Variable | Medium |
| Scalability | Limited | Unlimited | Good |
| Data privacy | Full control | Depends | Configurable |
| Real-time control | Yes | Difficult | Local only |

### Recommended Hybrid Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                        LOCAL LAB                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │  Workstation │  │   Jetson     │  │    Robot     │          │
│  │  (Training)  │  │   (Edge)     │  │  (Hardware)  │          │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘          │
│         └─────────────────┼─────────────────┘                   │
│                    Local Network (1Gbps+)                        │
└────────────────────────────┬────────────────────────────────────┘
                             │
                        VPN / Secure
                             │
┌────────────────────────────┴────────────────────────────────────┐
│                         CLOUD                                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │   Storage    │  │  Large Model │  │  Monitoring  │          │
│  │   (S3/GCS)   │  │   Training   │  │  Dashboard   │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└─────────────────────────────────────────────────────────────────┘
```

### When to Use Cloud

- **Large model training**: >24GB VRAM required
- **Dataset storage**: Multi-TB datasets
- **Parallel experiments**: Many hyperparameter sweeps
- **Collaboration**: Team across locations

### When to Stay Local

- **Real-time control**: Robot operation
- **Sensor data collection**: High-bandwidth streams
- **Privacy-sensitive**: Proprietary robot designs
- **Iteration speed**: Rapid prototyping

---

## Summary

Building a Physical AI lab requires balancing compute power, cost, and deployment constraints:

- **Workstations**: RTX 4090 provides excellent value for simulation and training
- **Edge compute**: Jetson Orin brings datacenter AI to robot platforms
- **Sensors**: RealSense cameras offer affordable depth perception with ROS 2 support
- **Platforms**: Unitree robots provide accessible entry to quadruped/humanoid development
- **Architecture**: Hybrid cloud-local setups optimize for both training and real-time control

The next chapter covers bridging the gap between your simulated models and physical hardware.

---

## References

1. NVIDIA Corporation. (2023). Jetson Orin Series Documentation. https://developer.nvidia.com/embedded/jetson-orin

2. Intel Corporation. (2023). Intel RealSense SDK 2.0 Documentation. https://dev.intelrealsense.com/docs

3. Unitree Robotics. (2023). Unitree Go2 Technical Specifications. https://www.unitree.com/go2

4. NVIDIA Corporation. (2023). CUDA Toolkit Documentation. https://docs.nvidia.com/cuda/

5. Open Robotics. (2023). ROS 2 Humble Hawksbill Documentation. https://docs.ros.org/en/humble/

6. Intel Corporation. (2023). RealSense ROS 2 Wrapper. https://github.com/IntelRealSense/realsense-ros

---

*Word count: ~1,500 words*
