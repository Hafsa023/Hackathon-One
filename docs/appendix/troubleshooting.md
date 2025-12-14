---
sidebar_position: 2
title: Troubleshooting Guide
description: Solutions for common issues with ROS 2, Gazebo, Unity, Isaac Sim, and hardware integration
---

# Troubleshooting Guide

This guide covers common issues encountered when working with Physical AI tools and their solutions.

---

## ROS 2 Issues

### DDS Communication Problems

**Symptom**: Nodes can't find each other, topics not visible across machines.

**Solutions**:

```bash
# Check DDS implementation
ros2 doctor --report | grep middleware

# Set same domain ID on all machines
export ROS_DOMAIN_ID=42

# For Docker containers, use host network
docker run --network host ...

# Check firewall (allow UDP 7400-7500)
sudo ufw allow 7400:7500/udp
```

### QoS Mismatch

**Symptom**: Subscriber receives no messages despite publisher running.

**Solution**:

```python
# Match QoS settings between publisher and subscriber
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Match sensor default
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

self.sub = self.create_subscription(Image, '/camera/image', callback, qos)
```

### Transform (TF) Lookup Failures

**Symptom**: `Could not find transform from X to Y`

**Solutions**:

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check if transform is being published
ros2 topic echo /tf

# Add static transform if missing
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link camera_link
```

### Package Not Found After Build

**Symptom**: `Package 'my_package' not found`

**Solution**:

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Add to ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Verify package exists
ros2 pkg list | grep my_package
```

---

## Gazebo Issues

### Gazebo Won't Start

**Symptom**: Black screen or immediate crash.

**Solutions**:

```bash
# Check GPU driver
nvidia-smi

# Use software rendering (fallback)
export LIBGL_ALWAYS_SOFTWARE=1
gz sim empty.sdf

# Clear cache
rm -rf ~/.gz/

# Check for conflicting Gazebo versions
dpkg -l | grep gazebo
```

### Model Loading Failures

**Symptom**: `Unable to find model: XXX`

**Solution**:

```bash
# Set resource path
export GZ_SIM_RESOURCE_PATH=/path/to/models:$GZ_SIM_RESOURCE_PATH

# Check model structure (must have model.sdf)
ls my_model/
# Expected: model.sdf, meshes/, materials/
```

### ros_gz Bridge Not Working

**Symptom**: No data flowing between Gazebo and ROS 2.

**Solutions**:

```bash
# Verify topic names match exactly
gz topic -l          # Gazebo topics
ros2 topic list      # ROS 2 topics

# Run bridge with explicit mapping
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan

# Check for type mismatches
ros2 run ros_gz_bridge parameter_bridge --help
```

### Physics Instability

**Symptom**: Robot explodes or behaves erratically.

**Solutions**:

```xml
<!-- Reduce step size in world file -->
<physics name="stable_physics" type="dart">
  <max_step_size>0.001</max_step_size>  <!-- 1ms -->
  <real_time_factor>1.0</real_time_factor>
</physics>
```

```python
# Check for self-collisions in URDF
# Ensure inertia values are reasonable (not zero)
```

---

## Unity Issues

### ROS Connection Failed

**Symptom**: Unity can't connect to ROS-TCP-Endpoint.

**Solutions**:

```bash
# Verify endpoint is running
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Check firewall
sudo ufw allow 10000/tcp

# In Unity, verify ROSConnection settings:
# - IP: 127.0.0.1 (localhost) or actual IP
# - Port: 10000
```

### URDF Import Errors

**Symptom**: Robot appears broken or missing parts.

**Solutions**:

```text
1. Check mesh file paths are relative
2. Convert COLLADA (.dae) to FBX if needed
3. Verify axis orientation (Unity Y-up vs ROS Z-up)
4. Check import settings: Axis Type = "Z Up"
```

### HDRP Performance Issues

**Symptom**: Low FPS in HDRP project.

**Solutions**:

```text
1. Edit → Project Settings → Quality
   - Reduce shadow resolution
   - Disable ray tracing if not needed

2. Edit → Project Settings → HDRP Global Settings
   - Lower LOD bias
   - Reduce reflection quality

3. Use baked lighting instead of real-time
```

---

## Isaac Sim Issues

### Out of GPU Memory

**Symptom**: `CUDA out of memory` error.

**Solutions**:

```bash
# Check GPU memory usage
nvidia-smi

# Reduce scene complexity
# Use smaller robot models
# Lower render resolution in settings

# Run with memory limit
./isaac-sim.sh --/renderer/gpuMemory=8192
```

### Isaac Sim Crashes on Launch

**Symptom**: Application closes immediately.

**Solutions**:

```bash
# Check driver version (need 535+)
nvidia-smi

# Clear cache
rm -rf ~/.nvidia-omniverse/

# Check logs
cat ~/.nvidia-omniverse/logs/Kit/Isaac-Sim/*/kit.log

# Run with verbose logging
./isaac-sim.sh --/log/level=verbose
```

### ROS 2 Bridge Not Found

**Symptom**: Extension `omni.isaac.ros2_bridge` not available.

**Solution**:

```bash
# Enable extension in Isaac Sim
# Window → Extensions → Search "ros2_bridge" → Enable

# Or via command line
./isaac-sim.sh --/app/extensions/enabled/0='omni.isaac.ros2_bridge'
```

---

## Isaac ROS Issues

### Container Build Fails

**Symptom**: Docker build errors.

**Solutions**:

```bash
# Ensure NVIDIA Container Toolkit installed
docker run --rm --gpus all nvidia/cuda:11.8-base-ubuntu22.04 nvidia-smi

# Clear Docker cache
docker system prune -a

# Use specific tag
./scripts/run_dev.sh -b isaac_ros_common:aarch64
```

### cuVSLAM Not Tracking

**Symptom**: Visual SLAM loses tracking frequently.

**Solutions**:

```yaml
# Tune parameters in cuvslam_params.yaml
visual_slam_node:
  ros__parameters:
    enable_imu_fusion: true          # Enable if IMU available
    image_jitter_threshold_ms: 50.0  # Increase for unstable cameras
    enable_debug_mode: true          # Enable for diagnostics
```

---

## RealSense Issues

### Camera Not Detected

**Symptom**: `rs-enumerate-devices` shows nothing.

**Solutions**:

```bash
# Check USB connection (must be USB 3.0)
lsusb | grep Intel

# Reload kernel module
sudo modprobe -r uvcvideo
sudo modprobe uvcvideo

# Check udev rules
sudo cp /opt/ros/humble/share/librealsense2/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Depth Quality Issues

**Symptom**: Noisy or missing depth data.

**Solutions**:

```python
# Enable post-processing filters
from launch_ros.actions import Node

realsense_node = Node(
    package='realsense2_camera',
    executable='realsense2_camera_node',
    parameters=[{
        'spatial_filter.enable': True,
        'temporal_filter.enable': True,
        'hole_filling_filter.enable': True,
        'decimation_filter.enable': True,
    }]
)
```

---

## Jetson Issues

### Thermal Throttling

**Symptom**: Performance drops during extended use.

**Solutions**:

```bash
# Check temperature
cat /sys/devices/virtual/thermal/thermal_zone*/temp

# Add active cooling (required for sustained workloads)

# Use power mode appropriate for thermal solution
sudo nvpmodel -m 2  # 15W mode for passive cooling
```

### Out of Memory

**Symptom**: Processes killed by OOM killer.

**Solutions**:

```bash
# Increase swap
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Add to /etc/fstab for persistence
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

---

## Network/Multi-Machine Issues

### ROS 2 Nodes on Different Machines Can't Communicate

**Solutions**:

```bash
# Use same ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42

# Disable localhost-only mode
export ROS_LOCALHOST_ONLY=0

# Check network connectivity
ping <other_machine_ip>

# For WiFi, ensure multicast works
# Some routers block multicast - use wired connection or configure router
```

### High Latency in Communication

**Solutions**:

```bash
# Use cyclonedds with faster settings
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create cyclonedds.xml
cat << EOF > ~/cyclonedds.xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
    </General>
    <Internal>
      <SocketReceiveBufferSize>10MB</SocketReceiveBufferSize>
    </Internal>
  </Domain>
</CycloneDDS>
EOF

export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

---

## General Debugging Tips

### Enable Verbose Logging

```bash
# ROS 2
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
ros2 run my_package my_node --ros-args --log-level debug

# Gazebo
GZ_DEBUG=1 gz sim world.sdf

# Isaac Sim
./isaac-sim.sh --/log/level=verbose
```

### System Resource Monitoring

```bash
# CPU and memory
htop

# GPU
watch -n 1 nvidia-smi

# Disk I/O
iotop

# Network
iftop
```

### Log Locations

| Tool | Log Location |
|------|--------------|
| ROS 2 | `~/.ros/log/` |
| Gazebo | `~/.gz/logs/` |
| Isaac Sim | `~/.nvidia-omniverse/logs/` |
| Unity | `~/.config/unity3d/Editor.log` |

---

## Getting Help

If these solutions don't resolve your issue:

1. **ROS 2**: https://answers.ros.org
2. **Gazebo**: https://answers.gazebosim.org
3. **Isaac Sim**: https://forums.developer.nvidia.com/c/omniverse/
4. **Unity Robotics**: https://github.com/Unity-Technologies/Unity-Robotics-Hub/issues
