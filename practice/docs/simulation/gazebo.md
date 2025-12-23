---
sidebar_position: 1
title: Gazebo Simulation Essentials
description: Master physics simulation, sensor modeling, and ROS 2 integration with Gazebo Fortress for realistic robot testing
---

# Gazebo Simulation Essentials

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain physics simulation fundamentals and their role in robotics
- Configure Gazebo Fortress with ROS 2 using ros_gz bridge
- Simulate common sensors: LiDAR, depth cameras, and IMUs
- Build custom worlds and environments for robot testing
- Integrate sensor data into ROS 2 applications

---

## Physics Simulation Fundamentals

Physics simulation creates a virtual environment where robots can be tested without physical hardware. This enables rapid prototyping, safe failure exploration, and massive parallelization for training AI systems.

### Why Simulate?

| Benefit | Physical Testing | Simulation |
|---------|------------------|------------|
| Cost per test | $100s-$1000s | ~$0.01 |
| Risk of damage | High | None |
| Parallelization | Limited | 1000s of instances |
| Edge case testing | Dangerous | Safe |
| Reproducibility | Difficult | Exact replay |

#### Core Simulation Components

A robotics simulator consists of four key subsystems:

1. **Physics Engine**: Computes rigid body dynamics, collisions, and contact forces
2. **Rendering Engine**: Visualizes the scene for debugging and synthetic data generation
3. **Sensor Models**: Simulate cameras, LiDAR, IMUs with realistic noise
4. **Plugin System**: Extends functionality with custom controllers and interfaces

![Gazebo Architecture](/assets/ch04/gazebo-architecture.svg)
*Figure 4.1: Gazebo Fortress modular architecture with physics, rendering, sensors, and plugin subsystems connected via ros_gz bridge to ROS 2*

---

## Gazebo Fortress Architecture

Gazebo Fortress (Ignition Gazebo 6.x) is the Long-Term Support release used with ROS 2 Humble. It represents a complete rewrite from "Gazebo Classic" with improved modularity and performance.

### Key Differences from Gazebo Classic

| Feature | Gazebo Classic | Gazebo Fortress |
|---------|----------------|-----------------|
| Architecture | Monolithic | Plugin-based |
| Physics | ODE default | DART, Bullet, TPE |
| Format | URDF + SDF | SDF preferred |
| ROS bridge | gazebo_ros | ros_gz |
| GUI framework | Qt | Ogre2 + ImGui |

### Installation with ROS 2 Humble

```bash
# Environment: Ubuntu 22.04, ROS 2 Humble
# Install Gazebo Fortress and ROS integration
sudo apt install ros-humble-ros-gz

# Verify installation
gz sim --version
# Expected: Gazebo Sim, version 6.x.x
```

### The ros_gz Bridge

The `ros_gz` package bridges Gazebo topics to ROS 2. It translates Gazebo's internal transport (ign-transport) to ROS 2 DDS:

```yaml title="bridge_config.yaml"
# Environment: ROS 2 Humble, Gazebo Fortress
# Bridge configuration for sensor topics

- ros_topic_name: "scan"
  gz_topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "cmd_vel"
  gz_topic_name: "/model/robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "imu/data"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
```

Launch the bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config.yaml
```

![ros_gz Bridge Architecture](/assets/ch04/ros-gz-bridge.svg)
*Figure 4.2: The ros_gz bridge translates between Gazebo's ign-transport and ROS 2 DDS, supporting bidirectional communication for sensors and commands*

---

## Sensor Simulation

Accurate sensor models are critical for sim-to-real transfer. Gazebo provides configurable noise models that approximate real sensor behavior.

### LiDAR Sensors

```xml title="lidar_sensor.sdf"
<!-- Environment: Gazebo Fortress SDF 1.9 -->
<sensor name="gpu_lidar" type="gpu_lidar">
  <topic>/lidar</topic>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>
        <max_angle>0.261799</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>
    </noise>
  </lidar>
  <visualize>true</visualize>
</sensor>
```

### Depth Cameras

```xml title="depth_camera.sdf"
<!-- Environment: Gazebo Fortress SDF 1.9 -->
<sensor name="depth_camera" type="depth_camera">
  <topic>/depth_camera</topic>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>
    </noise>
  </camera>
</sensor>
```

### IMU Sensors

```xml title="imu_sensor.sdf"
<!-- Environment: Gazebo Fortress SDF 1.9 -->
<sensor name="imu" type="imu">
  <topic>/imu</topic>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.0002</stddev>
        </noise>
      </x>
      <!-- Similar for y, z -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.017</stddev>
        </noise>
      </x>
      <!-- Similar for y, z -->
    </linear_acceleration>
  </imu>
</sensor>
```

![Sensor Pipeline](/assets/ch04/sensor-pipeline.svg)
*Figure 4.3: Sensor data flows from Gazebo simulation through ros_gz bridge to ROS 2 applications, with commands flowing in reverse*

---

## World Building

Gazebo worlds define the environment where robots operate. SDF (Simulation Description Format) provides a comprehensive XML schema.

### Basic World Structure

```xml title="warehouse_world.sdf"
<?xml version="1.0" ?>
<!-- Environment: Gazebo Fortress SDF 1.9 -->
<sdf version="1.9">
  <world name="warehouse">
    <!-- Physics configuration -->
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Warehouse shelves -->
    <include>
      <uri>model://shelf</uri>
      <pose>2 0 0 0 0 0</pose>
    </include>

    <!-- Robot spawn point -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.1 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### Launch File Integration

```python title="gazebo_launch.py"
# Environment: ROS 2 Humble, Python 3.10+
# Dependencies: launch_ros, ros_gz

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Start Gazebo with custom world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r warehouse_world.sdf'
        }.items()
    )

    # Bridge sensor topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    return LaunchDescription([gz_sim, bridge])
```

---

## ROS 2 Integration Example

Here's a complete example subscribing to simulated sensor data:

```python title="sensor_subscriber.py"
# Environment: ROS 2 Humble, Python 3.10+
# Dependencies: rclpy, sensor_msgs

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')

        # Best-effort QoS for sensor data (matches Gazebo default)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            sensor_qos
        )

        self.get_logger().info('Sensor subscriber initialized')

    def lidar_callback(self, msg):
        # Find minimum distance (closest obstacle)
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            min_dist = min(valid_ranges)
            self.get_logger().info(f'Closest obstacle: {min_dist:.2f}m')

    def imu_callback(self, msg):
        # Log angular velocity around z-axis (yaw rate)
        yaw_rate = msg.angular_velocity.z
        self.get_logger().debug(f'Yaw rate: {yaw_rate:.3f} rad/s')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Practical Assessment: Gazebo World Project

### Objective

Create a Gazebo simulation with a mobile robot, sensors, and obstacles for navigation testing.

### Requirements

1. World file with ground plane and 3+ obstacles
2. Robot model with LiDAR and IMU sensors
3. ros_gz bridge configuration
4. ROS 2 node that logs sensor data

### Steps

```bash
# Create package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python gazebo_demo

# Add world file to package
mkdir -p gazebo_demo/worlds
# Create warehouse_world.sdf

# Build and launch
cd ~/ros2_ws
colcon build --packages-select gazebo_demo
source install/setup.bash
ros2 launch gazebo_demo simulation.launch.py
```

### Success Criteria

- [ ] Gazebo launches with custom world
- [ ] Robot appears at spawn point
- [ ] LiDAR data visible in RViz
- [ ] IMU data logging at 100 Hz

---

## Summary

Gazebo Fortress provides the physics simulation foundation for Physical AI:

- **Physics engines** (DART, Bullet) compute realistic dynamics
- **Sensor models** with noise approximate real hardware
- **ros_gz bridge** connects simulation to ROS 2 applications
- **SDF worlds** define test environments

The next chapter explores Unity for high-fidelity rendering—essential for training vision-based AI systems that must transfer to the real world.

---

## References

1. Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 2149-2154. https://doi.org/10.1109/IROS.2004.1389727

2. Open Robotics. (2022). Gazebo Fortress Documentation. https://gazebosim.org/docs/fortress

3. Open Source Robotics Foundation. (2022). SDFormat Specification 1.9. http://sdformat.org/spec

4. Open Robotics. (2022). ros_gz Documentation. https://github.com/gazebosim/ros_gz

5. Sherman, M. A., Seth, A., & Delp, S. L. (2011). Simbody: multibody dynamics for biomedical research. *Procedia IUTAM*, 2, 241-261.

6. Todorov, E., Erez, T., & Tassa, Y. (2012). MuJoCo: A physics engine for model-based control. *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 5026-5033.

---

*Word count: ~1,400 words*
