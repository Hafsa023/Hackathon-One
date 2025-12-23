---
sidebar_position: 1
title: ROS 2 Fundamentals
description: Master ROS 2 architecture, nodes, topics, services, and actions with hands-on Python examples for robotics development
---

# ROS 2 Fundamentals

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain ROS 2's architecture and the role of DDS middleware
- Create publisher and subscriber nodes using rclpy
- Implement services and actions for request-response patterns
- Design and visualize robot models using URDF
- Build a working publisher-subscriber communication pair

---

## ROS 2 Architecture and Design Philosophy

The Robot Operating System 2 (ROS 2) is not an operating system but a middleware framework that provides communication infrastructure, tools, and libraries for building robot applications. ROS 2 Humble Hawksbill, the Long-Term Support (LTS) release, runs from May 2022 to May 2027.

### Why ROS 2?

ROS 2 addresses limitations of ROS 1 while maintaining its strengths:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | Custom protocol | DDS standard |
| Real-time | Limited | Supported |
| Security | None built-in | DDS Security |
| Multi-robot | Difficult | Native support |
| Platforms | Linux only | Linux, Windows, macOS |

### Core Design Principles

1. **Distributed by default**: No central master node; systems can span multiple machines
2. **Quality of Service**: Configurable reliability, durability, and timing guarantees
3. **Lifecycle management**: Nodes have defined states for controlled startup/shutdown
4. **Component-based**: Compose applications from reusable building blocks

---

## DDS Middleware

ROS 2 uses the Data Distribution Service (DDS) standard for communication. DDS provides:

- **Publish-subscribe messaging**: Decoupled, asynchronous communication
- **Discovery**: Automatic node and topic detection without central coordination
- **QoS policies**: Fine-grained control over message delivery

```
┌──────────────────────────────────────────────────────────┐
│                    ROS 2 Application                     │
│                                                          │
│     ┌──────────┐   ┌──────────┐   ┌──────────┐          │
│     │  Node A  │   │  Node B  │   │  Node C  │          │
│     └────┬─────┘   └────┬─────┘   └────┬─────┘          │
│          │              │              │                 │
└──────────┼──────────────┼──────────────┼─────────────────┘
           │              │              │
           └──────────────┼──────────────┘
                          │
┌──────────────────────────────────────────────────────────┐
│                 RCL (ROS Client Library)                 │
│               rmw (ROS Middleware Interface)             │
├──────────────────────────────────────────────────────────┤
│                    DDS Implementation                    │
│              (Fast DDS, Cyclone DDS, etc.)               │
└──────────────────────────────────────────────────────────┘
```

The default DDS implementation in ROS 2 Humble is **Fast DDS** by eProsima. You can switch implementations via the `RMW_IMPLEMENTATION` environment variable.

---

## Nodes, Topics, Services, and Actions

ROS 2 provides four communication patterns:

### Topics: Publish-Subscribe

Topics enable one-to-many, asynchronous communication. Publishers send messages to named topics; subscribers receive them.

```python title="minimal_publisher.py"
# Environment: ROS 2 Humble, Python 3.10+
# Dependencies: rclpy, std_msgs

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python title="minimal_subscriber.py"
# Environment: ROS 2 Humble, Python 3.10+
# Dependencies: rclpy, std_msgs

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Services: Request-Response

Services provide synchronous, one-to-one communication for request-response patterns.

```python title="service_example.py"
# Environment: ROS 2 Humble, Python 3.10+
# Service server example

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response
```

### Actions: Long-Running Tasks

Actions handle long-running tasks with feedback and preemption:

- **Goal**: What the action should achieve
- **Feedback**: Progress updates during execution
- **Result**: Final outcome when complete

![ROS 2 Communication Patterns](/assets/ch03/node-communication.svg)
*Figure 3.1: ROS 2 provides topics, services, and actions for different communication needs*

---

## rclpy Programming Basics

The `rclpy` library is the Python client for ROS 2. Key patterns:

### Node Lifecycle

```python
import rclpy
from rclpy.node import Node

def main():
    # 1. Initialize ROS 2
    rclpy.init()

    # 2. Create node
    node = Node('my_node')

    # 3. Spin (process callbacks)
    rclpy.spin(node)

    # 4. Cleanup
    node.destroy_node()
    rclpy.shutdown()
```

### Quality of Service

QoS policies control communication reliability:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Reliable delivery (like TCP)
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

# Best-effort delivery (like UDP)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5
)
```

:::tip QoS Matching
Publishers and subscribers must have compatible QoS settings to communicate. Use `ros2 topic info -v <topic>` to debug QoS mismatches.
:::

---

## Agent-to-Controller Bridges

In Physical AI systems, high-level AI agents (perception, planning) need to communicate with low-level controllers. ROS 2 provides the bridge:

```
┌───────────────────┐        Topics        ┌───────────────────┐
│                   │                      │                   │
│     AI Agent      │  ────────────────►   │    Controller     │
│     (Python)      │       /cmd_vel       │      (C++)        │
│                   │  ◄────────────────   │                   │
│                   │        /odom         │                   │
│                   │                      │                   │
└───────────────────┘                      └───────────────────┘
```

**Common message types:**

| Message | Package | Use Case |
|---------|---------|----------|
| `Twist` | geometry_msgs | Velocity commands |
| `Odometry` | nav_msgs | Position/velocity state |
| `JointState` | sensor_msgs | Joint positions/velocities |
| `Image` | sensor_msgs | Camera frames |

---

## Humanoid Robot URDF Design

The Unified Robot Description Format (URDF) describes robot structure in XML. A humanoid robot model includes:

### Basic URDF Structure

```xml title="humanoid_robot.urdf"
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0"
               iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <!-- Neck joint connecting torso to head -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Additional links and joints for limbs... -->
</robot>
```

### Visualizing in RViz

```bash
# Launch RViz with URDF display
ros2 launch urdf_tutorial display.launch.py model:=humanoid_robot.urdf
```

![URDF Humanoid](/assets/ch03/humanoid-urdf.svg)
*Figure 3.2: URDF defines robot structure through links (rigid bodies) and joints (connections)*

---

## Practical Assessment: ROS 2 Package Project

### Objective

Build a ROS 2 package with a publisher-subscriber pair that demonstrates bidirectional communication.

### Requirements

1. Create a package named `physical_ai_demo`
2. Implement a publisher node that sends sensor data
3. Implement a subscriber node that processes and responds
4. Use custom message types

### Steps

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python physical_ai_demo

# Build
cd ~/ros2_ws
colcon build --packages-select physical_ai_demo
source install/setup.bash

# Run
ros2 run physical_ai_demo publisher_node
ros2 run physical_ai_demo subscriber_node
```

### Success Criteria

- [ ] Package builds without errors
- [ ] Publisher sends messages at 10 Hz
- [ ] Subscriber receives and logs all messages
- [ ] Communication works across two terminals

---

## Summary

ROS 2 provides the communication backbone for Physical AI systems:

- **DDS middleware** enables reliable, distributed communication
- **Topics** for streaming sensor data (pub-sub pattern)
- **Services** for synchronous requests (request-response)
- **Actions** for long-running tasks with feedback
- **URDF** describes robot structure for visualization and simulation

With ROS 2 fundamentals established, the next chapter explores physics simulation with Gazebo—creating virtual environments to test robot behaviors safely before physical deployment.

---

## References

1. Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. *Science Robotics*, 7(66), eabm6074. https://doi.org/10.1126/scirobotics.abm6074

2. Open Robotics. (2022). ROS 2 Documentation: Humble Hawksbill. https://docs.ros.org/en/humble/

3. Object Management Group. (2015). Data Distribution Service (DDS) Specification, Version 1.4. https://www.omg.org/spec/DDS/

4. Quigley, M., et al. (2009). ROS: an open-source Robot Operating System. *ICRA Workshop on Open Source Software*.

5. eProsima. (2022). Fast DDS Documentation. https://fast-dds.docs.eprosima.com/

6. Open Robotics. (2022). URDF Specification. http://wiki.ros.org/urdf/XML

---

*Word count: ~1,450 words*
