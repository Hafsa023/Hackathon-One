---
sidebar_position: 3
title: Glossary
description: Terminology reference for Physical AI concepts, robotics, and simulation
---

# Glossary

This glossary defines the canonical terms used throughout the Physical AI book. Use these definitions for consistent terminology across all chapters.

## A

### Action (ROS 2)
A ROS 2 communication pattern for long-running tasks that provides feedback during execution and can be preempted. Actions consist of a goal, result, and feedback messages.

### Agent
In the context of Physical AI, an autonomous system that perceives its environment and takes actions to achieve goals. Distinguished from passive systems by its ability to make decisions.

### Articulation Body
Unity's physics component for simulating robotic joints with motor-driven motion, used when importing URDF models.

## B

### Behavior Tree
A hierarchical decision-making structure used in Nav2 for orchestrating complex robot behaviors. Consists of action, condition, and control nodes.

## C

### Coordinate Frame
A reference system used to describe positions and orientations in 3D space. In robotics, multiple coordinate frames (e.g., world, base_link, sensor frames) are used and transformed between using tf2.

## D

### DDS (Data Distribution Service)
The middleware standard used by ROS 2 for inter-process communication. Provides publish-subscribe messaging with configurable Quality of Service (QoS) policies.

### Digital Twin
A virtual representation of a physical system that mirrors its real-world counterpart in real-time. Used for simulation, testing, and monitoring.

### Domain Randomization
A sim-to-real transfer technique that varies visual and physical parameters during simulation training to improve generalization to real-world conditions.

## E

### Edge Computing
Processing data locally on embedded devices (e.g., Jetson Orin) near sensors rather than in the cloud, enabling low-latency real-time robotics.

### Embodied AI
Artificial intelligence systems that have a physical presence (body) in the world and interact with their environment through sensors and actuators. Contrasted with disembodied AI (e.g., chatbots).

### End Effector
The device at the end of a robotic arm that interacts with the environment (e.g., gripper, tool, sensor).

### ESDF (Euclidean Signed Distance Field)
A 3D representation of space where each voxel stores its distance to the nearest obstacle surface, used by Nvblox for navigation.

## F

### Fine-tuning
Adapting a pre-trained model to a new task or domain by training on a smaller, task-specific dataset.

## G

### Gazebo
An open-source robotics simulator that provides physics simulation, sensor modeling, and 3D visualization. Gazebo Fortress is the LTS version compatible with ROS 2 Humble.

## H

### HDRP (High Definition Render Pipeline)
Unity's advanced rendering pipeline that enables photorealistic graphics for synthetic data generation and visual simulation.

### Humanoid Robot
A robot with a human-like body structure, typically including a torso, head, two arms, and two legs. Examples include Boston Dynamics Atlas and Unitree H1.

## I

### IMU (Inertial Measurement Unit)
A sensor that measures acceleration and angular velocity, used for orientation estimation and motion tracking.

### Isaac ROS
NVIDIA's GPU-accelerated ROS 2 packages for perception, localization, and navigation. Includes modules for VSLAM, object detection, and path planning.

### Isaac Sim
NVIDIA's Omniverse-based robotics simulator providing photorealistic rendering and physics simulation with native ROS 2 support.

## J

### Jetson
NVIDIA's line of embedded AI computing platforms (e.g., Jetson Orin) for edge robotics deployment, providing GPU acceleration in compact form factors.

### JetPack
NVIDIA's SDK for Jetson platforms, including Ubuntu, CUDA, TensorRT, and other AI libraries.

## L

### LiDAR (Light Detection and Ranging)
A sensor that uses laser pulses to measure distances, creating 3D point cloud representations of the environment.

### LLM (Large Language Model)
A neural network trained on vast text data capable of generating human-like text. In Physical AI, LLMs are used for task planning and natural language command interpretation.

## M

### Message (ROS 2)
A data structure used for communication in ROS 2. Messages are defined using `.msg` files and compiled into language-specific code.

## N

### Nav2
The ROS 2 Navigation Stack, providing autonomous navigation capabilities including path planning, obstacle avoidance, and behavior trees.

### Node (ROS 2)
The fundamental unit of computation in ROS 2. Nodes are processes that communicate through topics, services, and actions.

### Nvblox
NVIDIA's GPU-accelerated library for 3D reconstruction and ESDF mapping, integrated with Isaac ROS for real-time navigation.

## O

### Omniverse
NVIDIA's platform for 3D simulation and collaboration, providing the foundation for Isaac Sim with USD (Universal Scene Description) support.

## P

### Perception
The process of interpreting sensor data to understand the environment. Includes tasks like object detection, semantic segmentation, and depth estimation.

### Physical AI
The field combining robotics, simulation, and artificial intelligence to create intelligent systems that interact with the physical world.

### PhysX
NVIDIA's physics simulation engine used in Isaac Sim and Unity for realistic rigid body dynamics, collisions, and articulated robot simulation.

### Point Cloud
A set of 3D points representing the surface of objects, typically generated by LiDAR or depth cameras.

## Q

### QoS (Quality of Service)
ROS 2 configuration settings that control communication reliability, durability, and timing. Critical for real-time robotics applications.

## R

### rclpy
The Python client library for ROS 2, providing APIs for creating nodes, publishers, subscribers, services, and actions.

### Reality Gap
The difference between simulation and real-world behavior that can cause policies trained in simulation to fail on physical robots.

### RealSense
Intel's line of depth cameras (e.g., D435, D455) commonly used in robotics for 3D perception and SLAM.

### Replicator
NVIDIA's tool in Isaac Sim for generating synthetic training data with automatic labeling and domain randomization.

### ROS 2 (Robot Operating System 2)
An open-source framework for robot software development, providing communication infrastructure, tools, and libraries. ROS 2 Humble is the LTS release (2022-2027).

### ROS-TCP-Connector
Unity package that enables bidirectional communication between Unity and ROS 2 via TCP.

### RT Cores
NVIDIA GPU hardware units dedicated to ray tracing acceleration, enabling photorealistic rendering in Isaac Sim.

## S

### SDF (Simulation Description Format)
An XML format for describing robots and environments in Gazebo, providing more features than URDF for simulation purposes.

### Sensorimotor Loop
The continuous cycle of sensing, processing, and acting that forms the basis of embodied intelligence.

### Service (ROS 2)
A ROS 2 communication pattern for request-response interactions. Used for synchronous operations where a node requests data or action from another node.

### Sim-to-Real
The process of transferring models, policies, or behaviors trained in simulation to real-world robots.

### Synthetic Data
Training data generated from simulation rather than collected from the real world. Used to reduce data collection costs and enable perfect ground-truth labeling.

## T

### TensorRT
NVIDIA's deep learning inference optimizer and runtime for deploying neural networks with reduced latency on NVIDIA GPUs.

### TOPS (Tera Operations Per Second)
A measure of AI compute performance, commonly used to compare edge devices like Jetson.

### tf2
The ROS 2 transform library that maintains the relationship between coordinate frames over time. Essential for sensor fusion and robot kinematics.

### Topic (ROS 2)
A named bus for message exchange in ROS 2. Publishers send messages to topics, and subscribers receive them. Follows the publish-subscribe pattern.

### Transfer Learning
The technique of applying knowledge learned in one domain (e.g., simulation) to another domain (e.g., real world).

## U

### URDF (Unified Robot Description Format)
An XML format for describing robot structure, including links, joints, visual geometry, and collision geometry.

### USD (Universal Scene Description)
Pixar's open-source format for 3D scene interchange, used by NVIDIA Omniverse and Isaac Sim.

## V

### VLA (Vision-Language-Action)
Models that combine visual perception, language understanding, and action generation for robot control. Examples include RT-2 and PaLM-E.

### VSLAM (Visual Simultaneous Localization and Mapping)
A technique that uses camera images to build a map of the environment while simultaneously tracking the robot's position within it.

## W

### Whisper
OpenAI's automatic speech recognition model used for transcribing voice commands in Physical AI applications.

### World Frame
The global coordinate frame in a robotics system, typically fixed to the environment. All other frames are defined relative to the world frame.

---

## Version Information

| Term | Version | Notes |
|------|---------|-------|
| ROS 2 | Humble Hawksbill | LTS until May 2027 |
| Gazebo | Fortress | LTS until Sep 2026 |
| Isaac Sim | 2023.1+ | Annual releases |
| Unity | 2022.3 LTS | Robotics Hub compatible |

---

*Last updated: 2025-12-12*
