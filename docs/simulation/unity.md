---
sidebar_position: 2
title: Unity for High-Fidelity Robotics
description: Build photorealistic simulations and digital twins using Unity's robotics toolkit, ROS 2 integration, and HDRP rendering pipeline
---

# Unity for High-Fidelity Robotics

## Learning Objectives

By the end of this chapter, you will be able to:

- Configure Unity with the Robotics Hub packages for ROS 2 integration
- Import and visualize URDF robot models in Unity
- Establish bidirectional communication between Unity and ROS 2
- Create photorealistic environments using HDRP for sim-to-real transfer
- Generate synthetic training data with domain randomization

---

## Unity Robotics Toolkit Overview

While Gazebo excels at physics accuracy, Unity provides unmatched visual fidelity—critical for training vision-based AI systems. Unity's game engine heritage delivers photorealistic rendering, while its Robotics Hub packages bridge the gap to ROS 2.

### Why Unity for Robotics?

| Capability | Gazebo | Unity |
|------------|--------|-------|
| Physics accuracy | ★★★★★ | ★★★☆☆ |
| Visual fidelity | ★★★☆☆ | ★★★★★ |
| Synthetic data | Limited | Extensive |
| Domain randomization | Manual | Built-in tools |
| Cross-platform | Linux-focused | Windows/Mac/Linux |
| Learning curve | Moderate | Steeper |

### Unity Robotics Hub Packages

Unity provides three core packages for robotics development:

1. **ROS-TCP-Connector**: Bidirectional communication with ROS 2 via TCP
2. **URDF Importer**: Converts URDF robot descriptions to Unity GameObjects
3. **Perception**: Synthetic data generation with automatic labeling

```text
Unity Project
├── ROS-TCP-Connector     # ROS 2 communication
│   └── ROSConnection     # Publisher/Subscriber API
├── URDF-Importer        # Robot model import
│   └── UrdfRobot        # Joint/Link hierarchy
└── Perception           # Training data generation
    ├── Labelers         # Bounding box, semantic
    └── Randomizers      # Domain randomization
```

![Unity Robotics Architecture](/assets/ch05/unity-robotics.svg)
*Figure 5.1: Unity Robotics Hub architecture showing integration between Unity, ROS-TCP-Connector, and ROS 2*

---

## ROS-TCP-Connector Setup

The ROS-TCP-Connector enables Unity to publish and subscribe to ROS 2 topics without requiring ROS to be installed on the Unity machine.

### Installation

```bash
# Environment: Unity 2022.3 LTS
# Install via Package Manager (Window > Package Manager)
# 1. Click '+' > Add package from git URL
# 2. Enter: https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector

# On ROS 2 side (Ubuntu 22.04, ROS 2 Humble)
cd ~/ros2_ws/src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

### Configuration

In Unity, configure the ROS connection:

1. Create an empty GameObject named "ROSConnection"
2. Add the `ROSConnection` component
3. Set ROS IP Address (default: `127.0.0.1`)
4. Set ROS Port (default: `10000`)

```csharp
// Environment: Unity 2022.3 LTS, C# 9.0
// Dependencies: ROS-TCP-Connector package
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VelocityPublisher : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/cmd_vel";
    private float publishRate = 10f; // Hz
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= 1f / publishRate)
        {
            var msg = new TwistMsg
            {
                linear = new Vector3Msg { x = 0.5, y = 0, z = 0 },
                angular = new Vector3Msg { x = 0, y = 0, z = 0.1 }
            };
            ros.Publish(topicName, msg);
            lastPublishTime = Time.time;
        }
    }
}
```

### Subscribing to ROS Topics

```csharp
// Environment: Unity 2022.3 LTS, C# 9.0
// Dependencies: ROS-TCP-Connector package
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class LaserScanSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private float[] latestRanges;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<LaserScanMsg>("/scan", OnLaserScanReceived);
    }

    void OnLaserScanReceived(LaserScanMsg msg)
    {
        latestRanges = msg.ranges;
        Debug.Log($"Received {latestRanges.Length} range readings");

        // Find closest obstacle
        float minRange = float.MaxValue;
        foreach (float range in latestRanges)
        {
            if (range > msg.range_min && range < msg.range_max)
                minRange = Mathf.Min(minRange, range);
        }
        Debug.Log($"Closest obstacle: {minRange:F2}m");
    }
}
```

Launch the ROS 2 endpoint:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

---

## URDF Importer Workflow

The URDF Importer converts standard robot description files into Unity GameObjects with proper joint hierarchies and physics.

### Import Process

1. Place URDF file and meshes in `Assets/URDF/`
2. Right-click the `.urdf` file
3. Select **Import Robot from URDF**
4. Configure import settings:
   - **Axis Type**: Y-Up (Unity) or Z-Up (ROS)
   - **Mesh Decomposition**: VHACD for convex colliders
   - **Controller Type**: Position or Velocity

```xml
<!-- Example URDF structure (compatible with Unity Importer) -->
<!-- Environment: ROS 2 Humble URDF format -->
<?xml version="1.0"?>
<robot name="simple_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="arm_link">
    <visual>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </visual>
  </link>

  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

### Controlling Imported Robots

```csharp
// Environment: Unity 2022.3 LTS, C# 9.0
// Dependencies: URDF-Importer package
using Unity.Robotics.UrdfImporter;

public class RobotController : MonoBehaviour
{
    private ArticulationBody[] joints;

    void Start()
    {
        // Get all articulation bodies (joints)
        joints = GetComponentsInChildren<ArticulationBody>();
    }

    public void SetJointTarget(int jointIndex, float targetPosition)
    {
        if (jointIndex < 0 || jointIndex >= joints.Length) return;

        var joint = joints[jointIndex];
        var drive = joint.xDrive;
        drive.target = targetPosition * Mathf.Rad2Deg;
        joint.xDrive = drive;
    }

    public float GetJointPosition(int jointIndex)
    {
        if (jointIndex < 0 || jointIndex >= joints.Length) return 0f;
        return joints[jointIndex].jointPosition[0];
    }
}
```

---

## Photorealistic Rendering with HDRP

The High Definition Render Pipeline (HDRP) delivers film-quality visuals essential for training vision-based neural networks that must transfer to the real world.

### HDRP Setup

```text
# Creating HDRP Project (Unity Hub)
1. New Project > 3D (HDRP)
2. Or upgrade existing: Window > Rendering > HDRP Wizard
```

### Key HDRP Features for Robotics

| Feature | Robotics Application |
|---------|---------------------|
| Physical lighting | Accurate indoor/outdoor illumination |
| Ray tracing | Realistic reflections and shadows |
| Post-processing | Motion blur, depth of field |
| Material system | PBR materials matching real surfaces |
| Volumetric fog | Atmospheric effects for outdoor scenes |

### Environment Configuration

```csharp
// Environment: Unity 2022.3 LTS HDRP, C# 9.0
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public class LightingController : MonoBehaviour
{
    public Light directionalLight;
    public HDAdditionalLightData hdLight;

    void Start()
    {
        hdLight = directionalLight.GetComponent<HDAdditionalLightData>();
    }

    public void SetTimeOfDay(float hour)
    {
        // Rotate sun based on time (0-24)
        float angle = (hour - 6f) * 15f; // 6 AM = 0 degrees
        directionalLight.transform.rotation = Quaternion.Euler(angle, -30f, 0f);

        // Adjust intensity for dawn/dusk
        float intensity = Mathf.Clamp01(Mathf.Sin(hour / 24f * Mathf.PI));
        hdLight.intensity = intensity * 100000f; // lux
    }
}
```

![HDRP Rendering Pipeline](/assets/ch05/hdrp-pipeline.svg)
*Figure 5.2: HDRP rendering pipeline stages from geometry to final image with post-processing*

---

## Digital Twin Synchronization

A digital twin mirrors the physical robot's state in real-time, enabling remote monitoring and predictive simulation.

### Architecture Pattern

```text
Physical Robot                  Unity Digital Twin
┌─────────────────┐            ┌─────────────────┐
│   ROS 2 Node    │            │  Unity Scene    │
│  ┌───────────┐  │            │  ┌───────────┐  │
│  │  Sensors  │──┼─── /tf ───►│  │ Transform │  │
│  │  Joints   │──┼─ /joint ──►│  │  Joints   │  │
│  │ Actuators │◄─┼─ /cmd_vel ─│  │ Controller│  │
│  └───────────┘  │            │  └───────────┘  │
└─────────────────┘            └─────────────────┘
```

### Transform Synchronization

```csharp
// Environment: Unity 2022.3 LTS, C# 9.0
// Dependencies: ROS-TCP-Connector package
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Tf2;

public class TFSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private Dictionary<string, Transform> frameMap = new();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TFMessageMsg>("/tf", OnTFReceived);

        // Map ROS frames to Unity transforms
        BuildFrameMap();
    }

    void OnTFReceived(TFMessageMsg msg)
    {
        foreach (var transform in msg.transforms)
        {
            string childFrame = transform.child_frame_id;
            if (frameMap.TryGetValue(childFrame, out Transform unityTransform))
            {
                // Convert ROS (right-hand, Z-up) to Unity (left-hand, Y-up)
                var pos = transform.transform.translation;
                var rot = transform.transform.rotation;

                unityTransform.localPosition = new Vector3(
                    (float)pos.x,
                    (float)pos.z,  // Z becomes Y
                    (float)pos.y   // Y becomes Z
                );

                unityTransform.localRotation = new Quaternion(
                    -(float)rot.x,
                    -(float)rot.z,
                    -(float)rot.y,
                    (float)rot.w
                );
            }
        }
    }

    void BuildFrameMap()
    {
        // Map ROS frame IDs to Unity GameObjects
        foreach (Transform child in GetComponentsInChildren<Transform>())
        {
            frameMap[child.name] = child;
        }
    }
}
```

![Digital Twin Workflow](/assets/ch05/digital-twin.svg)
*Figure 5.3: Digital twin synchronization workflow between physical robot and Unity simulation*

---

## Synthetic Data Generation

Unity's Perception package automates dataset generation for training computer vision models with perfect ground truth labels.

### Perception Setup

```csharp
// Environment: Unity 2022.3 LTS, C# 9.0
// Dependencies: Perception package (com.unity.perception)
using UnityEngine.Perception.GroundTruth;
using UnityEngine.Perception.Randomization;

// Configure in Inspector:
// 1. Add PerceptionCamera to main camera
// 2. Add Labelers: BoundingBox2D, SemanticSegmentation
// 3. Create IdLabelConfig asset with object labels
```

### Domain Randomization

Domain randomization varies simulation parameters to improve model generalization:

```csharp
// Environment: Unity 2022.3 LTS, C# 9.0
// Dependencies: Perception package
using UnityEngine.Perception.Randomization.Randomizers;

[Serializable]
[AddRandomizerMenu("Robotics/Lighting Randomizer")]
public class LightingRandomizer : Randomizer
{
    public FloatParameter intensity = new() { value = new UniformSampler(0.5f, 2f) };
    public ColorRgbParameter color = new();

    protected override void OnIterationStart()
    {
        var lights = tagManager.Query<LightRandomizerTag>();
        foreach (var light in lights)
        {
            light.GetComponent<Light>().intensity = intensity.Sample();
            light.GetComponent<Light>().color = color.Sample();
        }
    }
}
```

### Output Formats

The Perception package generates:

- **RGB images**: Camera renders
- **Bounding boxes**: 2D/3D object detection labels
- **Semantic segmentation**: Per-pixel class labels
- **Instance segmentation**: Per-pixel object IDs
- **Depth maps**: Z-buffer data
- **Metadata**: Camera intrinsics, object poses

```json
// Example perception output (annotations.json)
{
  "captures": [{
    "id": "frame_001",
    "filename": "rgb/frame_001.png",
    "annotations": [{
      "label_name": "robot_arm",
      "bounding_box": {
        "x": 120, "y": 80,
        "width": 200, "height": 150
      }
    }]
  }]
}
```

---

## Practical Assessment: Unity Digital Twin

### Objective

Create a Unity digital twin that visualizes a simulated robot from Gazebo in real-time.

### Requirements

1. Unity 2022.3 LTS project with ROS-TCP-Connector
2. URDF-imported robot model
3. TF listener for pose synchronization
4. Joint state subscriber for articulation control

### Steps

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch gazebo_demo simulation.launch.py

# Terminal 2: Launch ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# Terminal 3: Run Unity (Play mode)
# Open Unity project and press Play
```

### Success Criteria

- [ ] Robot appears in Unity scene
- [ ] Robot pose updates in real-time from Gazebo
- [ ] Joint positions match between simulators
- [ ] Command velocity from Unity moves Gazebo robot

---

## Summary

Unity complements Gazebo by providing:

- **Photorealistic rendering** via HDRP for vision-based AI training
- **ROS 2 integration** through ROS-TCP-Connector
- **URDF import** for consistent robot models across simulators
- **Digital twin** capabilities for real-time monitoring
- **Synthetic data generation** with automatic ground truth labeling

Together, Gazebo (physics accuracy) and Unity (visual fidelity) create a comprehensive simulation pipeline for Physical AI development.

---

## References

1. Unity Technologies. (2023). Unity Robotics Hub. https://github.com/Unity-Technologies/Unity-Robotics-Hub

2. Unity Technologies. (2023). ROS-TCP-Connector Documentation. https://github.com/Unity-Technologies/ROS-TCP-Connector

3. Unity Technologies. (2023). URDF Importer Documentation. https://github.com/Unity-Technologies/URDF-Importer

4. Unity Technologies. (2023). Perception Package Documentation. https://docs.unity3d.com/Packages/com.unity.perception@latest

5. Juliani, A., et al. (2018). Unity: A General Platform for Intelligent Agents. *arXiv preprint arXiv:1809.02627*.

6. Tremblay, J., et al. (2018). Training Deep Networks with Synthetic Data: Bridging the Reality Gap by Domain Randomization. *CVPR Workshops*, 969-977.

7. Unity Technologies. (2023). High Definition Render Pipeline Documentation. https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@latest

---

*Word count: ~1,500 words*
