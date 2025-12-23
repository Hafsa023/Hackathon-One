# Research: Physical AI Complete Guide Book

**Feature**: 001-physical-ai-book
**Date**: 2025-12-11
**Status**: Complete

## Overview

This document consolidates research findings for the Physical AI book project, resolving all technical decisions and providing authoritative references for implementation.

---

## 1. Documentation Platform

### Decision: Docusaurus 3.5+

**Rationale**: Modern static site generator with excellent MDX support, React 18 compatibility, and native GitHub Pages deployment.

**Alternatives Considered**:
| Option | Pros | Cons | Verdict |
|--------|------|------|---------|
| Docusaurus 3.x | MDX 3.0, React 18, active development | Newer, fewer tutorials | ✅ Selected |
| Docusaurus 2.x | Stable, many examples | Legacy, MDX 2 only | ❌ Rejected |
| GitBook | Easy setup | Vendor lock-in, less customization | ❌ Rejected |
| MkDocs | Python ecosystem | Limited interactivity | ❌ Rejected |

**Key References**:
- Docusaurus Documentation: https://docusaurus.io/docs
- MDX 3.0 Specification: https://mdxjs.com/

---

## 2. ROS 2 Distribution

### Decision: ROS 2 Humble Hawksbill (LTS)

**Rationale**: Long-term support until May 2027, widest third-party package compatibility, official Ubuntu 22.04 support.

**Alternatives Considered**:
| Distribution | Release | EOL | Verdict |
|--------------|---------|-----|---------|
| Humble | May 2022 | May 2027 | ✅ Selected (LTS) |
| Iron | May 2023 | Nov 2024 | ❌ Short support |
| Jazzy | May 2024 | Nov 2025 | ❌ Too new |

**Key Topics for Book**:
1. DDS middleware architecture (FastDDS default)
2. Node lifecycle management
3. rclpy client library for Python examples
4. Launch system (Python-based)
5. URDF/Xacro robot description
6. tf2 transforms
7. Quality of Service (QoS) policies

**Key References**:
- Macenski, S., et al. (2022). "Robot Operating System 2: Design, Architecture, and Uses in the Wild." *Science Robotics*.
- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/

---

## 3. Physics Simulation

### Decision: Gazebo Fortress (LTS)

**Rationale**: Ignition-based Gazebo with official ROS 2 Humble support, LTS until September 2026.

**Alternatives Considered**:
| Simulator | ROS 2 Support | Physics Engine | Verdict |
|-----------|---------------|----------------|---------|
| Gazebo Fortress | Official | DART, Bullet | ✅ Selected |
| Gazebo Garden | Official | DART, Bullet | ❌ Less mature |
| Gazebo Classic 11 | Bridge only | ODE | ❌ Deprecated |

**Integration Components**:
- `ros_gz_bridge`: Topic/service bridging
- `ros_gz_sim`: World management, entity spawning
- `ros_gz_image`: Camera streaming

**Sensor Plugins**:
- `gpu_lidar`: GPU-accelerated LiDAR
- `depth_camera`: RGB-D sensing
- `imu`: Inertial measurement
- `contact_sensor`: Collision detection

**Key References**:
- Koenig, N., & Howard, A. (2004). "Design and Use Paradigms for Gazebo." *IEEE/RSJ IROS*.
- Gazebo Fortress Docs: https://gazebosim.org/docs/fortress

---

## 4. High-Fidelity Rendering

### Decision: Unity 2022.3 LTS + Unity Robotics Hub

**Rationale**: Production-stable LTS with maintained robotics packages, strong synthetic data capabilities.

**Key Packages**:
| Package | Purpose |
|---------|---------|
| ROS-TCP-Connector | ROS 2 communication |
| URDF Importer | Robot model loading |
| Perception | Synthetic data generation |
| HDRP | Photorealistic rendering |

**Workflow**:
```
URDF → Unity Importer → Physics Setup → HDRP Materials → ROS Bridge
```

**Key References**:
- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- Juliani, A., et al. (2018). "Unity: A General Platform for Intelligent Agents." *arXiv:1809.02627*.

---

## 5. NVIDIA Isaac Platform

### Decision: Isaac Sim 2023.1+ with Isaac ROS 2.0+

**Rationale**: Production-ready GPU-accelerated robotics with tight ROS 2 integration.

**Components**:
| Component | Purpose |
|-----------|---------|
| Isaac Sim | Omniverse-based simulation |
| Isaac ROS | GPU-accelerated perception |
| Isaac Perceptor | Multi-camera 3D perception |
| cuVSLAM | GPU visual SLAM |

**Hardware Requirements**:
- **Minimum**: NVIDIA RTX 3070, 32GB RAM, Ubuntu 22.04
- **Recommended**: NVIDIA RTX 4080+, 64GB RAM, NVMe SSD
- **Note**: WSL2 has limited Isaac Sim support

**Nav2 Integration**:
```
Isaac ROS Perception → costmap → Nav2 Planner → Isaac ROS Controller
```

**Key References**:
- NVIDIA Isaac ROS: https://nvidia-isaac-ros.github.io/
- Macenski, S., et al. (2020). "The Marathon 2: A Navigation System." *arXiv:2003.00368* (Nav2).

---

## 6. Vision-Language-Action Systems

### Decision: Cover VLA concepts with practical LLM integration

**Rationale**: Emerging research area with increasing production relevance; focus on implementable patterns.

**Key Architectures**:
| Model | Type | Availability |
|-------|------|--------------|
| RT-2 | VLA Transformer | Research (Google) |
| PaLM-E | Multimodal LLM | Research (Google) |
| OpenVLA | Open VLA | Open source |

**Practical Integration Pattern**:
```
Voice → Whisper → LLM (GPT-4/Claude) → Task Plan → ROS 2 Actions
```

**Key References**:
- Brohan, A., et al. (2023). "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control." *arXiv:2307.15818*.
- Driess, D., et al. (2023). "PaLM-E: An Embodied Multimodal Language Model." *ICML 2023*.

---

## 7. Voice Integration

### Decision: OpenAI Whisper (open-source, local deployment)

**Rationale**: State-of-the-art accuracy, MIT license, local inference capability.

**Models**:
| Model | Parameters | Speed | Accuracy |
|-------|------------|-------|----------|
| tiny | 39M | ~32x real-time | Good |
| base | 74M | ~16x real-time | Better |
| small | 244M | ~6x real-time | Great |
| medium | 769M | ~2x real-time | Excellent |

**Integration**:
```python
import whisper
model = whisper.load_model("small")
result = model.transcribe("audio.wav")
```

**Key References**:
- Radford, A., et al. (2022). "Robust Speech Recognition via Large-Scale Weak Supervision." *arXiv:2212.04356*.
- Whisper GitHub: https://github.com/openai/whisper

---

## 8. Sim-to-Real Transfer

### Decision: Domain randomization + progressive transfer

**Rationale**: Established technique for bridging simulation-reality gap.

**Techniques**:
1. **Visual Domain Randomization**: Textures, lighting, colors
2. **Physics Randomization**: Friction, mass, joint dynamics
3. **Sensor Noise Injection**: Gaussian noise, dropout
4. **Progressive Transfer**: Sim → Simplified Real → Full Real

**Key References**:
- Tobin, J., et al. (2017). "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World." *IROS 2017*.
- Peng, X., et al. (2018). "Sim-to-Real Robot Learning from Pixels with Progressive Nets." *CoRL 2017*.

---

## 9. Hardware Recommendations

### Development Workstation

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | Intel i7-10th / AMD Ryzen 7 | Intel i9-13th / AMD Ryzen 9 |
| GPU | NVIDIA RTX 3070 | NVIDIA RTX 4080+ |
| RAM | 32GB DDR4 | 64GB DDR5 |
| Storage | 512GB NVMe | 2TB NVMe |
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |

### Edge Deployment

| Device | Use Case | Notes |
|--------|----------|-------|
| Jetson Orin Nano | Entry-level inference | 8GB, 40 TOPS |
| Jetson Orin NX | Production inference | 16GB, 100 TOPS |
| Jetson AGX Orin | Full autonomy | 64GB, 275 TOPS |

### Sensors

| Sensor | Model | Interface |
|--------|-------|-----------|
| Depth Camera | Intel RealSense D435i | USB 3.0 |
| LiDAR | Velodyne VLP-16 / Ouster OS1 | Ethernet |
| IMU | Integrated (RealSense) | USB |

---

## 10. Citation Master List

### Foundational Papers

1. Brooks, R. A. (1991). "Intelligence without Representation." *Artificial Intelligence*, 47(1-3), 139-159.
2. Pfeifer, R., & Bongard, J. (2006). *How the Body Shapes the Way We Think*. MIT Press.

### ROS & Navigation

3. Macenski, S., et al. (2022). "Robot Operating System 2: Design, Architecture, and Uses in the Wild." *Science Robotics*, 7(66).
4. Macenski, S., et al. (2020). "The Marathon 2: A Navigation System." *arXiv:2003.00368*.

### Simulation

5. Koenig, N., & Howard, A. (2004). "Design and Use Paradigms for Gazebo, An Open-Source Multi-Robot Simulator." *IEEE/RSJ IROS*.
6. Juliani, A., et al. (2018). "Unity: A General Platform for Intelligent Agents." *arXiv:1809.02627*.

### Transfer Learning

7. Tobin, J., et al. (2017). "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World." *IROS 2017*.
8. Peng, X., et al. (2018). "Sim-to-Real Robot Learning from Pixels with Progressive Nets." *CoRL 2017*.

### Vision-Language-Action

9. Brohan, A., et al. (2023). "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control." *arXiv:2307.15818*.
10. Driess, D., et al. (2023). "PaLM-E: An Embodied Multimodal Language Model." *ICML 2023*.

### Voice & Language

11. Radford, A., et al. (2022). "Robust Speech Recognition via Large-Scale Weak Supervision." *arXiv:2212.04356*.

---

## Version Compatibility Matrix

| Component | Version | Tested With | Notes |
|-----------|---------|-------------|-------|
| Ubuntu | 22.04 LTS | All components | Primary OS |
| ROS 2 | Humble | Gazebo Fortress, Isaac ROS 2.0 | LTS |
| Gazebo | Fortress | ROS 2 Humble | via ros_gz |
| Unity | 2022.3 LTS | ROS-TCP-Connector 0.7+ | Robotics Hub |
| Isaac Sim | 2023.1+ | Isaac ROS 2.0+, ROS 2 Humble | Omniverse |
| Python | 3.10 | Ubuntu 22.04 default | rclpy |
| CUDA | 12.0+ | Isaac ROS, Isaac Sim | Driver 525+ |

---

## Unresolved Items

None. All technical decisions have been made with documented rationale.

---

*Research completed: 2025-12-11*
