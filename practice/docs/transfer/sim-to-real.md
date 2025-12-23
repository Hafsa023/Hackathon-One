---
sidebar_position: 1
title: Sim-to-Real Strategies
description: Bridge the reality gap with domain randomization, transfer learning, and systematic validation workflows
---

# Sim-to-Real Strategies

## Learning Objectives

By the end of this chapter, you will be able to:

- Apply domain randomization to improve policy generalization
- Implement transfer learning for perception and control
- Identify and mitigate common reality gap challenges
- Design validation workflows for sim-to-real deployment
- Measure and reduce the simulation-to-reality performance gap

---

## The Reality Gap Problem

Models trained purely in simulation often fail when deployed on physical robots. This "reality gap" stems from systematic differences between simulated and real environments.

### Sources of the Reality Gap

| Domain | Simulation | Reality | Gap Impact |
|--------|------------|---------|------------|
| Physics | Idealized contacts | Complex friction | Control failures |
| Visuals | Perfect lighting | Variable conditions | Perception errors |
| Sensors | Clean data | Noise, calibration drift | State estimation |
| Dynamics | Exact parameters | Manufacturing variance | Motion errors |
| Timing | Deterministic | Latency, jitter | Control instability |

### Gap Measurement

```python
# Environment: Python 3.10
# Measuring sim-to-real gap for a navigation task

import numpy as np
from dataclasses import dataclass

@dataclass
class TransferMetrics:
    """Metrics for quantifying sim-to-real transfer."""
    sim_success_rate: float  # Success in simulation
    real_success_rate: float  # Success on hardware
    sim_completion_time: float  # Average time (sim)
    real_completion_time: float  # Average time (real)

    @property
    def success_gap(self) -> float:
        """Performance drop from sim to real (0 = perfect transfer)."""
        return self.sim_success_rate - self.real_success_rate

    @property
    def time_gap_ratio(self) -> float:
        """Time increase ratio (1.0 = no slowdown)."""
        return self.real_completion_time / self.sim_completion_time

    def report(self):
        print(f"Simulation Success: {self.sim_success_rate:.1%}")
        print(f"Real-world Success: {self.real_success_rate:.1%}")
        print(f"Success Gap: {self.success_gap:.1%}")
        print(f"Time Gap Ratio: {self.time_gap_ratio:.2f}x")

# Example: Measuring navigation transfer
metrics = TransferMetrics(
    sim_success_rate=0.95,
    real_success_rate=0.72,
    sim_completion_time=45.0,
    real_completion_time=68.0
)
metrics.report()
# Output:
# Simulation Success: 95.0%
# Real-world Success: 72.0%
# Success Gap: 23.0%
# Time Gap Ratio: 1.51x
```

![Sim-to-Real Workflow](/assets/ch08/sim-to-real-workflow.svg)
*Figure 8.1: Sim-to-real transfer workflow showing the iterative process of simulation training, real-world testing, and gap analysis*

---

## Domain Randomization Techniques

Domain randomization trains models on varied simulation conditions, forcing them to learn robust features that transfer to reality.

### Visual Randomization

Randomize visual properties to create perception models robust to lighting and appearance changes:

```python
# Environment: Isaac Sim 2023.1+, Omniverse Replicator
# Visual domain randomization for RGB camera training

import omni.replicator.core as rep

def setup_visual_randomization():
    """Configure visual domain randomization."""

    # Randomize lighting
    with rep.trigger.on_frame():
        # Sun position and intensity
        rep.modify.pose(
            input_prims=rep.get.prims(path_pattern="/World/Sun"),
            rotation=rep.distribution.uniform(
                (0, -180, 0), (90, 180, 0)
            )
        )

        # Light color temperature
        rep.modify.attribute(
            input_prims=rep.get.prims(path_pattern="/World/Light*"),
            attribute_name="intensity",
            value=rep.distribution.uniform(500, 2000)
        )

    # Randomize textures
    with rep.trigger.on_frame():
        materials = rep.get.prims(
            path_pattern="/World/Environment/*/material"
        )
        rep.randomizer.materials(
            materials,
            diffuse=rep.distribution.uniform((0.1, 0.1, 0.1), (0.9, 0.9, 0.9))
        )

    # Randomize camera properties
    with rep.trigger.on_frame():
        rep.modify.attribute(
            input_prims=rep.get.prims(path_pattern="/World/Camera"),
            attribute_name="focalLength",
            value=rep.distribution.normal(35, 2)
        )

    return rep.orchestrator.preview()
```

### Physics Randomization

Randomize physical parameters to handle real-world manufacturing variance:

```python
# Environment: Isaac Sim 2023.1+, Python 3.10
# Physics domain randomization for robust control

from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.prims import RigidPrim
import numpy as np

class PhysicsRandomizer:
    """Randomize physics parameters for sim-to-real transfer."""

    def __init__(self, robot_path: str):
        self.robot_path = robot_path
        self.nominal_mass = {}
        self.nominal_friction = {}
        self._cache_nominal_values()

    def _cache_nominal_values(self):
        """Store original physics parameters."""
        # Cache would be populated from USD stage
        pass

    def randomize_mass(self, variation: float = 0.1):
        """
        Randomize link masses within ±variation percentage.

        Args:
            variation: Fractional variation (0.1 = ±10%)
        """
        for link_name, nominal in self.nominal_mass.items():
            scale = np.random.uniform(1 - variation, 1 + variation)
            new_mass = nominal * scale
            # Apply to simulation
            self._set_link_mass(link_name, new_mass)

    def randomize_friction(self, variation: float = 0.2):
        """Randomize contact friction coefficients."""
        for surface, nominal in self.nominal_friction.items():
            scale = np.random.uniform(1 - variation, 1 + variation)
            new_friction = np.clip(nominal * scale, 0.1, 2.0)
            self._set_friction(surface, new_friction)

    def randomize_actuator_dynamics(self):
        """Add realistic actuator modeling."""
        # Randomize motor parameters
        params = {
            'damping': np.random.uniform(0.8, 1.2),
            'friction': np.random.uniform(0.01, 0.05),
            'delay_ms': np.random.uniform(5, 20),
        }
        return params

    def apply_all(self):
        """Apply comprehensive physics randomization."""
        self.randomize_mass(variation=0.15)
        self.randomize_friction(variation=0.25)
        actuator_params = self.randomize_actuator_dynamics()
        return actuator_params
```

### Sensor Noise Injection

Add realistic sensor noise to train robust state estimators:

```python
# Environment: ROS 2 Humble, Python 3.10
# Sensor noise injection for sim-to-real transfer

import numpy as np
from dataclasses import dataclass

@dataclass
class SensorNoiseModel:
    """Realistic sensor noise parameters."""
    # IMU noise (from datasheet specifications)
    gyro_noise_density: float = 0.000244  # rad/s/√Hz
    accel_noise_density: float = 0.000144  # m/s²/√Hz
    gyro_bias_instability: float = 0.00003  # rad/s
    accel_bias_instability: float = 0.00004  # m/s²

    # Depth camera noise
    depth_noise_std: float = 0.005  # meters
    depth_quantization: float = 0.001  # meters

    # LiDAR noise
    lidar_range_noise: float = 0.02  # meters
    lidar_dropout_rate: float = 0.01  # probability

def inject_imu_noise(
    angular_velocity: np.ndarray,
    linear_acceleration: np.ndarray,
    noise_model: SensorNoiseModel,
    dt: float
) -> tuple:
    """
    Inject realistic IMU noise.

    Args:
        angular_velocity: Clean gyro reading [rad/s]
        linear_acceleration: Clean accel reading [m/s²]
        noise_model: Noise parameters
        dt: Time step

    Returns:
        Noisy (angular_velocity, linear_acceleration)
    """
    # White noise
    gyro_noise = np.random.normal(
        0, noise_model.gyro_noise_density / np.sqrt(dt), 3
    )
    accel_noise = np.random.normal(
        0, noise_model.accel_noise_density / np.sqrt(dt), 3
    )

    # Bias random walk (simplified)
    gyro_bias = np.random.normal(0, noise_model.gyro_bias_instability, 3)
    accel_bias = np.random.normal(0, noise_model.accel_bias_instability, 3)

    noisy_gyro = angular_velocity + gyro_noise + gyro_bias
    noisy_accel = linear_acceleration + accel_noise + accel_bias

    return noisy_gyro, noisy_accel

def inject_depth_noise(
    depth_image: np.ndarray,
    noise_model: SensorNoiseModel
) -> np.ndarray:
    """Inject realistic depth camera noise."""
    # Gaussian noise
    noise = np.random.normal(0, noise_model.depth_noise_std, depth_image.shape)

    # Quantization
    noisy_depth = depth_image + noise
    noisy_depth = np.round(
        noisy_depth / noise_model.depth_quantization
    ) * noise_model.depth_quantization

    # Dropout at edges and far range (simplified)
    dropout_mask = np.random.random(depth_image.shape) < 0.02
    noisy_depth[dropout_mask] = 0

    return noisy_depth
```

![Domain Randomization](/assets/ch08/domain-randomization.svg)
*Figure 8.2: Domain randomization varies visual, physics, and sensor parameters to train robust policies*

---

## Transfer Learning Approaches

Transfer learning leverages pre-trained models or simulation experience to accelerate real-world learning.

### Perception Transfer

Pre-train vision models on synthetic data, fine-tune on real images:

```python
# Environment: Python 3.10, PyTorch 2.0+
# Transfer learning for object detection

import torch
import torch.nn as nn
from torchvision.models.detection import fasterrcnn_resnet50_fpn_v2
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor

def create_transfer_detector(num_classes: int, pretrain_path: str = None):
    """
    Create object detector with transfer learning.

    Args:
        num_classes: Number of object classes + background
        pretrain_path: Path to simulation-pretrained weights

    Returns:
        Fine-tunable detection model
    """
    # Load model with ImageNet backbone
    model = fasterrcnn_resnet50_fpn_v2(weights="DEFAULT")

    # Load simulation pre-training if available
    if pretrain_path:
        state_dict = torch.load(pretrain_path)
        model.load_state_dict(state_dict, strict=False)
        print(f"Loaded sim-pretrained weights from {pretrain_path}")

    # Replace classifier head for fine-tuning
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(
        in_features, num_classes
    )

    # Freeze backbone for initial fine-tuning
    for param in model.backbone.parameters():
        param.requires_grad = False

    return model

def progressive_unfreeze(model, epoch: int):
    """Progressively unfreeze layers during training."""
    if epoch >= 5:
        # Unfreeze backbone layer4
        for param in model.backbone.body.layer4.parameters():
            param.requires_grad = True
    if epoch >= 10:
        # Unfreeze layer3
        for param in model.backbone.body.layer3.parameters():
            param.requires_grad = True
```

### Policy Transfer

Transfer control policies from simulation using domain adaptation:

```text
Simulation Policy Training:
1. Train policy π_sim with domain randomization
2. Collect experience: D_sim = {(s, a, r, s')}
3. Learn robust features via randomization

Real-World Adaptation:
1. Deploy π_sim on hardware (zero-shot)
2. Collect real experience: D_real = {(s, a, r, s')}
3. Fine-tune: π_real = adapt(π_sim, D_real)

Techniques:
- System identification: Match sim parameters to real
- Residual policy: π_real = π_sim + Δπ (learn correction)
- Online adaptation: Update policy during deployment
```

![Transfer Learning Pipeline](/assets/ch08/transfer-learning.svg)
*Figure 8.3: Transfer learning pipeline from simulation pre-training to real-world fine-tuning*

---

## Validation and Testing Workflows

Systematic validation ensures safe, reliable sim-to-real transfer.

### Validation Ladder

```text
Level 1: Unit Tests (Simulation)
├── Individual component tests
├── Sensor processing validation
└── Control response verification

Level 2: Integration Tests (Simulation)
├── Full pipeline tests
├── Edge case scenarios
└── Failure mode injection

Level 3: Hardware-in-Loop (HIL)
├── Real sensors, simulated robot
├── Real actuators, simulated environment
└── Timing validation

Level 4: Controlled Real-World
├── Structured test environment
├── Safety monitors active
└── Limited operating envelope

Level 5: Field Deployment
├── Full autonomy
├── Remote monitoring
└── Graceful degradation
```

### Automated Testing Framework

```python
# Environment: Python 3.10, pytest
# Sim-to-real validation test framework

import pytest
from dataclasses import dataclass
from typing import Callable, List
import numpy as np

@dataclass
class TransferTest:
    """Definition of a sim-to-real transfer test."""
    name: str
    sim_test: Callable  # Simulation test function
    real_test: Callable  # Real-world test function
    max_gap: float  # Maximum acceptable performance gap
    metric: str  # Metric name (e.g., "success_rate", "rmse")

class SimToRealValidator:
    """Validate sim-to-real transfer quality."""

    def __init__(self):
        self.tests: List[TransferTest] = []
        self.results = {}

    def add_test(self, test: TransferTest):
        self.tests.append(test)

    def run_simulation_tests(self) -> dict:
        """Run all tests in simulation."""
        sim_results = {}
        for test in self.tests:
            result = test.sim_test()
            sim_results[test.name] = result
            print(f"[SIM] {test.name}: {result:.3f}")
        return sim_results

    def run_real_tests(self) -> dict:
        """Run all tests on hardware."""
        real_results = {}
        for test in self.tests:
            result = test.real_test()
            real_results[test.name] = result
            print(f"[REAL] {test.name}: {result:.3f}")
        return real_results

    def validate_transfer(
        self,
        sim_results: dict,
        real_results: dict
    ) -> bool:
        """Check if transfer meets requirements."""
        all_passed = True

        for test in self.tests:
            sim_val = sim_results[test.name]
            real_val = real_results[test.name]
            gap = abs(sim_val - real_val)

            passed = gap <= test.max_gap
            status = "PASS" if passed else "FAIL"

            print(f"{test.name}: gap={gap:.3f}, max={test.max_gap} [{status}]")

            if not passed:
                all_passed = False

        return all_passed

# Example usage
validator = SimToRealValidator()
validator.add_test(TransferTest(
    name="navigation_success",
    sim_test=lambda: 0.95,  # Placeholder
    real_test=lambda: 0.85,  # Placeholder
    max_gap=0.15,
    metric="success_rate"
))
```

### Safety Monitoring

```python
# Environment: ROS 2 Humble, Python 3.10
# Safety monitor for sim-to-real deployment

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class SafetyMonitor(Node):
    """Monitor robot behavior and enforce safety limits."""

    def __init__(self):
        super().__init__('safety_monitor')

        # Safety parameters
        self.max_linear_vel = 1.0  # m/s
        self.max_angular_vel = 1.5  # rad/s
        self.emergency_stop = False

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_callback, 10
        )

        # Publishers
        self.safe_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        self.get_logger().info('Safety monitor active')

    def cmd_callback(self, msg: Twist):
        if self.emergency_stop:
            # Send zero velocity
            self.safe_cmd_pub.publish(Twist())
            return

        # Clamp velocities to safe limits
        safe_cmd = Twist()
        safe_cmd.linear.x = np.clip(
            msg.linear.x, -self.max_linear_vel, self.max_linear_vel
        )
        safe_cmd.angular.z = np.clip(
            msg.angular.z, -self.max_angular_vel, self.max_angular_vel
        )

        # Check for anomalous commands
        if abs(msg.linear.x) > 2 * self.max_linear_vel:
            self.get_logger().warn('Anomalous velocity command detected!')
            self.trigger_estop()
            return

        self.safe_cmd_pub.publish(safe_cmd)

    def trigger_estop(self):
        self.emergency_stop = True
        self.estop_pub.publish(Bool(data=True))
        self.get_logger().error('EMERGENCY STOP TRIGGERED')
```

---

## Summary

Successful sim-to-real transfer requires systematic approaches:

- **Domain randomization**: Train on varied conditions to learn robust features
- **Transfer learning**: Leverage simulation pre-training, fine-tune on real data
- **Reality gap analysis**: Measure and track performance differences
- **Validation workflows**: Progress through simulation → HIL → controlled → field
- **Safety systems**: Monitor behavior and enforce limits during deployment

The gap between simulation and reality continues to narrow as tools improve, but careful engineering remains essential for reliable Physical AI systems.

---

## References

1. Tobin, J., et al. (2017). Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. *IEEE/RSJ IROS*, 23-30.

2. Peng, X. B., et al. (2018). Sim-to-Real Robot Learning from Pixels with Progressive Nets. *CoRL*.

3. OpenAI. (2019). Solving Rubik's Cube with a Robot Hand. https://openai.com/research/solving-rubiks-cube

4. Sadeghi, F., & Levine, S. (2017). CAD2RL: Real Single-Image Flight without a Single Real Image. *RSS*.

5. Tan, J., et al. (2018). Sim-to-Real: Learning Agile Locomotion For Quadruped Robots. *RSS*.

6. Muratore, F., et al. (2022). Robot Learning from Randomized Simulations: A Review. *Frontiers in Robotics and AI*.

---

*Word count: ~1,400 words*
