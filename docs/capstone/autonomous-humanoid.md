---
sidebar_position: 1
title: Capstone - Designing the Autonomous Humanoid
description: Build a complete voice-controlled autonomous humanoid using Vision-Language-Action models, Whisper speech recognition, and ROS 2
---

# Capstone: Designing the Autonomous Humanoid

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand Vision-Language-Action (VLA) model architectures
- Integrate Whisper speech-to-text for voice commands
- Build an LLM-based task planner for robot actions
- Connect language understanding to ROS 2 action execution
- Deploy a complete voice-controlled autonomous system

---

## Vision-Language-Action Architecture Overview

Vision-Language-Action (VLA) models represent the cutting edge of embodied AI, combining visual perception, language understanding, and action generation in unified architectures.

### The VLA Paradigm

Traditional robotics separates perception, planning, and control into distinct modules. VLA models learn end-to-end mappings from multimodal inputs to robot actions:

```text
┌──────────────────────────────────────────────────────────────────┐
│                     VLA Model Architecture                       │
│                                                                  │
│    ┌────────────┐    ┌────────────┐    ┌────────────┐           │
│    │   Vision   │    │  Language  │    │   Robot    │           │
│    │  Encoder   │    │  Encoder   │    │   State    │           │
│    └─────┬──────┘    └─────┬──────┘    └─────┬──────┘           │
│          │                 │                 │                   │
│          └─────────────────┼─────────────────┘                   │
│                            │                                     │
│                    ┌───────┴───────┐                             │
│                    │   Multimodal  │                             │
│                    │  Transformer  │                             │
│                    └───────┬───────┘                             │
│                            │                                     │
│                    ┌───────┴───────┐                             │
│                    │    Action     │                             │
│                    │    Decoder    │                             │
│                    └───────┬───────┘                             │
│                            │                                     │
│                            ▼                                     │
│                    ┌───────────────┐                             │
│                    │ Robot Action  │                             │
│                    │  (dx,dy,dθ)   │                             │
│                    └───────────────┘                             │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

### Key Capabilities

| Capability | Description | Example |
|------------|-------------|---------|
| Visual grounding | Link language to scene regions | "Pick up the red cup" |
| Task decomposition | Break commands into steps | "Make coffee" → [grind, pour, brew] |
| Spatial reasoning | Understand 3D relationships | "Put it to the left of the bowl" |
| Common sense | Apply world knowledge | Know cups contain liquid |

![VLA Architecture](/assets/ch09/vla-architecture.svg)
*Figure 9.1: Vision-Language-Action architecture showing multimodal fusion and action generation*

---

## VLA Research Background

Several landmark models have advanced VLA capabilities:

### RT-2: Robotics Transformer 2

RT-2 (Google DeepMind, 2023) fine-tunes vision-language models on robot demonstration data:

- **Architecture**: PaLI-X (55B) or PaLM-E (12B) backbone
- **Training**: 130K robot episodes + web-scale VL data
- **Output**: Discretized action tokens (position, rotation, gripper)
- **Key insight**: Web knowledge transfers to robotics

### PaLM-E: Embodied Multimodal Language Model

PaLM-E (Google, 2023) integrates continuous sensor data directly into language model context:

- **Architecture**: 562B parameter PaLM + ViT-22B
- **Inputs**: Images, robot state, language interleaved
- **Outputs**: Language, plans, or low-level actions
- **Key insight**: Scale enables emergent embodied reasoning

### OpenVLA: Open-Source Alternative

OpenVLA provides accessible VLA capabilities:

```python
# Environment: Python 3.10, PyTorch 2.0+
# OpenVLA inference example (conceptual)

from openvla import OpenVLAModel

# Load pre-trained model
model = OpenVLAModel.from_pretrained("openvla/openvla-7b")

# Prepare inputs
image = camera.get_frame()  # RGB image
instruction = "Pick up the apple and place it in the bowl"
robot_state = robot.get_state()  # Joint positions, gripper state

# Generate action
action = model.predict(
    image=image,
    instruction=instruction,
    robot_state=robot_state
)

# Execute action
robot.execute(action)  # [dx, dy, dz, droll, dpitch, dyaw, gripper]
```

---

## Whisper Voice-to-Action Pipeline

OpenAI's Whisper provides state-of-the-art speech recognition for voice-controlled robotics.

### Whisper Integration

```python
# Environment: Python 3.10, ROS 2 Humble
# Dependencies: openai-whisper, pyaudio, rclpy

import whisper
import pyaudio
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WhisperTranscriptionNode(Node):
    """Real-time speech transcription for robot commands."""

    def __init__(self):
        super().__init__('whisper_transcription')

        # Load Whisper model (use 'base' for speed, 'large' for accuracy)
        self.model = whisper.load_model("base")
        self.get_logger().info('Whisper model loaded')

        # Audio configuration
        self.sample_rate = 16000
        self.chunk_duration = 3.0  # seconds
        self.chunk_samples = int(self.sample_rate * self.chunk_duration)

        # Publisher for transcribed commands
        self.command_pub = self.create_publisher(
            String, '/voice_command', 10
        )

        # Initialize audio
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_samples
        )

        # Processing timer
        self.timer = self.create_timer(
            self.chunk_duration, self.process_audio
        )

        self.get_logger().info('Voice transcription ready')

    def process_audio(self):
        """Capture and transcribe audio chunk."""
        # Read audio
        audio_data = self.stream.read(self.chunk_samples)
        audio_np = np.frombuffer(audio_data, dtype=np.float32)

        # Check for voice activity (simple energy threshold)
        energy = np.sqrt(np.mean(audio_np ** 2))
        if energy < 0.01:
            return  # Silence, skip transcription

        # Transcribe
        result = self.model.transcribe(
            audio_np,
            language='en',
            fp16=False  # Use fp32 for CPU
        )

        text = result['text'].strip()
        if text:
            self.get_logger().info(f'Transcribed: "{text}"')
            msg = String()
            msg.data = text
            self.command_pub.publish(msg)

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WhisperTranscriptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

![Whisper Pipeline](/assets/ch09/whisper-pipeline.svg)
*Figure 9.2: Whisper speech recognition pipeline from microphone to ROS 2 command topic*

---

## LLM Planning to ROS 2 Action Sequences

An LLM converts natural language commands into executable robot action sequences.

### Task Planner Architecture

```python
# Environment: Python 3.10, ROS 2 Humble
# Dependencies: openai, rclpy

import openai
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class LLMTaskPlanner(Node):
    """Convert natural language to robot action plans."""

    def __init__(self):
        super().__init__('llm_task_planner')

        # Available robot capabilities (used in prompt)
        self.capabilities = """
        Available actions:
        - navigate_to(location): Move robot to named location
        - pick_object(object_name): Pick up an object
        - place_object(location): Place held object at location
        - say(message): Speak a message
        - wait(seconds): Pause execution

        Known locations: kitchen, living_room, bedroom, charging_station
        """

        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10
        )

        # Action client for navigation
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Location database
        self.locations = {
            'kitchen': (2.0, 3.0, 0.0),
            'living_room': (0.0, 0.0, 0.0),
            'bedroom': (-2.0, 4.0, 1.57),
            'charging_station': (-1.0, -1.0, 3.14)
        }

        self.get_logger().info('LLM Task Planner ready')

    def command_callback(self, msg: String):
        """Process voice command through LLM."""
        command = msg.data
        self.get_logger().info(f'Planning for: "{command}"')

        # Generate plan using LLM
        plan = self.generate_plan(command)
        self.get_logger().info(f'Generated plan: {plan}')

        # Execute plan
        self.execute_plan(plan)

    def generate_plan(self, command: str) -> list:
        """Use LLM to convert command to action sequence."""
        prompt = f"""You are a robot task planner. Convert the user command into a sequence of robot actions.

{self.capabilities}

User command: "{command}"

Respond with a JSON array of actions. Example:
[{{"action": "navigate_to", "args": {{"location": "kitchen"}}}},
 {{"action": "pick_object", "args": {{"object_name": "cup"}}}},
 {{"action": "say", "args": {{"message": "Task complete"}}}}]

Only use available actions. Output valid JSON only."""

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1,
            max_tokens=500
        )

        plan_text = response.choices[0].message.content
        try:
            return json.loads(plan_text)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid plan JSON: {plan_text}')
            return []

    def execute_plan(self, plan: list):
        """Execute sequence of actions."""
        for step in plan:
            action = step.get('action')
            args = step.get('args', {})

            self.get_logger().info(f'Executing: {action}({args})')

            if action == 'navigate_to':
                self.navigate_to(args.get('location'))
            elif action == 'say':
                self.say(args.get('message'))
            elif action == 'wait':
                self.wait(args.get('seconds', 1))
            # Add more action handlers as needed

    def navigate_to(self, location: str):
        """Navigate to named location."""
        if location not in self.locations:
            self.get_logger().warn(f'Unknown location: {location}')
            return

        x, y, yaw = self.locations[location]

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0  # Simplified

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

    def say(self, message: str):
        """Text-to-speech output."""
        self.get_logger().info(f'Robot says: "{message}"')
        # Integrate with TTS system (e.g., pyttsx3, espeak)

    def wait(self, seconds: float):
        """Pause execution."""
        import time
        time.sleep(seconds)
```

![LLM-ROS Action Flow](/assets/ch09/llm-ros-action.svg)
*Figure 9.3: LLM task planner converting natural language to ROS 2 action sequences*

---

## Conversational AI Integration Patterns

Effective conversational robots require context management and clarification handling.

### Dialogue State Management

```python
# Environment: Python 3.10
# Dialogue context manager

from dataclasses import dataclass, field
from typing import List, Optional
from datetime import datetime

@dataclass
class DialogueTurn:
    """Single turn in conversation."""
    speaker: str  # 'user' or 'robot'
    utterance: str
    timestamp: datetime = field(default_factory=datetime.now)
    intent: Optional[str] = None
    entities: dict = field(default_factory=dict)

@dataclass
class DialogueContext:
    """Manages conversation state."""
    turns: List[DialogueTurn] = field(default_factory=list)
    current_task: Optional[str] = None
    held_object: Optional[str] = None
    robot_location: str = 'unknown'
    max_history: int = 10

    def add_turn(self, speaker: str, utterance: str,
                 intent: str = None, entities: dict = None):
        turn = DialogueTurn(
            speaker=speaker,
            utterance=utterance,
            intent=intent,
            entities=entities or {}
        )
        self.turns.append(turn)

        # Trim history
        if len(self.turns) > self.max_history:
            self.turns = self.turns[-self.max_history:]

    def get_context_string(self) -> str:
        """Format context for LLM prompt."""
        lines = [f"Robot location: {self.robot_location}"]
        if self.held_object:
            lines.append(f"Robot is holding: {self.held_object}")
        if self.current_task:
            lines.append(f"Current task: {self.current_task}")

        lines.append("\nRecent conversation:")
        for turn in self.turns[-5:]:
            lines.append(f"{turn.speaker}: {turn.utterance}")

        return "\n".join(lines)

    def needs_clarification(self, utterance: str) -> Optional[str]:
        """Check if command is ambiguous."""
        ambiguous_patterns = [
            ("pick up", "that", "Which object should I pick up?"),
            ("go to", "there", "Which location should I go to?"),
            ("bring", "it", "What should I bring?"),
        ]

        for action, pronoun, question in ambiguous_patterns:
            if action in utterance.lower() and pronoun in utterance.lower():
                return question

        return None
```

---

## End-to-End Autonomous System Walkthrough

Integrating all components into a complete voice-controlled humanoid system.

### System Architecture

```text
┌──────────────────────────────────────────────────────────────────┐
│                   Autonomous Humanoid System                     │
│                                                                  │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐ │
│  │ Microphone │─►│  Whisper   │─►│    LLM     │─►│   Action   │ │
│  │            │  │ Transcribe │  │  Planner   │  │  Executor  │ │
│  └────────────┘  └────────────┘  └────────────┘  └──────┬─────┘ │
│                                                         │       │
│                                          ┌──────────────┘       │
│                                          ▼                      │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐                 │
│  │  Cameras   │─►│   Isaac    │─►│    Nav2    │                 │
│  │  + Depth   │  │    ROS     │  │ Navigation │                 │
│  └────────────┘  └─────┬──────┘  └──────┬─────┘                 │
│                        │                │                       │
│                        ▼                ▼                       │
│                  ┌───────────────────────────┐                  │
│                  │     Robot Controller      │                  │
│                  │     (Joint Commands)      │                  │
│                  └───────────────────────────┘                  │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

### Launch Configuration

```python
# Environment: ROS 2 Humble, Python 3.10
# Full system launch file

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Speech recognition
        Node(
            package='humanoid_voice',
            executable='whisper_node',
            name='whisper_transcription',
            parameters=[{
                'model_size': 'base',
                'language': 'en'
            }]
        ),

        # Task planning
        Node(
            package='humanoid_planning',
            executable='llm_planner',
            name='task_planner',
            parameters=[{
                'llm_model': 'gpt-4',
                'temperature': 0.1
            }]
        ),

        # Perception (Isaac ROS)
        Node(
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='vslam'
        ),

        # Navigation
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='navigation'
        ),

        # Safety monitor
        Node(
            package='humanoid_safety',
            executable='safety_monitor',
            name='safety_monitor',
            parameters=[{
                'max_velocity': 0.5,
                'emergency_stop_topic': '/estop'
            }]
        ),

        # Text-to-speech feedback
        Node(
            package='humanoid_voice',
            executable='tts_node',
            name='text_to_speech'
        )
    ])
```

![End-to-End System](/assets/ch09/e2e-system.svg)
*Figure 9.4: Complete end-to-end autonomous humanoid system architecture*

---

## Practical Assessment: Autonomous Humanoid Capstone

### Objective

Build a voice-controlled robot that can navigate to locations and report what it observes.

### Requirements

1. Whisper transcription node for voice commands
2. LLM planner that converts commands to Nav2 goals
3. Camera feed with basic object detection
4. Text-to-speech feedback to user

### Example Interaction

```text
User: "Go to the kitchen and tell me what you see"

Robot processing:
1. Whisper: Transcribe → "Go to the kitchen and tell me what you see"
2. LLM Plan: [navigate_to(kitchen), observe(), say(observations)]
3. Nav2: Navigate to kitchen waypoint
4. Perception: Detect objects [table, chair, cup, refrigerator]
5. TTS: "I see a table, chair, cup, and refrigerator in the kitchen"
```

### Success Criteria

- [ ] Voice commands transcribed with >90% accuracy
- [ ] LLM generates valid action plans
- [ ] Robot navigates to correct locations
- [ ] Robot provides verbal feedback
- [ ] System handles clarification requests

---

## Summary

This capstone integrates all Physical AI concepts into a complete autonomous system:

- **VLA models** provide end-to-end vision-language-action capabilities
- **Whisper** enables robust voice command recognition
- **LLM planning** converts natural language to executable robot actions
- **ROS 2 integration** connects high-level planning to low-level control
- **Conversational AI** manages dialogue context and clarifications

The convergence of large language models, computer vision, and robotics is creating a new generation of intelligent, interactive machines.

---

## References

1. Brohan, A., et al. (2023). RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control. *arXiv preprint arXiv:2307.15818*.

2. Driess, D., et al. (2023). PaLM-E: An Embodied Multimodal Language Model. *ICML*.

3. Radford, A., et al. (2023). Robust Speech Recognition via Large-Scale Weak Supervision (Whisper). *ICML*.

4. OpenAI. (2023). GPT-4 Technical Report. *arXiv preprint arXiv:2303.08774*.

5. Ahn, M., et al. (2022). Do As I Can, Not As I Say: Grounding Language in Robotic Affordances. *CoRL*.

6. Huang, W., et al. (2022). Inner Monologue: Embodied Reasoning through Planning with Language Models. *CoRL*.

7. Open X-Embodiment Collaboration. (2023). Open X-Embodiment: Robotic Learning Datasets and RT-X Models. *arXiv preprint arXiv:2310.08864*.

---

*Word count: ~1,700 words*
