# Quickstart Guide: Vision-Language-Action (VLA) Systems

## Overview
This guide provides a quick introduction to developing Vision-Language-Action (VLA) systems for the Physical AI & Humanoid Robotics textbook Module 4. This guide builds upon the ROS 2 foundation established in previous modules.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- NVIDIA GPU with RTX capabilities (for optimal performance)
- ROS 2 Humble Hawksbill installed
- NVIDIA Isaac SDK (if available)
- Python 3.11+ with pip

### Software Dependencies
```bash
# ROS 2 dependencies
sudo apt update
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Python packages for VLA systems
pip install torch torchvision torchaudio
pip install transformers  # For vision-language models
pip install openai-whisper  # For speech recognition (if needed)
pip install opencv-python  # For computer vision
pip install numpy matplotlib
```

## Setting Up the Development Environment

### 1. Clone the Textbook Repository
```bash
git clone <repository-url>
cd robotics-book-2
git checkout 004-vla-module
```

### 2. Install ROS 2 Workspace Dependencies
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
# Create or navigate to your VLA package directory
mkdir -p vla_systems/src
cd vla_systems
```

### 3. Create a VLA Package
```bash
cd ~/ros2_ws/src/vla_systems
ros2 pkg create --build-type ament_python vla_integration
```

## Basic VLA System Example

### 1. Create a Simple VLA Node
Create `~/ros2_ws/src/vla_systems/vla_integration/vla_integration/vla_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv2 import cv2 as cv
import numpy as np

class VLANode(Node):
    def __init__(self):
        super().__init__('vla_node')

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String,
            'vla/command',
            self.command_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.action_pub = self.create_publisher(
            String,
            'vla/action_plan',
            10
        )

        self.get_logger().info('VLA Node initialized')

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        # Process command and generate action plan
        action_plan = self.generate_action_plan(command)
        self.publish_action_plan(action_plan)

    def image_callback(self, msg):
        # Process image data for vision component
        self.get_logger().info('Received image data')

    def generate_action_plan(self, command):
        # Simple command parser - in real implementation, this would use
        # vision-language models to generate action sequences
        if 'pick up' in command.lower():
            return 'move_to_object, grasp_object, lift_object'
        elif 'move to' in command.lower():
            return 'navigate_to_location'
        else:
            return 'idle'

    def publish_action_plan(self, action_plan):
        msg = String()
        msg.data = action_plan
        self.action_pub.publish(msg)
        self.get_logger().info(f'Published action plan: {action_plan}')

def main(args=None):
    rclpy.init(args=args)
    vla_node = VLANode()
    rclpy.spin(vla_node)
    vla_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Configure the Package
Update `~/ros2_ws/src/vla_systems/vla_integration/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'vla_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='VLA Integration Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_node = vla_integration.vla_node:main',
        ],
    },
)
```

### 3. Build and Run the Example
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vla_integration
source install/setup.bash

# Run the VLA node
ros2 run vla_integration vla_node
```

### 4. Test the System
In another terminal:
```bash
# Send a command to the VLA system
ros2 topic pub /vla/command std_msgs/String "data: 'Pick up the red ball'"

# Monitor the action plan output
ros2 topic echo /vla/action_plan
```

## Simulation Example with Gazebo

### 1. Launch Gazebo with a Simple Environment
```bash
# Launch Gazebo simulation
ros2 launch gazebo_ros empty_world.launch.py
```

### 2. Spawn a Robot with Camera
```bash
# Spawn a robot with camera sensor
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/robot/model.sdf
```

## Content Development Workflow

### 1. Creating New Chapter Content
All VLA module content should be created directly in the book-source/docs directory:

```bash
# Create new chapter file
touch book-source/docs/voice-to-action.md
```

### 2. Chapter Structure Template
Use this template for new chapter files:

```markdown
# Vision-Language-Action Integration

## Learning Objectives
- Understand the fundamentals of VLA systems
- Implement a basic VLA pipeline using ROS 2
- Connect vision-language models to robotic action

## Introduction
[Content about VLA systems and their importance in Physical AI]

## Theoretical Background
[Academic content about vision-language models and action planning]

## Practical Implementation
[ROS 2 code examples and implementation details]

## Simulation Example
[Step-by-step simulation example using Gazebo/Isaac Sim]

## Connecting to Previous Modules
[How this connects to content from Modules 1-3]

## Exercises
[Hands-on exercises for students]

## Summary
[Key takeaways from the chapter]
```

## Key Files for Module 4

The following files will be created as part of Module 4:

- `book-source/docs/voice-to-action.md` - Core VLA system integration
- `book-source/docs/vision-language-models.md` - Vision-language model foundations
- `book-source/docs/action-planning.md` - Action planning and execution
- `book-source/docs/multimodal-interaction.md` - Multi-modal interaction concepts
- `book-source/docs/vla-simulation-examples.md` - Simulation examples for VLA
- `book-source/docs/vla-ros-integration.md` - ROS 2 integration for VLA systems

## Running the Complete Example

1. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

2. Launch simulation environment:
```bash
ros2 launch gazebo_ros empty_world.launch.py
```

3. Run the VLA node:
```bash
ros2 run vla_integration vla_node
```

4. Send commands and observe the system behavior:
```bash
ros2 topic pub /vla/command std_msgs/String "data: 'Move to the blue cube'"
```

## Troubleshooting

### Common Issues
- **ROS 2 not found**: Ensure you've sourced the ROS 2 setup.bash file
- **GPU memory issues**: Reduce model size or batch processing for VLA systems
- **Camera not publishing**: Check Gazebo simulation and camera plugin configuration

### Verification Steps
1. Confirm ROS 2 nodes are communicating:
   ```bash
   ros2 node list
   ros2 topic list
   ```
2. Verify VLA node is running:
   ```bash
   ros2 run vla_integration vla_node
   ```

This quickstart guide provides the foundation for developing and understanding Vision-Language-Action systems as part of Module 4 of the Physical AI & Humanoid Robotics textbook.