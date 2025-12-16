---
sidebar_position: 5
---

# Bridging Python Agents to ROS Controllers using rclpy

## Learning Objectives

- Integrate Python-based AI agents with ROS 2 robot controllers using rclpy
- Implement communication patterns between high-level AI reasoning and low-level robot control
- Design efficient data structures for transferring information between Python agents and ROS controllers
- Apply agent-controller integration to humanoid robot decision-making and control systems

## Introduction

The integration of Python-based AI agents with ROS 2 robot controllers through rclpy represents a crucial bridge between high-level cognitive functions and low-level robot control in Physical AI systems. Python's rich ecosystem of machine learning libraries, combined with ROS 2's robust communication infrastructure, enables sophisticated robot behaviors that incorporate perception, reasoning, and action in a unified framework.

This integration is particularly important for humanoid robots, where cognitive agents must make real-time decisions based on sensor data while coordinating complex motor control systems. The rclpy library provides the necessary Python bindings to ROS 2, enabling seamless communication between AI algorithms and robot hardware.

## Key Concepts

### rclpy Architecture and Node Design

rclpy provides Python access to ROS 2's client library, enabling Python programs to participate in ROS 2 communication as nodes. The architecture follows the same principles as other ROS 2 client libraries but with Python-specific idioms:

```python
import rclpy
from rclpy.node import Node

class AIControllerNode(Node):
    def __init__(self):
        super().__init__('ai_controller_node')

        # Publishers for sending commands to robot
        self.cmd_pub = self.create_publisher(RobotCommand, 'robot_command', 10)

        # Subscribers for receiving sensor data
        self.sensor_sub = self.create_subscription(
            SensorData,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Initialize AI agent
        self.ai_agent = self.initialize_agent()
```

### Asynchronous Communication Patterns

Python agents often perform computationally intensive operations that should not block the ROS communication loop. Proper integration requires careful handling of asynchronous operations:

```python
import asyncio
from rclpy.executors import MultiThreadedExecutor

# Use multi-threaded executor to handle both ROS and AI processing
def main():
    rclpy.init()
    ai_node = AIControllerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(ai_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        ai_node.destroy_node()
        rclpy.shutdown()
```

### Data Serialization and Type Conversion

Efficient data transfer between Python agents and ROS controllers requires careful consideration of data formats and serialization methods. NumPy arrays are commonly used for sensor data processing and must be converted to ROS message formats for communication.

## Practical Examples

### Example: AI-Powered Humanoid Behavior Controller

This example demonstrates an AI agent that controls humanoid robot behaviors based on sensor inputs and high-level goals:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import time

class AIHumanoidController(Node):
    def __init__(self):
        super().__init__('ai_humanoid_controller')

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        # Internal state
        self.current_joints = {}
        self.imu_data = None
        self.behavior_state = 'idle'

        # Timer for AI decision making
        self.ai_timer = self.create_timer(0.1, self.ai_decision_callback)

        self.get_logger().info('AI Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Update internal joint state from robot feedback"""
        for name, position in zip(msg.name, msg.position):
            self.current_joints[name] = position

    def imu_callback(self, msg):
        """Process IMU data for balance and orientation"""
        self.imu_data = {
            'orientation': [msg.orientation.x, msg.orientation.y,
                           msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x,
                                msg.angular_velocity.y,
                                msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z]
        }

    def ai_decision_callback(self):
        """Main AI decision-making loop"""
        # Process sensor data
        sensor_input = self.get_sensor_input()

        # Apply AI reasoning
        action = self.decide_action(sensor_input)

        # Execute action
        self.execute_action(action)

    def get_sensor_input(self):
        """Aggregate and process sensor data for AI"""
        sensor_data = {
            'joint_positions': self.current_joints,
            'imu_data': self.imu_data,
            'timestamp': time.time()
        }
        return sensor_data

    def decide_action(self, sensor_input):
        """AI decision-making function"""
        # Simple balance control example
        if self.imu_data:
            roll, pitch, yaw = self.get_euler_from_quaternion(
                self.imu_data['orientation']
            )

            # If tilted beyond threshold, try to balance
            if abs(pitch) > 0.2 or abs(roll) > 0.2:
                return {
                    'type': 'balance',
                    'pitch_error': pitch,
                    'roll_error': roll
                }

        # Default: maintain current pose
        return {'type': 'maintain_pose'}

    def execute_action(self, action):
        """Convert AI decisions to ROS commands"""
        if action['type'] == 'balance':
            # Generate corrective joint commands
            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']

            # Simple proportional control for balance
            pitch_correction = -action['pitch_error'] * 0.5
            roll_correction = -action['roll_error'] * 0.5

            cmd_msg.position = [
                pitch_correction,  # Hip adjustment
                roll_correction,   # Knee adjustment
                pitch_correction   # Ankle adjustment
            ]

            self.joint_cmd_pub.publish(cmd_msg)

    def get_euler_from_quaternion(self, quaternion):
        """Convert quaternion to Euler angles"""
        x, y, z, w = quaternion

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    controller = AIHumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: Vision-Language-Action Integration

This example demonstrates how to integrate visual perception, language processing, and robot action through rclpy:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np
from cv_bridge import CvBridge
import cv2

class VisionLanguageActionNode(Node):
    def __init__(self):
        super().__init__('vla_node')

        # Publishers
        self.action_pub = self.create_publisher(
            String,
            'robot_action',
            10
        )
        self.gesture_pub = self.create_publisher(
            Point,
            'gesture_target',
            10
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.command_sub = self.create_subscription(
            String,
            'voice_command',
            self.command_callback,
            10
        )

        # Initialize components
        self.bridge = CvBridge()
        self.last_image = None
        self.pending_command = None

        self.get_logger().info('Vision-Language-Action node initialized')

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_image = cv_image
            self.process_image_command()
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def command_callback(self, msg):
        """Process incoming voice commands"""
        self.pending_command = msg.data
        self.process_image_command()

    def process_image_command(self):
        """Process combined image and command data"""
        if self.last_image is not None and self.pending_command is not None:
            # Simple object detection and action planning
            detected_objects = self.detect_objects(self.last_image)
            action = self.plan_action(
                detected_objects,
                self.pending_command
            )

            if action:
                self.execute_action(action)
                self.pending_command = None

    def detect_objects(self, image):
        """Simple object detection (in practice, use deep learning models)"""
        # Convert to HSV for simple color-based detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for common objects
        color_ranges = {
            'red_ball': (np.array([0, 50, 50]), np.array([10, 255, 255])),
            'blue_box': (np.array([100, 50, 50]), np.array([130, 255, 255])),
        }

        objects = []
        for obj_name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                if cv2.contourArea(contour) > 100:  # Filter small objects
                    # Calculate center of the object
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        objects.append({
                            'name': obj_name,
                            'center': (cx, cy),
                            'area': cv2.contourArea(contour)
                        })

        return objects

    def plan_action(self, objects, command):
        """Plan action based on objects and command"""
        command_lower = command.lower()

        for obj in objects:
            if obj['name'] in command_lower or 'ball' in command_lower:
                # Move towards the object
                center_x, center_y = obj['center']

                # Convert image coordinates to robot coordinates
                # (simplified for example)
                robot_x = (center_x - 320) / 100.0  # Normalize to robot space
                robot_y = (240 - center_y) / 100.0  # Invert Y axis

                return {
                    'type': 'move_to_object',
                    'target': {'x': robot_x, 'y': robot_y},
                    'object': obj['name']
                }

        return None

    def execute_action(self, action):
        """Execute planned action"""
        if action['type'] == 'move_to_object':
            target = action['target']
            target_msg = Point()
            target_msg.x = target['x']
            target_msg.y = target['y']
            target_msg.z = 0.0  # Ground level

            self.gesture_pub.publish(target_msg)

            action_msg = String()
            action_msg.data = f"Moving to {action['object']} at ({target['x']:.2f}, {target['y']:.2f})"
            self.action_pub.publish(action_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionLanguageActionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

### Exercise 1: Implement a Learning Agent with ROS Integration

Create a Python-based learning agent that:
- Subscribes to sensor data from a humanoid robot
- Learns optimal control policies through interaction
- Publishes commands to improve robot performance
- Implements a replay buffer for experience replay

### Exercise 2: Multi-Agent Coordination System

Design a system where multiple AI agents coordinate through ROS 2 topics and services:
- One agent handles perception and object recognition
- Another agent handles path planning and navigation
- A third agent handles high-level task planning
- Use appropriate message passing patterns to coordinate between agents

## Summary

The integration of Python AI agents with ROS 2 controllers through rclpy enables sophisticated Physical AI applications by combining the strengths of Python's machine learning ecosystem with ROS 2's robust robot communication infrastructure. This bridge allows for complex decision-making processes to control physical robots in real-time, which is essential for humanoid robots that must interact with dynamic environments.

Proper implementation requires careful consideration of asynchronous processing, data serialization, and real-time constraints to ensure responsive and reliable robot behavior.

## References

- rclpy Documentation: https://docs.ros.org/en/rolling/p/rclpy/
- ROS 2 Python Client Library: https://github.com/ros2/rclpy
- "Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart
- "Python Robotics" by Anis Koubaa
- CV Bridge: https://github.com/ros-perception/vision_opencv