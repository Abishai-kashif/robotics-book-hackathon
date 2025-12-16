---
sidebar_position: 3
---

# ROS 2 Fundamentals

## Learning Objectives

- Understand the core architecture and concepts of ROS 2 as a middleware for robot control
- Identify the key differences between ROS 1 and ROS 2 and their implications for Physical AI
- Implement basic ROS 2 nodes, topics, and services for robot communication
- Apply ROS 2 concepts to humanoid robot control systems

## Introduction

Robot Operating System 2 (ROS 2) serves as the foundational middleware for modern robotics applications, providing a flexible framework for robot control and communication. In the context of Physical AI and humanoid robotics, ROS 2 enables seamless integration between perception, planning, and control systems. Unlike its predecessor, ROS 2 addresses critical issues such as real-time performance, security, and multi-robot systems that are essential for Physical AI applications.

ROS 2's architecture is built on DDS (Data Distribution Service) for communication, providing improved reliability and real-time capabilities that are crucial for controlling physical systems. This makes it particularly suitable for humanoid robots that require precise timing and coordination between multiple subsystems.

## Key Concepts

### ROS 2 Architecture

ROS 2 utilizes a distributed system architecture based on the DDS (Data Distribution Service) standard. This provides several advantages over ROS 1's centralized master approach:

- **Decentralized Communication**: Nodes can discover and communicate with each other without a central master
- **Improved Real-time Performance**: DDS provides better timing guarantees essential for robot control
- **Enhanced Security**: Built-in security features support authentication, access control, and encryption
- **Multi-robot Support**: Better handling of multiple robots and distributed systems

### Nodes in ROS 2

Nodes are the fundamental building blocks of ROS 2 applications. Each node represents a single process that performs specific functionality within the robot system:

```python
import rclpy
from rclpy.node import Node

class HumanoidControlNode(Node):
    def __init__(self):
        super().__init__('humanoid_control_node')
        self.get_logger().info('Humanoid Control Node initialized')
```

### Topics and Message Passing

Topics enable asynchronous communication between nodes through a publish-subscribe pattern. This is essential for Physical AI systems where sensor data needs to be shared across multiple processing nodes:

```python
from std_msgs.msg import String

# Publisher
publisher = self.create_publisher(String, 'robot_status', 10)

# Subscriber
subscriber = self.create_subscription(
    String,
    'sensor_data',
    self.sensor_callback,
    10
)
```

### Services and Actions

Services provide synchronous request-response communication, while Actions offer asynchronous communication with feedback and goal management - crucial for humanoid robot behaviors that may take extended time to complete.

## Practical Examples

### Example: Basic ROS 2 Publisher for Sensor Data

This example demonstrates creating a ROS 2 publisher that simulates sensor data from a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        msg.position = [
            math.sin(self.i * 0.1),
            math.cos(self.i * 0.1),
            math.sin(self.i * 0.2)
        ]
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.position}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: ROS 2 Service for Robot Control

This example demonstrates creating a service that accepts high-level commands for humanoid robot control:

```python
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.srv import SetBool
import rclpy

class RobotControlService(Node):
    def __init__(self):
        super().__init__('robot_control_service')
        self.srv = self.create_service(
            SetBool,
            'enable_robot',
            self.enable_robot_callback
        )

    def enable_robot_callback(self, request, response):
        if request.data:
            self.get_logger().info('Enabling robot control')
            # Enable robot systems
            response.success = True
            response.message = 'Robot enabled successfully'
        else:
            self.get_logger().info('Disabling robot control')
            # Disable robot systems safely
            response.success = True
            response.message = 'Robot disabled successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    service = RobotControlService()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()
```

## Exercises

### Exercise 1: Create a ROS 2 Node for Humanoid Balance Control

Create a ROS 2 node that subscribes to IMU sensor data and publishes corrective commands to maintain humanoid balance. The node should:
- Subscribe to IMU orientation data
- Calculate deviation from upright position
- Publish corrective joint commands to maintain balance

### Exercise 2: Implement a ROS 2 Action Server for Walking Gait

Implement an action server that manages a humanoid robot's walking gait. The action should:
- Accept parameters for walking speed and distance
- Provide feedback on current step progress
- Handle preemption if the robot needs to stop walking
- Return success when the walk is completed

## Summary

ROS 2 provides the essential middleware infrastructure for Physical AI and humanoid robotics applications. Its decentralized architecture, real-time capabilities, and security features make it suitable for controlling complex physical systems. Understanding ROS 2 fundamentals is crucial for developing robust and reliable robot control systems that can integrate perception, planning, and action in real-time.

The publish-subscribe model enables efficient data sharing between robot subsystems, while services and actions provide the necessary synchronous and asynchronous communication patterns for complex robot behaviors.

## References

- ROS 2 Documentation: https://docs.ros.org/en/rolling/
- DDS Specification: https://www.omg.org/spec/DDS/
- ROS 2 for Real-time Systems: https://github.com/ros2-realtime-demo
- "Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart
- "Effective Robotics Programming with ROS" by Anil Mahtani, Enrique Fernandez, and Luis Sanchez Crespo