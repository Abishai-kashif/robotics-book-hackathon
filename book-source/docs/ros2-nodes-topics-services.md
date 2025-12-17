---
sidebar_position: 4
---

# ROS 2 Nodes, Topics, and Services

## Learning Objectives

- Implement ROS 2 nodes with proper lifecycle management for humanoid robot applications
- Design efficient topic-based communication patterns for real-time robot control
- Create robust services for synchronous robot operations and state management
- Apply communication patterns to Physical AI systems with real-time constraints

## Introduction

The core communication mechanisms in ROS 2 - nodes, topics, and services - form the foundation of distributed robot control systems. In Physical AI applications, these mechanisms must handle real-time constraints while maintaining reliable communication between perception, planning, and control systems. Understanding how to properly design and implement these communication patterns is crucial for developing responsive and robust humanoid robot systems.

Nodes represent individual processes that perform specific functions within the robot system. Topics enable asynchronous data sharing through publish-subscribe patterns, ideal for sensor data and state information. Services provide synchronous request-response communication for operations requiring confirmation and error handling.

## Key Concepts

### Node Lifecycle and Management

ROS 2 nodes follow a well-defined lifecycle that includes initialization, configuration, activation, and cleanup phases. For humanoid robots, proper lifecycle management ensures safe startup and shutdown sequences:

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

class HumanoidController(LifecycleNode):
    def __init__(self):
        super().__init__('humanoid_controller')

    def on_configure(self, state):
        self.get_logger().info('Configuring humanoid controller')
        # Initialize hardware interfaces
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating humanoid controller')
        # Enable actuators safely
        return TransitionCallbackReturn.SUCCESS
```

### Topic Communication Patterns

Topics in ROS 2 implement publish-subscribe communication with different quality of service (QoS) profiles to meet various real-time requirements:

- **Reliability**: Ensure all messages are delivered (for critical control commands)
- **Durability**: Provide message persistence for late-joining subscribers
- **History**: Control how many messages are retained
- **Deadline**: Set maximum time between messages

For Physical AI systems, selecting appropriate QoS profiles is crucial for meeting real-time performance requirements.

### Service Architecture for Robot Control

Services provide synchronous communication suitable for operations requiring confirmation, such as:
- Robot state changes (enable/disable)
- Calibration procedures
- Emergency stops
- Configuration updates

Services include built-in error handling and response confirmation, making them ideal for critical robot operations.

## Practical Examples

### Example: Humanoid Joint Control Node

This example demonstrates a ROS 2 node that controls humanoid joint positions with proper error handling and safety checks:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.srv import QueryTrajectoryState
import time

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publishers for joint commands
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribers for joint feedback
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Service for querying joint states
        self.query_service = self.create_service(
            QueryTrajectoryState,
            'query_joint_state',
            self.query_joint_state_callback
        )

        # Initialize joint positions
        self.current_positions = {}

    def joint_state_callback(self, msg):
        """Update current joint positions from feedback"""
        for name, position in zip(msg.name, msg.position):
            self.current_positions[name] = position

    def move_to_position(self, joint_names, positions, duration=1.0):
        """Move joints to specified positions"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        trajectory_msg.points.append(point)
        self.joint_pub.publish(trajectory_msg)

    def query_joint_state_callback(self, request, response):
        """Service callback to query current joint state"""
        try:
            response.name = request.name
            response.position = [self.current_positions.get(name, 0.0)
                                for name in request.name]
            response.velocity = [0.0] * len(request.name)  # Simplified
            response.effort = [0.0] * len(request.name)    # Simplified
            response.success = True
        except Exception as e:
            response.success = False
            response.message = f"Error querying joint state: {str(e)}"

        return response

def main(args=None):
    rclpy.init(args=args)
    controller = JointController()

    # Example: Move humanoid to a standing position
    joints = ['hip_joint', 'knee_joint', 'ankle_joint']
    positions = [0.0, 0.0, 0.0]  # Standing position

    # Wait for a moment before publishing
    time.sleep(1)
    controller.move_to_position(joints, positions)

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: Safety Monitor Service

This example demonstrates a safety monitoring service that validates robot commands before execution:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Subscribe to joint states for monitoring
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Service for validating commands
        self.validate_service = self.create_service(
            Trigger,
            'validate_command',
            self.validate_command_callback
        )

        # Safety parameters
        self.max_joint_velocity = 2.0  # rad/s
        self.joint_limits = {
            'hip_joint': (-1.57, 1.57),
            'knee_joint': (0.0, 2.0),
            'ankle_joint': (-0.78, 0.78)
        }

        self.current_positions = {}

    def joint_state_callback(self, msg):
        """Monitor joint states for safety violations"""
        self.current_positions.update(dict(zip(msg.name, msg.position)))

        # Check for joint limit violations
        for name, pos in self.current_positions.items():
            if name in self.joint_limits:
                min_limit, max_limit = self.joint_limits[name]
                if pos < min_limit or pos > max_limit:
                    self.get_logger().warn(f'Joint {name} limit violation: {pos}')

    def validate_command_callback(self, request, response):
        """Validate a robot command for safety"""
        try:
            # In a real system, this would validate the actual command
            # For this example, we'll just return success
            response.success = True
            response.message = "Command validated successfully"
        except Exception as e:
            response.success = False
            response.message = f"Command validation failed: {str(e)}"

        return response

def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()
    rclpy.spin(safety_monitor)
    safety_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

### Exercise 1: Create a Multi-Node Communication System

Design and implement a multi-node ROS 2 system for humanoid robot control that includes:
- A sensor node that publishes IMU and joint state data
- A controller node that subscribes to sensor data and publishes joint commands
- A safety node that monitors all communications and can halt the robot
- Use appropriate QoS profiles for real-time communication

### Exercise 2: Implement a Fault-Tolerant Service Architecture

Create a service architecture that handles robot calibration with the following features:
- Primary calibration service that performs the actual calibration
- Backup service that takes over if the primary fails
- Client that automatically switches between services
- Health monitoring to detect service availability

## Summary

ROS 2 nodes, topics, and services provide the essential communication infrastructure for Physical AI and humanoid robot systems. Proper implementation of these communication patterns ensures reliable, real-time robot control with appropriate safety measures. The publish-subscribe model enables efficient data sharing, while services provide synchronous communication for critical operations.

Understanding QoS profiles and lifecycle management is crucial for developing robust robot systems that can handle real-time constraints while maintaining safety and reliability.

## References

- ROS 2 Communication Patterns: https://docs.ros.org/en/rolling/Concepts/About-Topics-Services-Actions.html
- Quality of Service in ROS 2: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
- "Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart
- "Mastering ROS for Robotics Programming" by Lentin Joseph
- Real-time ROS 2: https://github.com/ros2-realtime-demo