# ROS 2 Integration Problems

## Exercise Set: Understanding ROS 2 in Physical AI Contexts

### Exercise 1: Node Communication Design
**Problem**: Design a ROS 2 communication architecture for a humanoid robot performing a pick-and-place task. Identify the necessary topics, services, and actions, and explain how they coordinate the physical task.

**Solution Approach**:
- **Topics**:
  - `/camera/image_raw` - Visual input for object detection
  - `/joint_states` - Current joint positions for feedback
  - `/tf` - Transform data for spatial relationships
  - `/imu/data` - Balance and orientation information
- **Services**:
  - `/arm_controller/enable` - Enable/disable arm controller
  - `/gripper/grasp` - Execute grasp operation
- **Actions**:
  - `/move_group/execute_trajectory` - Execute complex movements
  - `/navigation/navigate_to_pose` - Navigate to object location

### Exercise 2: rclpy Implementation
**Problem**: Implement a simple rclpy node that subscribes to joint state information and publishes velocity commands for a humanoid robot's arm. Include error handling for communication failures.

**Solution Approach**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control')
        self.joint_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.velocity_publisher = self.create_publisher(
            Float64MultiArray, '/velocity_commands', 10)
        self.last_joint_states = None

    def joint_callback(self, msg):
        try:
            self.last_joint_states = msg
            # Calculate velocity commands based on desired behavior
            velocity_cmd = self.calculate_velocity_commands(msg)
            self.velocity_publisher.publish(velocity_cmd)
        except Exception as e:
            self.get_logger().error(f'Error in joint callback: {e}')
```

### Exercise 3: Physical AI Integration
**Problem**: Explain how ROS 2's Quality of Service (QoS) settings can be configured to ensure reliable communication in time-critical Physical AI applications like balance control for humanoid robots.

**Solution Approach**:
- **Reliability**: Use RELIABLE for critical control data like joint states
- **Durability**: Use TRANSIENT_LOCAL for configuration parameters
- **Deadline**: Set deadlines for time-critical messages (e.g., balance control)
- **Liveliness**: Monitor node health for safety-critical applications
- **History**: Keep adequate history for control algorithms to function properly

### Exercise 4: Multi-Node Coordination
**Problem**: Design a ROS 2 system architecture for a humanoid robot that simultaneously performs navigation, manipulation, and balance control. How would you handle potential conflicts between these behaviors?

**Solution Approach**:
- Use action servers for high-level behaviors with built-in coordination
- Implement a behavior arbitration system as a central node
- Use priority-based message handling for conflicting commands
- Implement safety layers that can override lower-priority behaviors