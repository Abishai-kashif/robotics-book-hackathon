# Sim2Real Transfer Example

## Scenario: Humanoid Robot Walking Controller Transfer

This example demonstrates the complete Sim2Real transfer process for developing a humanoid robot walking controller.

### Components
- Gazebo simulation environment with humanoid robot model
- Walking controller algorithm
- Domain randomization parameters
- ROS 2 interfaces for simulation and real robot
- Performance evaluation metrics

### Simulation Setup
```xml
<!-- Gazebo world file for humanoid walking simulation -->
<sdf version="1.7">
  <world name="humanoid_walking">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane with variable friction -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <surface>
            <friction>
              <ode>
                <mu>$(randomize 0.4 1.0)</mu>
                <mu2>$(randomize 0.4 1.0)</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Humanoid robot model -->
    <include>
      <uri>model://humanoid_robot</uri>
      <pose>0 0 0.8 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### ROS 2 Controller Implementation
```python
# ROS 2 node for Sim2Real walking controller
# Implements domain randomization during training

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64MultiArray
import numpy as np
import random

class Sim2RealWalkingController(Node):
    def __init__(self):
        super().__init__('sim2real_walking_controller')

        # Publishers and subscribers
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10)
        self.imu_subscriber = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.joint_state_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)

        # Walking state
        self.current_joint_states = None
        self.imu_data = None
        self.balance_error = 0.0
        self.step_phase = 0.0

        # Domain randomization parameters
        self.randomize_physics_parameters()

    def randomize_physics_parameters(self):
        """Apply domain randomization to physics parameters"""
        # Randomize robot mass properties
        self.robot_mass_multiplier = random.uniform(0.8, 1.2)

        # Randomize actuator dynamics
        self.actuator_delay = random.uniform(0.005, 0.015)
        self.actuator_noise = random.uniform(0.001, 0.01)

        # Randomize sensor noise
        self.imu_noise = random.uniform(0.001, 0.01)

    def imu_callback(self, msg):
        # Process IMU data with simulated noise
        noisy_orientation = self.add_sensor_noise(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z],
            self.imu_noise
        )
        self.imu_data = {
            'orientation': noisy_orientation,
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }

        # Calculate balance error
        self.balance_error = self.calculate_balance_error(self.imu_data)

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def calculate_balance_error(self, imu_data):
        """Calculate balance error from IMU data"""
        # Simple balance error based on roll and pitch angles
        roll = np.arctan2(
            2 * (imu_data['orientation'][3] * imu_data['orientation'][0] +
                 imu_data['orientation'][1] * imu_data['orientation'][2]),
            1 - 2 * (imu_data['orientation'][0]**2 + imu_data['orientation'][1]**2)
        )
        pitch = np.arcsin(
            2 * (imu_data['orientation'][3] * imu_data['orientation'][1] -
                 imu_data['orientation'][2] * imu_data['orientation'][0])
        )

        return np.sqrt(roll**2 + pitch**2)

    def control_loop(self):
        """Main walking control loop with Sim2Real considerations"""
        if self.current_joint_states and self.imu_data:
            # Generate walking commands based on balance and step phase
            joint_commands = self.generate_walking_commands()

            # Apply actuator delay simulation
            joint_commands = self.apply_actuator_dynamics(joint_commands)

            # Publish commands with noise simulation
            command_msg = Float64MultiArray()
            noisy_commands = self.add_actuator_noise(joint_commands, self.actuator_noise)
            command_msg.data = noisy_commands
            self.joint_command_publisher.publish(command_msg)

            # Update step phase
            self.step_phase += 0.01

    def generate_walking_commands(self):
        """Generate walking commands based on current state"""
        # Simplified walking controller using balance feedback
        balance_compensation = -self.balance_error * 10.0
        step_oscillation = np.sin(self.step_phase * 10.0) * 0.1

        # Return joint commands for walking
        return [balance_compensation + step_oscillation] * len(self.current_joint_states.name)

    def add_sensor_noise(self, data, noise_level):
        """Add noise to sensor data to simulate reality gap"""
        return [x + random.gauss(0, noise_level) for x in data]

    def add_actuator_noise(self, commands, noise_level):
        """Add noise to actuator commands"""
        return [cmd + random.gauss(0, noise_level) for cmd in commands]

    def apply_actuator_dynamics(self, commands):
        """Simulate actuator dynamics and delays"""
        # Apply simple first-order dynamics simulation
        return commands  # Simplified implementation

def main(args=None):
    rclpy.init(args=args)
    controller = Sim2RealWalkingController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Unity Visualization Integration
For enhanced visualization, Unity can be used alongside Gazebo:

1. **Unity Environment**: Create detailed visual environments for better rendering
2. **ROS 2 Integration**: Use ROS 2 Unity integration packages for data exchange
3. **Photorealistic Simulation**: Implement advanced rendering for perception training
4. **Human-in-the-loop**: Enable human interaction with simulated humanoid robots

### NVIDIA Isaac Platform Considerations
- GPU-accelerated physics simulation
- Photorealistic rendering for perception tasks
- Synthetic data generation capabilities
- Integration with AI training pipelines

### Multi-Platform Consistency
- Maintain consistent physics parameters across simulation platforms
- Standardize sensor interfaces between simulation and reality
- Validate simulation accuracy through real-world testing