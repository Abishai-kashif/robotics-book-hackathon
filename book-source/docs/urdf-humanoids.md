---
sidebar_position: 6
---

# Understanding URDF for Humanoids

## Learning Objectives

- Design URDF files that accurately represent humanoid robot kinematics and dynamics
- Implement proper joint constraints and limits for safe humanoid robot operation
- Create URDF models that integrate with ROS 2 simulation and control systems
- Apply URDF best practices to complex multi-degree-of-freedom humanoid systems

## Introduction

The Unified Robot Description Format (URDF) serves as the standard for representing robot models in ROS, and its proper implementation is crucial for humanoid robotics applications. Unlike simpler robotic systems, humanoid robots require complex kinematic chains with multiple degrees of freedom, precise joint constraints, and detailed dynamic properties to enable accurate simulation and control.

In the context of Physical AI, URDF models must not only represent the physical structure of the robot but also provide the necessary information for perception, planning, and control algorithms to interact effectively with the robot's physical form. This includes proper inertial properties, visual representations, and collision geometries that reflect the robot's real-world behavior.

## Key Concepts

### URDF Structure for Humanoid Robots

A humanoid robot URDF consists of multiple interconnected links and joints that form the kinematic chains for legs, arms, torso, and head. The structure typically follows a tree topology with the base link (usually the pelvis or torso) as the root:

```xml
<robot name="humanoid_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Hip joint connecting to left leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="0 0.1 -0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  </joint>
</robot>
```

### Joint Types and Constraints

Humanoid robots require various joint types to achieve human-like mobility:

- **Revolute joints**: Single-axis rotation, used for most humanoid joints (knees, elbows, etc.)
- **Continuous joints**: Unlimited rotation, used for wheels or rotating sensors
- **Prismatic joints**: Linear motion, used for telescoping mechanisms
- **Fixed joints**: Rigid connections between links

Joint limits are critical for humanoid safety and realistic motion:

```xml
<joint name="knee_joint" type="revolute">
  <parent link="thigh"/>
  <child link="shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.0" effort="150" velocity="2.0"/>
  <safety_controller k_position="10" k_velocity="10"
                    soft_lower_limit="0.05" soft_upper_limit="1.95"/>
</joint>
```

### Inertial Properties and Dynamics

Accurate inertial properties are essential for realistic simulation and control:

- Mass: The mass of each link
- Center of mass: Location of the center of mass relative to the link frame
- Inertia matrix: How mass is distributed around the center of mass

## Practical Examples

### Example: Complete Humanoid Lower Body URDF

This example shows a simplified lower body model for a humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="humanoid_lower_body" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base/Pelvis Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.25 0.15"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.25 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0"
               iyy="0.15" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_yaw_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.78" upper="0.78" effort="200" velocity="2.0"/>
  </joint>

  <link name="left_hip">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <material name="dark_grey">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_hip_pitch_joint" type="revolute">
    <parent link="left_hip"/>
    <child link="left_thigh"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.5" effort="300" velocity="2.0"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="dark_grey">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0"
               iyy="0.02" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="300" velocity="2.0"/>
  </joint>

  <link name="left_shin">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="dark_grey">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <mass value="2.5"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0"
               iyy="0.015" iyz="0.0" izz="0.006"/>
    </inertial>
  </link>

  <joint name="left_ankle_pitch_joint" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="150" velocity="2.0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.1 0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Right Leg (mirror of left leg) -->
  <joint name="right_hip_yaw_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="0 -0.1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.78" upper="0.78" effort="200" velocity="2.0"/>
  </joint>

  <link name="right_hip">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <material name="dark_grey">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="right_hip_pitch_joint" type="revolute">
    <parent link="right_hip"/>
    <child link="right_thigh"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.5" effort="300" velocity="2.0"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="dark_grey">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0"
               iyy="0.02" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="300" velocity="2.0"/>
  </joint>

  <link name="right_shin">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="dark_grey">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <mass value="2.5"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0"
               iyy="0.015" iyz="0.0" izz="0.006"/>
    </inertial>
  </link>

  <joint name="right_ankle_pitch_joint" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="150" velocity="2.0"/>
  </joint>

  <link name="right_foot">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.1 0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

</robot>
```

### Example: URDF with Xacro for Complex Humanoid

Xacro (XML Macros) allows for more maintainable and parameterized URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_xacro">

  <!-- Parameters -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="mass_pelvis" value="10.0" />
  <xacro:property name="mass_leg" value="5.0" />
  <xacro:property name="length_thigh" value="0.4" />
  <xacro:property name="length_shin" value="0.4" />

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="dark_grey">
    <color rgba="0.3 0.3 0.3 1.0"/>
  </material>
  <material name="light_grey">
    <color rgba="0.7 0.7 0.7 1.0"/>
  </material>

  <!-- Macro for a leg segment -->
  <xacro:macro name="leg_segment" params="name parent xyz axis joint_limits mass size">
    <joint name="${name}_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${name}_link"/>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <axis xyz="${axis}"/>
      <limit lower="${joint_limits[0]}" upper="${joint_limits[1]}"
             effort="${joint_limits[2]}" velocity="${joint_limits[3]}"/>
    </joint>

    <link name="${name}_link">
      <visual>
        <origin xyz="0 0 ${-size[2]/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${size[0]}" length="${size[2]}"/>
        </geometry>
        <material name="dark_grey"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${-size[2]/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${size[0]}" length="${size[2]}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 ${-size[2]/2}" rpy="0 0 0"/>
        <mass value="${mass}"/>
        <inertia ixx="${mass*size[0]*size[0]/2}" ixy="0" ixz="0"
                 iyy="${mass*(3*size[0]*size[0] + size[2]*size[2])/12}" iyz="0"
                 izz="${mass*(3*size[0]*size[0] + size[2]*size[2])/12}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.25 0.15"/>
      </geometry>
      <material name="light_grey"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.25 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${mass_pelvis}"/>
      <inertia ixx="0.2" ixy="0" ixz="0"
               iyy="0.15" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left leg using macro -->
  <xacro:leg_segment name="left_thigh" parent="base_link"
                     xyz="0 0.1 -0.1" axis="0 1 0"
                     joint_limits="[-1.57, 0.5, 300, 2.0]"
                     mass="3.0" size="[0.06, 0.06, ${length_thigh}]"/>

  <xacro:leg_segment name="left_shin" parent="left_thigh_link"
                     xyz="0 0 ${-length_thigh}" axis="0 1 0"
                     joint_limits="[0, 2.0, 300, 2.0]"
                     mass="2.5" size="[0.05, 0.05, ${length_shin}]"/>

  <!-- Foot link -->
  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_shin_link"/>
    <child link="left_foot"/>
    <origin xyz="0 0 ${-length_shin}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="150" velocity="2.0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.1 0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.002" ixy="0" ixz="0"
               iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

</robot>
```

## Exercises

### Exercise 1: Complete Humanoid Upper Body URDF

Create a complete URDF model for a humanoid's upper body including:
- Torso with proper inertial properties
- Two arms with shoulder, elbow, and wrist joints
- Head with neck joint
- Proper joint limits for safe operation
- Visual and collision geometries for each link

### Exercise 2: URDF with Gazebo Integration

Extend your URDF to include Gazebo-specific tags for simulation:
- Add transmission elements for joint control
- Include Gazebo materials and colors
- Add plugin configurations for sensors
- Define proper friction and damping parameters

## Summary

URDF is the fundamental format for describing humanoid robot models in ROS, and its proper implementation is critical for successful simulation, control, and perception in Physical AI systems. Creating accurate URDF models requires careful attention to kinematic structure, dynamic properties, and geometric representations that reflect the physical robot's characteristics.

The use of Xacro macros and parameterization makes complex humanoid URDFs more maintainable and allows for variations of the same basic design. Proper joint limits and safety controllers are essential for preventing damage to real robots during operation.

## References

- URDF Documentation: http://wiki.ros.org/urdf
- Xacro Documentation: http://wiki.ros.org/xacro
- ROS 2 URDF Tutorials: https://docs.ros.org/en/rolling/Tutorials/URDF/Working-with-URDF.html
- "Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart
- "Mastering ROS for Robotics Programming" by Lentin Joseph
- Gazebo URDF Integration: http://gazebosim.org/tutorials?tut=ros_urdf