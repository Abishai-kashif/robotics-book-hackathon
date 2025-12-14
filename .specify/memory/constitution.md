# Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Physical AI First
Every feature and concept must connect AI systems to the physical world. All implementations must demonstrate how digital intelligence translates to physical action. Clear understanding of embodied intelligence required - no purely theoretical AI concepts without physical application context.

### II. ROS 2 Integration (NON-NEGOTIABLE)
All robotic control concepts must utilize ROS 2 (Robot Operating System) as the foundational middleware. Every component concept must be designed as a ROS 2 node with proper topics, services, and actions. Implementation must follow rclpy patterns for Python integration.

### III. Simulation-to-Reality (Sim2Real)
All robotics development starts in simulation environments (Gazebo, Unity, Isaac Sim) before real-world deployment. Every concept must be demonstrable in simulation first, with clear path to physical implementation. Content must support both cloud and local deployment scenarios. Synthetic data generation and photorealistic simulation required for training.

### IV. Multi-Platform Compatibility
Support NVIDIA Isaac platform, Gazebo physics simulation, and Unity visualization environments. Content must be designed for cross-platform deployment between simulation and real hardware. Standardized interfaces required for platform migration. Content must be adaptable to different hardware configurations (cloud vs. local workstations).

### V. Vision-Language-Action Integration
Focus on convergence of LLMs, computer vision, and robotic action. Natural language ("Clean the room") must translate to ROS 2 action sequences. Voice-to-action capabilities using OpenAI Whisper integration required for capstone projects. Multi-modal interaction: speech, gesture, vision integration essential.

### VI. Humanoid-Centric Design
All robot design and control systems must prioritize humanoid form factors and human-centered environments. Bipedal locomotion, balance control, and natural human-robot interaction must be fundamental to all implementations.

### VII. Conversational AI Integration
Content must integrate GPT models for conversational AI in robots. Speech recognition and natural language understanding must be taught as core components. Multi-modal interaction combining speech, gesture, and vision required for comprehensive human-robot interaction.

### VIII. Hardware-Aware Content Delivery
Content must acknowledge computational demands of Physics Simulation, Visual Perception, and Generative AI simultaneously. Material must be structured for different hardware configurations: high-performance workstations with RTX GPUs for local development vs. cloud-based instances. Content must support both simulation and real-world deployment with clear pathways between them.

## Additional Constraints

Content requirements: All chapters must include practical examples using ROS 2, Python, NVIDIA Isaac SDK, Gazebo, Unity, OpenAI Whisper, and LLMs
Educational standards: Follow ROS 2 best practices and safety protocols for humanoid robots
Pedagogical approach: Simulated examples required before real-world application concepts
Hardware considerations: Content must address RTX-enabled workstations vs. cloud-based delivery with different performance characteristics
Deployment scenarios: Content must support both local (Ubuntu 22.04) and cloud (AWS/Azure) environments

## Development Workflow

Content review requirements must verify: Physical AI integration, ROS 2 compliance, Simulation examples, Humanoid applicability
Educational gates: All content must demonstrate functionality in simulation before advancing to real-world concepts
Quality standards: Natural human-robot interaction capabilities required for all capstone concepts
Hardware awareness: Content must address computational demands and different deployment scenarios
Assessment alignment: Content must support ROS 2 package development, Gazebo simulation, Isaac pipeline, and capstone humanoid robot projects

## Governance

This constitution supersedes all other practices for the Physical AI & Humanoid Robotics textbook project. All content must demonstrate connection between digital AI and physical robot control. Amendments require documentation of impact on Physical AI principles and embodied intelligence focus.

All PRs/reviews must verify compliance with Physical AI principles; Complexity must be justified by real-world robot application; Use CLAUDE.md for runtime development guidance.

**Version**: 1.2.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14