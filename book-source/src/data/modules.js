// Module data structure for the Physical AI & Humanoid Robotics textbook

export const modules = [
  {
    id: 'module-1',
    title: 'Module 1: Robotic Nervous System (ROS 2)',
    description: 'Introduction to ROS 2 fundamentals and robotic communication systems',
    position: 1,
    chapterCount: 4,
    path: '/docs/ros2-fundamentals',
    difficulty: 'Beginner',
    duration: '3 weeks',
    prerequisites: 'Programming',
    items: [
      'ros2-fundamentals',
      'ros2-nodes-topics-services',
      'rclpy-integration',
      'urdf-humanoids'
    ],
    learningObjectives: [
      'Understand ROS 2 architecture and communication patterns',
      'Implement robotic systems using ROS 2',
      'Integrate sensors and actuators with ROS 2',
      'Design humanoid robot control systems'
    ]
  },
  {
    id: 'module-2',
    title: 'Module 2: Digital Twin (Gazebo & Unity)',
    description: 'Simulation environments for robotic development and testing',
    position: 2,
    chapterCount: 4,
    path: '/docs/gazebo-simulation',
    difficulty: 'Intermediate',
    duration: '4 weeks',
    prerequisites: 'Module 1',
    items: [
      'gazebo-simulation',
      'unity-visualization',
      'sensor-simulation',
      'physics-collision-modeling'
    ],
    learningObjectives: [
      'Create realistic simulation environments for robotics',
      'Implement physics-based simulation models',
      'Validate robotic systems in simulation',
      'Transfer knowledge from simulation to real robots'
    ]
  },
  {
    id: 'module-3',
    title: 'Module 3: AI-Robot Brain (NVIDIA Isaac™)',
    description: 'AI integration and cognitive systems for robotics',
    position: 3,
    chapterCount: 6,
    path: '/docs/nvidia-isaac-platform',
    difficulty: 'Advanced',
    duration: '6 weeks',
    prerequisites: 'Module 1, Module 2',
    items: [
      'nvidia-isaac-platform',
      'isaac-ai-workflows',
      'robot-brain-integration',
      'isaac-simulation-environments',
      'ros2-isaac-integration',
      'deployment-scenarios'
    ],
    learningObjectives: [
      'Implement AI algorithms for robotic systems',
      'Integrate NVIDIA Isaac platform with robotic systems',
      'Design cognitive architectures for robots',
      'Deploy AI models on robotic platforms'
    ]
  },
  {
    id: 'module-4',
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Advanced systems integrating perception, understanding, and action',
    position: 4,
    chapterCount: 6,
    path: '/docs/introduction-to-vla-systems',
    difficulty: 'Advanced',
    duration: '6 weeks',
    prerequisites: 'Module 1, Module 2, Module 3',
    items: [
      'introduction-to-vla-systems',
      'vision-language-models',
      'action-planning',
      'multimodal-interaction',
      'vla-simulation-examples',
      'vla-ros-integration'
    ],
    learningObjectives: [
      'Understand Vision-Language-Action system architectures',
      'Implement multimodal perception systems',
      'Create action planning algorithms',
      'Integrate vision, language, and action in robotic systems'
    ]
  }
];

// Get module by ID
export const getModuleById = (id) => {
  return modules.find(module => module.id === id);
};

// Get module by position
export const getModuleByPosition = (position) => {
  return modules.find(module => module.position === position);
};

// Get all modules
export const getAllModules = () => {
  return modules;
};

// Get total chapter count across all modules
export const getTotalChapters = () => {
  return modules.reduce((total, module) => total + module.chapterCount, 0);
};