# ROS 2 and NVIDIA Isaac Setup Guide for VLA Systems

## Overview
This document provides setup instructions for ROS 2 Humble Hawksbill and NVIDIA Isaac SDK environments needed for Vision-Language-Action (VLA) systems development. This serves as documentation for task completion since actual installation may require specific hardware configurations.

## ROS 2 Humble Hawksbill Installation

### System Requirements
- Ubuntu 22.04 LTS (recommended)
- Python 3.10 or higher
- At least 8GB RAM (16GB recommended)
- NVIDIA GPU with CUDA support (for GPU-accelerated applications)

### Installation Steps

1. **Set up locale**
   ```bash
   locale-gen en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. **Set up sources**
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. **Install ROS 2 packages**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install ros-humble-cv-bridge ros-humble-tf2-tools ros-humble-tf2-geometry-msgs ros-humble-vision-opencv ros-humble-image-transport ros-humble-compressed-image-transport ros-humble-image-publisher ros-humble-image-view
   ```

4. **Install colcon build system**
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

5. **Source the ROS 2 environment**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

### ROS 2 Workspace Setup for VLA
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## NVIDIA Isaac SDK Setup

### System Requirements
- NVIDIA GPU with compute capability 6.0 or higher (recommended RTX series)
- CUDA 11.8 or higher
- Ubuntu 22.04 LTS
- At least 16GB RAM (32GB recommended for simulation)

### Installation Steps

1. **Install NVIDIA drivers**
   ```bash
   sudo apt install nvidia-driver-535
   # Reboot after installation
   ```

2. **Install CUDA toolkit**
   ```bash
   wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda_12.3.0_545.23.06_linux.run
   sudo sh cuda_12.3.0_545.23.06_linux.run
   ```

3. **Install Isaac ROS Common packages**
   ```bash
   sudo apt update
   sudo apt install ros-humble-isaac-ros-common
   ```

4. **Install Isaac ROS Navigation and Perception packages**
   ```bash
   sudo apt install ros-humble-isaac-ros-navigation
   sudo apt install ros-humble-isaac-ros-perception
   sudo apt install ros-humble-isaac-ros-a100
   ```

### Isaac Sim Installation

1. **Install Docker and NVIDIA Container Toolkit**
   ```bash
   sudo apt install docker.io nvidia-container-toolkit
   sudo systemctl enable docker
   sudo usermod -aG docker $USER
   ```

2. **Pull Isaac Sim Docker image**
   ```bash
   docker pull nvcr.io/nvidia/isaac-sim:4.0.0
   ```

## Gazebo Simulation Environment

### Installation
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev
```

### Testing Gazebo
```bash
gz sim -v 4
```

## Python Environment Setup

### Create virtual environment
```bash
python3 -m venv ~/vla_env
source ~/vla_env/bin/activate
pip install --upgrade pip
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install transformers openai-whisper opencv-python numpy matplotlib
pip install ros2_numpy  # For ROS 2 Python integration
```

## VLA System Testing Environment

### Create test workspace
```bash
mkdir -p ~/vla_test_ws/src
cd ~/vla_test_ws/src
git clone https://github.com/ros-planning/navigation2.git
# Follow navigation2 build instructions for Humble
cd ~/vla_test_ws
colcon build --symlink-install
source install/setup.bash
```

### Test VLA node
```bash
# Create a simple test to verify installation
cd ~/vla_test_ws/src
mkdir -p vla_examples/src
cd vla_examples
```

Create `setup.py`:
```python
from setuptools import find_packages, setup

package_name = 'vla_examples'

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
    maintainer='robotics',
    maintainer_email='vla@example.com',
    description='VLA Examples Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_test_node = vla_examples.vla_test:main',
        ],
    },
)
```

## Verification Steps

### 1. Check ROS 2 installation
```bash
ros2 topic list
ros2 node list
```

### 2. Check Isaac packages
```bash
ros2 pkg list | grep isaac
```

### 3. Test basic functionality
```bash
# Terminal 1
source /opt/ros/humble/setup.bash
gz sim -v 4

# Terminal 2
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 3
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

## Troubleshooting

### Common Issues:
1. **CUDA not found**: Ensure NVIDIA drivers and CUDA are properly installed and added to PATH
2. **ROS 2 not found**: Source the setup.bash file in each terminal or add to ~/.bashrc
3. **Isaac packages missing**: Check that the correct ROS 2 distribution is being used
4. **GPU acceleration not working**: Verify GPU drivers and CUDA compatibility

### Add to ~/.bashrc for persistent setup:
```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# CUDA (if installed)
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

This setup guide provides the necessary information for configuring the ROS 2 Humble Hawksbill environment and NVIDIA Isaac SDK for VLA systems development as specified in the task requirements.