---
title: Deployment Scenarios: Cloud vs. Local with Isaac
description: Different deployment scenarios for Isaac-based robotic systems comparing cloud and local options
sidebar_position: 6
---

# Deployment Scenarios: Cloud vs. Local with Isaac

## Learning Objectives

- Understand different deployment scenarios for Isaac-based robotic systems
- Analyze the trade-offs between cloud and local deployment
- Recognize the requirements for RTX-enabled workstations
- Evaluate best practices for deployment in different environments

## Introduction

This chapter explores the various deployment scenarios available for NVIDIA Isaac™-based robotic systems, examining the differences between cloud-based and local deployment options. Following the hardware-aware content approach required by the project constitution, we'll analyze the requirements and considerations for different deployment environments.

## Key Concepts

### Local Deployment with RTX GPUs

Local deployment scenarios include:
- Development workstations with RTX GPUs for simulation and testing
- Edge computing solutions with Jetson platforms
- On-premise robot control systems
- High-performance computing clusters for training
- Real-time processing requirements for robotics applications

### Cloud Deployment Options

Cloud deployment options encompass:
- NVIDIA GPU Cloud (NGC) for containerized Isaac applications
- AWS RoboMaker with GPU instances
- Azure IoT Edge with GPU support
- Google Cloud with Compute Engine GPUs
- Multi-cloud strategies for redundancy and performance

### Performance Considerations

Performance considerations for deployment include:
- Latency requirements for real-time robot control
- Bandwidth constraints for sensor data transmission
- Compute resource allocation for AI workloads
- Power consumption for mobile robots
- Security and privacy requirements

## Practical Examples

### Example 1: Local Deployment Setup
```bash
# Install Isaac ROS on local RTX workstation
sudo apt update
sudo apt install ros-humble-isaac-ros-common
docker run --gpus all -it nvcr.io/nvidia/isaac-ros:latest
```

### Example 2: Cloud Deployment
```bash
# Deploy Isaac application to cloud with GPU
kubectl apply -f isaac-ros-gpu-deployment.yaml
```

## Connection to Modules 1 & 2

This chapter builds upon the hardware-aware content approach from the project constitution, connecting to the computational requirements discussed in Modules 1 and 2. The deployment scenarios address the RTX-enabled workstation vs. cloud-based delivery concepts introduced in previous modules, providing practical guidance for different implementation contexts.

## Exercises

1. Compare the performance of local RTX deployment versus cloud GPU deployment for a specific Isaac application.
2. Design a deployment strategy that combines local and cloud resources for optimal performance.

## Summary

This chapter explored various deployment scenarios for NVIDIA Isaac™-based robotic systems, examining the trade-offs between cloud and local deployment options. The considerations for RTX-enabled workstations and cloud platforms provide practical guidance for implementing Isaac-based solutions in different environments while maintaining the performance requirements for AI-based robotics applications.