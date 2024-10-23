# Realtime Target Vehicle LiDAR Point Cloud Emulation

This repository contains the code and implementation for generating realistic and accurate emulated LiDAR point cloud data for one or more principal other vehicles (POVs). The project integrates these point clouds in real-time with the environment's point cloud and is designed to support testing of automated vehicles and active safety systems.

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Usage](#usage)
  - [Offline Point Cloud Generation](#offline-point-cloud-generation)
  - [Real-time Integration](#real-time-integration)
  - [Visualization with RVIZ](#visualization-with-rviz)
- [Contributing](#contributing)
- [Acknowledgments](#acknowledgments)
- [License](#license)

## Introduction
The aim of this project is to develop a methodology and software for generating realistic LiDAR point-cloud data for principal other vehicles (POVs) and integrating these with real-time environment point clouds based on ego-vehicle position and orientation. This system enables the testing of automated vehicle systems in dangerous or safety-critical scenarios by utilizing virtual "ghost" vehicles that interact with real vehicles.

## Features
- **Offline Ray Casting**: Uses CARLA simulator to generate a dataset of point clouds from various vantage points.
- **Real-Time ROS Integration**: Integrates the generated POV point clouds with environment point clouds based on real-time position and orientation of vehicles.
- **Occlusion Removal**: Ensures realistic point cloud generation by removing occluded points through frustum culling techniques.
- **RVIZ Visualization**: Provides real-time visualization of the emulated and real point clouds using RVIZ in ROS.
- **Efficient Data Storage**: Stores point cloud data in an optimized manner for efficient retrieval and transformation.

## System Requirements
- Ubuntu 18.04+ (for ROS support)
- Python 3.7+
- [CARLA Simulator](https://carla.org/)
- [ROS (Robot Operating System)](https://www.ros.org/)
- [Open3D](http://www.open3d.org/) for point cloud processing
- GPU with CUDA support (recommended for real-time visualization)
- [Anaconda](https://www.anaconda.com/distribution/) (optional, for environment management)

## Installation

### 1. Clone the repository
```bash
git clone https://github.com/your-repo/realtime-lidar-emulation.git
cd realtime-lidar-emulation
