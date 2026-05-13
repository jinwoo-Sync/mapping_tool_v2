# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Voxel-SLAM is a complete, accurate, and versatile LiDAR-inertial SLAM (Simultaneous Localization and Mapping) system implemented in C++ using ROS (Robot Operating System). The system utilizes short-term, mid-term, long-term, and multi-map data associations with five core modules: initialization, odometry, local mapping, loop closure, and global mapping.

## Architecture

The system consists of two main packages:
- **VoxelSLAM**: Main SLAM implementation
- **VoxelSLAMPointCloud2**: Custom RViz plugin for point cloud visualization

### Core Components

- `voxelslam.cpp/hpp`: Main SLAM node implementation
- `BTC.cpp/BTC.h`: Binary Tree Construction module
- `ekf_imu.hpp`: Extended Kalman Filter for IMU integration
- `feature_point.hpp`: Point cloud feature extraction
- `loop_refine.hpp`: Loop closure detection and refinement
- `preintegration.hpp`: IMU preintegration
- `tools.hpp`: Utility functions
- `voxel_map.hpp`: Voxel-based map representation

## Build System

This is a ROS Noetic package using catkin build system:

### Prerequisites
- Ubuntu 20.04
- ROS Noetic
- PCL 1.10
- Eigen 3.3.7
- GTSAM 4.0.3
- livox_ros_driver

### Build Commands
```bash
# From workspace root (e.g., ~/catkin_ws)
catkin_make
source devel/setup.bash
```

### CMake Configuration
- C++14 standard
- Release build with `-O3` optimization
- Links against PCL, GTSAM, and ROS libraries

## Running the System

### Supported LiDAR Sensors
- Livox Avia: `roslaunch voxel_slam vxlm_avia.launch`
- Livox Mid360: `roslaunch voxel_slam vxlm_mid360.launch`
- Hesai: `roslaunch voxel_slam vxlm_hesai.launch`
- Ouster: `roslaunch voxel_slam vxlm_ouster.launch`
- Velodyne: `roslaunch voxel_slam vxlm_velodyne.launch`
- Avia Flying: `roslaunch voxel_slam vxlm_avia_fly.launch`

### Rosbag Playback
Use `--pause` flag for consistent timing:
```bash
rosbag play dataset.bag --pause
```

### Global Bundle Adjustment
After rosbag completion, trigger global mapping:
```bash
rosparam set finish true
```

## Configuration

Each sensor has its own YAML configuration in `config/` directory:
- `avia.yaml`, `mid360.yaml`, `hesai.yaml`, etc.

### Key Configuration Sections
- **General**: Topics, save paths, extrinsics, LiDAR type
- **Odometry**: Covariances, voxel sizes, filter parameters
- **LocalBA**: Local bundle adjustment parameters
- **Loop**: Loop closure detection parameters
- **GBA**: Global bundle adjustment parameters

### Multi-Session Configuration
For multi-session mapping (e.g., HILTI dataset):
1. Configure `save_path` for map storage
2. Set `previous_map` to load offline maps from previous sessions
3. Update `bagname` for current session
4. Enable `is_save_map: 1` to save current session

## Development Guidelines

### Code Style
- C++14 standard
- Header files use `.hpp` extension
- Implementation files use `.cpp` extension
- Follow existing naming conventions

### File Structure
- Main implementation in `VoxelSLAM/src/`
- Configuration files in `VoxelSLAM/config/`
- Launch files in `VoxelSLAM/launch/`
- RViz configurations in `VoxelSLAM/rviz_cfg/`

### Parameter Tuning
- Indoor environments: smaller voxel sizes (0.1-0.5m)
- Outdoor/high-speed: larger voxel sizes (2-4m)
- High altitude: increase altitude-specific parameters

### Testing Datasets
- Livox Avia: Campus elevator dataset for relocalization testing
- HILTI 2023: Multi-session handheld datasets
- MARS Dataset: Aerial drone sequences
- Mid360: High-speed motion datasets