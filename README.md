# Odin1_ROS2_Driver

Modern ROS2 driver for Odin1 LIDAR/camera sensor modules with advanced composable node architecture and real-time incremental mapping.

## Overview

This ROS2 driver provides comprehensive support for Odin1 sensor modules, featuring:
- **Composable Node Architecture** - Efficient memory usage and flexible deployment
- **Incremental Point Cloud Mapping** - Real-time novelty detection and map accumulation
- **RGB-PointCloud Fusion** - Real-time fusion of camera and LIDAR data
- **Runtime Stream Control** - Service-based control of data streams
- **Multi-format Support** - Raw and compressed image publishing
- **SLAM Integration** - Odometry and SLAM point cloud support
- **Intelligent Color Handling** - Automatic RGB/Intensity-based point cloud coloring

**Compatibility:** ROS2 Humble (Ubuntu 22.04)

## Version

Current Version: v1.1.0

## Requirements

### Operating System
- Ubuntu 22.04 LTS (for ROS2 Humble)

### Dependencies
- **ROS2 Humble** - Core ROS2 framework
- **OpenCV >= 4.5.0** - Computer vision library
- **PCL (Point Cloud Library)** - Point cloud processing and octree spatial indexing
- **pcl_conversions** - PCL-ROS2 message conversion
- **pcl_ros** - ROS2 integration for PCL
- **yaml-cpp** - YAML configuration parsing  
- **Eigen3** - Linear algebra library
- **cv_bridge** - ROS2-OpenCV integration
- **message_filters** - Message synchronization
- **rclcpp_components** - Composable node support

## Installation

### 1. Install System Dependencies
```bash
sudo apt update
sudo apt install -y build-essential cmake git libgtk2.0-dev pkg-config \
    libavcodec-dev libavformat-dev libswscale-dev libyaml-cpp-dev \
    libusb-1.0-0-dev libopencv-dev libeigen3-dev \
    libpcl-dev ros-humble-pcl-conversions ros-humble-pcl-ros
```

### 2. Install ROS2 Humble
Follow the official ROS2 Humble installation guide:
[ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### 3. Setup USB Device Rules
```bash
sudo tee /etc/udev/rules.d/99-odin-usb.rules << EOF
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 4. Build the Package
```bash
# Navigate to your ROS2 workspace
cd /path/to/your/ros2_workspace

# Build the package
colcon build --packages-select odin1_ros2_driver

# Source the workspace
source install/setup.bash
```
## Usage

### Quick Start
```bash
# Launch the driver with default configuration
ros2 launch odin1_ros2_driver odin1_ros2_driver.launch.py rviz:=true

# Launch as composable node (recommended for production)
ros2 launch odin1_ros2_driver odin1_composable.launch.py rviz:=true

# Launch with incremental point cloud mapping (NEW!)
ros2 launch odin1_ros2_driver odin1_with_incremental_points.launch.py rviz:=true
```

### Runtime Control
The driver provides a service for runtime stream control:

```bash
# Start streaming
ros2 service call /odin1/stream_control std_srvs/srv/SetBool "data: true"

# Stop streaming
ros2 service call /odin1/stream_control std_srvs/srv/SetBool "data: false"
```

### Saving Map Service
To save the current SLAM map to a file, use the following service:
```bash
ros2 service call /odin1/save_map odin1_ros2_driver/srv/SaveMap "{name: {data: '/home/admin/map.bin'}}"
```

### Configuration
Edit `config/control_command.yaml` to customize driver parameters:
- Stream types (RGB, IMU, odometry, point clouds)
- Data publishing rates
- Calibration settings
## Project Structure

```
odin1_ros2_driver/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # ROS2 package manifest
├── README.md                   # This file
├── SDK/                        # Odin SDK libraries
│   ├── include/               # SDK headers
│   └── lib/                   # Platform-specific libraries
├── config/
│   └── control_command.yaml   # Driver configuration
├── description/
│   └── odin1_description.urdf # Robot description for TF frames
├── include/odin1_ros2_driver/
│   ├── odin1_driver.hpp       # Main driver class
│   ├── incremental_map_node.hpp  # Incremental mapping node (NEW)
│   ├── helper.hpp             # Utility functions
│   └── rawCloudRender.h       # Point cloud rendering
├── launch/
│   ├── odin1_ros2_driver.launch.py           # Standard node launch
│   ├── odin1_composable.launch.py            # Composable node launch
│   └── odin1_with_incremental_points.launch.py  # With mapping (NEW)
├── src/
│   ├── odin1_driver.cpp       # Main driver implementation
│   ├── odin1_driver_node.cpp  # Node executable
│   └── incremental_map_node.cpp  # Incremental mapping implementation (NEW)
├── srv/
│   └── SaveMap.srv            # Custom service for saving maps
└── rviz/                      # RVIZ configurations
    └── odin_ros2_Increment_pl.rviz  # Incremental mapping visualization (NEW)
```



## ROS2 Topics and Services

### Published Topics
| Topic Name | Message Type | Description |
|------------|--------------|-------------|
| `/odin1/imu` | `sensor_msgs/msg/Imu` | IMU sensor data |
| `/odin1/image` | `sensor_msgs/msg/Image` | Raw RGB camera images |
| `/odin1/image/compressed` | `sensor_msgs/msg/CompressedImage` | Compressed RGB images |
| `/odin1/cloud_raw` | `sensor_msgs/msg/PointCloud2` | Raw LIDAR point cloud |
| `/odin1/cloud_render` | `sensor_msgs/msg/PointCloud2` | RGB-fused point cloud |
| `/odin1/cloud_slam` | `sensor_msgs/msg/PointCloud2` | SLAM-processed point cloud |
| `/odin1/cloud_incremental` | `sensor_msgs/msg/PointCloud2` | **NEW**: Incremental points (novelty only) |
| `/odin1/cloud_accumulated` | `sensor_msgs/msg/PointCloud2` | **NEW**: Accumulated map (full history) |
| `/odin1/odometry` | `nav_msgs/msg/Odometry` | SLAM odometry data |

### Services
| Service Name | Service Type | Description |
|--------------|--------------|-------------|
| `/odin1/stream_control` | `std_srvs/srv/SetBool` | Start/stop data streaming |
| `/odin1/save_map` | `odin1_ros2_driver/srv/SaveMap` | Save current SLAM map to file |

### Parameters

#### Driver Parameters (config/control_command.yaml)
- `streamctrl`: Enable/disable stream control (default: 1)
- `sendrgb`: Publish RGB images (default: 1)  
- `sendimu`: Publish IMU data (default: 1)
- `sendodom`: Publish odometry (default: 1)
- `senddtof`: Publish raw point cloud (default: 1)
- `sendcloudslam`: Publish SLAM point cloud (default: 0)
- `sendcloudrender`: Enable RGB-point cloud fusion (default: 0)

#### Incremental Mapping Parameters **NEW**
Configure via launch file or command line:
- `voxel_leaf_size` (double): Voxel grid size for downsampling (default: 0.01m)
- `octree_resolution` (double): Octree spatial index resolution (default: 0.02m)  
- `novelty_threshold` (double): Distance threshold for novelty detection (default: 0.02m)
- `max_accumulated_points` (int): Max points before auto-downsample (default: 1,000,000)

## Troubleshooting

### Common Issues

#### Device Connection Problems
**Symptom**: "No device connected" error  
**Solution**: 
1. Verify USB 3.0+ connection (USB 2.0 not supported)
2. Check device power and reconnect USB
3. Verify udev rules are properly installed
