# Odin1 ROS2 Driver Launch Files

This package provides several launch files for different use cases with the Odin1 ROS2 driver.

## Available Launch Files

### 1. odin1_ros2_driver.launch.py (Full-featured)
The main launch file with all configurable parameters.

**Usage:**
```bash
ros2 launch odin1_ros2_driver odin1_ros2_driver.launch.py
```

**Parameters:**
- `sendrgb` (default: 1): Enable/disable RGB image publishing
- `sendimu` (default: 1): Enable/disable IMU data publishing  
- `sendodom` (default: 1): Enable/disable odometry publishing
- `senddtof` (default: 1): Enable/disable depth/ToF data publishing
- `sendcloudslam` (default: 0): Enable/disable SLAM point cloud publishing
- `sendcloudrender` (default: 0): Enable/disable rendered point cloud publishing
- `sendrgbcompressed` (default: 0): Enable/disable compressed RGB publishing
- `senddepth` (default: 1): Enable/disable depth image publishing
- `recorddata` (default: 0): Enable/disable data recording
- `use_sim_time` (default: false): Use simulation time
- `log_level` (default: info): Log level (debug, info, warn, error, fatal)
- `node_name` (default: odin1_ros2_driver): Name of the driver node
- `namespace` (default: ''): Namespace for the driver node

**Example with custom parameters:**
```bash
ros2 launch odin1_ros2_driver odin1_ros2_driver.launch.py \
    sendrgb:=1 \
    sendimu:=1 \
    sendcloudslam:=1 \
    log_level:=debug \
    namespace:=odin1
```

### 2. odin1_composable.launch.py (High Performance)
Runs the driver as a composable node for better performance by reducing inter-process communication overhead.

**Usage:**
```bash
ros2 launch odin1_ros2_driver odin1_composable.launch.py
```

## Published Topics

When enabled, the driver publishes the following topics:

- `/imu/data` (sensor_msgs/Imu): IMU data
- `/camera/image_raw` (sensor_msgs/Image): RGB images
- `/camera/compressed` (sensor_msgs/CompressedImage): Compressed RGB images
- `/camera/depth/image_raw` (sensor_msgs/Image): Depth images
- `/cloud` (sensor_msgs/PointCloud2): Point cloud data
- `/rendered_cloud` (sensor_msgs/PointCloud2): Rendered point cloud
- `/slam_cloud` (sensor_msgs/PointCloud2): SLAM point cloud
- `/odom` (nav_msgs/Odometry): Odometry data

## Hardware Requirements

- USB 3.0 or higher port required
- Compatible Odin1 LiDAR device
- Linux system with proper USB permissions

## Troubleshooting

1. **USB Version Error**: Make sure the device is connected to a USB 3.0+ port
2. **Permission Issues**: Ensure your user has access to USB devices
3. **Device Not Found**: Check if the device vendor/product IDs match (2207:0019)
4. **Performance Issues**: Use the composable launch file for better performance
