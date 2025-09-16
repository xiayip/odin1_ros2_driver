/*
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "lidar_api_type.h"
#include "odin1_ros2_driver/rawCloudRender.h"

namespace odin1_ros2_driver
{
class Odin1Driver : public rclcpp::Node
{
public:
    explicit Odin1Driver(rclcpp::NodeOptions options);
    ~Odin1Driver();
    
    // Public method for graceful shutdown
    void closeDevice();

private:
    bool init();
    void loadParameters();
    void setupPublishers();
    void setupServices();
    bool setupSDKCallbacks();
    void setupMessageFilters();
    // sdk callbacks
    static void lidarDeviceCallbackStatic(const lidar_device_info_t* device, bool attach);
    void lidarDeviceCallback(const lidar_device_info_t* device, bool attach);
    static void lidarDataCallbackStatic(const lidar_data_t* data, void* user_data);
    void lidarDataCallback(const lidar_data_t* data, void* user_data);
    bool activeStream(bool active);
    // ros2 publish functions
    void publishRgb(capture_Image_List_t* data);
    void publishImu(icm_6aixs_data_t* stream);
    void publishOdometry(capture_Image_List_t* stream);
    void publishIntensityCloud(capture_Image_List_t* stream, int idx);
    void publishPC2XYZRGBA(capture_Image_List_t* stream, int idx);
    // fuse rgb and pointcloud
    void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);
    // service callbacks
    void streamControlCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // device
    device_handle odin_device_;
    int slam_mode_;
    bool stream_active_;
    // render
    std::shared_ptr<rawCloudRender> render_;
    // Parameters
    int streamctrl_, sendrgb_, sendimu_, sendodom_, senddtof_, sendcloudslam_, sendcloudrender_, sendrgbcompressed_, senddepth_, recorddata_;
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr xyzrgbacloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_rgb_pub_; // New compressed image publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rgbcloud_pub_;
    // Subscribers using message filters for synchronization
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
    // Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stream_control_service_;
    // 
};
} // namespace odin1_ros2_driver