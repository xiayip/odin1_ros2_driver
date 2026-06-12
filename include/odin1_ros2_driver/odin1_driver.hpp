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

#include <deque>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "lidar_api_type.h"
#include "odin1_ros2_driver/rawCloudRender.h"
#include "odin1_ros2_driver/srv/save_map.hpp"

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
    bool checkUsbConnection();
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
    void publishImu(imu_convert_data_t* stream);
    void publishOdometry(capture_Image_List_t* stream);
    void publishPath(capture_Image_List_t* stream);
    void publishBaseToOdomTF(capture_Image_List_t* stream);
    void publishOdomToMapTF(capture_Image_List_t* stream);
    void publishStaticMapToOdomTF();
    // Look up (and cache) the static extrinsic T_base_sensor from the robot
    // description TF tree (e.g. base_footprint -> odin1_base_link). Returns
    // true when the cached transform is valid.
    bool lookupBaseToSensor();
    // Anchor the odom frame at the robot's initial base pose. The device
    // defines its odometry origin at the sensor's power-on pose (top of the
    // robot); re-anchoring keeps odom at ground level. Also publishes the
    // static odom -> device_odom_frame edge so device-frame data (clouds,
    // path) stays consistent.
    void ensureOdomAnchor(const tf2::Transform& t_dev_base, const rclcpp::Time& stamp);
    // timestamp alignment (ported from official driver: device ts / host ros time / PTP-corrected)
    rclcpp::Time makeAlignedStamp(uint64_t sensor_timestamp_ns);
    void updatePtpSync(const ptp_sync_data_t* ptp_data);
    void publishIntensityCloud(capture_Image_List_t* stream, int idx);
    void publishPC2XYZRGBA(capture_Image_List_t* stream, int idx);
    // fuse rgb and pointcloud
    void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);
    // service callbacks
    void streamControlCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void saveMapCallback(const std::shared_ptr<odin1_ros2_driver::srv::SaveMap::Request> request,
                         std::shared_ptr<odin1_ros2_driver::srv::SaveMap::Response> response);

    // device
    device_handle odin_device_;
    int slam_mode_;
    bool stream_active_;
    // render
    std::shared_ptr<rawCloudRender> render_;
    // Parameters
    int streamctrl_, sendrgb_, sendimu_, sendodom_, senddtof_, sendcloudslam_, sendcloudrender_, sendrgbcompressed_, senddepth_, recorddata_, custom_map_mode_, sendpath_;
    // Timestamp source: 0 = device timestamp, 1 = host ROS time, 2 = device timestamp with smoothed PTP offset
    int use_host_ros_time_;
    std::string relocalization_map_abs_path_;
    // When non-empty (e.g. "base_footprint"), the driver looks up the mounting
    // extrinsic tf_base_frame -> sensor_frame from TF and publishes
    // odom -> tf_base_frame instead of odom -> sensor_frame, so the robot's
    // URDF TF tree stays a single tree without duplicate parents.
    std::string tf_base_frame_;
    std::string sensor_frame_;
    // PTP/NTP time-sync smoothing state (updated from LIDAR_DT_NTP stream)
    static constexpr size_t PTP_SMOOTH_WINDOW_SIZE = 300;
    std::mutex ptp_mutex_;
    std::deque<double> ptp_delay_buf_;
    std::deque<double> ptp_offset_buf_;
    std::atomic<double> ptp_delay_smooth_{0.0};
    std::atomic<double> ptp_offset_smooth_{0.0};
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr xyzrgbacloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_rgb_pub_; // New compressed image publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rgbcloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    // Accumulated trajectory poses for the path
    std::vector<geometry_msgs::msg::PoseStamped> path_poses_;
    // tf_broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    // TF listener for the mounting extrinsic (only created when tf_base_frame_ is set)
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool have_base_to_sensor_{false};
    tf2::Transform base_to_sensor_; // T_base_sensor (mounting extrinsic)
    // Odom re-anchoring state (used when tf_base_frame_ is set)
    std::string device_odom_frame_{"odin1_odom"}; // device-native odometry frame
    bool have_odom_anchor_{false};
    tf2::Transform odom_anchor_;     // B0 = T_devodom_base at first odometry
    tf2::Transform odom_anchor_inv_; // B0^-1
    // Subscribers using message filters for synchronization
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
    // Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stream_control_service_;
    rclcpp::Service<odin1_ros2_driver::srv::SaveMap>::SharedPtr save_map_service_;
};
} // namespace odin1_ros2_driver