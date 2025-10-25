#include "odin1_ros2_driver/odin1_driver.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <sensor_msgs/msg/point_field.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "lidar_api.h"
#include "odin1_ros2_driver/helper.hpp"

#include <signal.h>
#include <memory>
#include <thread>
#include <chrono>
#include "std_srvs/srv/set_bool.hpp"

namespace odin1_ros2_driver
{

// Static instance pointer for callback handling
static Odin1Driver *g_driver_instance = nullptr;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"), "Interrupt signal (%d) received. Shutting down gracefully...", signum);
    if (g_driver_instance) {
        g_driver_instance->closeDevice();
    }
    rclcpp::shutdown();
    exit(signum);
}

Odin1Driver::Odin1Driver(rclcpp::NodeOptions options)
    : Node("odin1_ros2_driver", options), odin_device_(nullptr), slam_mode_(LIDAR_MODE_SLAM), stream_active_(false)
{
    g_driver_instance = this; // Set global instance for static callback
    
    // Register signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    init();
    RCLCPP_INFO(this->get_logger(), "Odin1 ROS2 Driver Node has been started.");
}

Odin1Driver::~Odin1Driver()
{
    closeDevice();
    g_driver_instance = nullptr; // Clear global instance
    RCLCPP_INFO(this->get_logger(), "Odin1 ROS2 Driver Node is shutting down.");
}

bool Odin1Driver::init()
{
    loadParameters();
    setupPublishers();
    setupServices();
    setupSDKCallbacks();
    setupMessageFilters();
    RCLCPP_INFO(this->get_logger(), "Publishers initialized.");
    return true;
}

void Odin1Driver::loadParameters()
{
    this->declare_parameter("streamctrl", 1);
    this->declare_parameter("sendrgb", 1);
    this->declare_parameter("sendimu", 1);
    this->declare_parameter("sendodom", 1);
    this->declare_parameter("senddtof", 1);
    this->declare_parameter("sendcloudslam", 0);
    this->declare_parameter("sendcloudrender", 0);
    this->declare_parameter("sendrgbcompressed", 0);
    this->declare_parameter("senddepth", 0);
    this->declare_parameter("recorddata", 0);

    this->get_parameter("streamctrl", streamctrl_);
    this->get_parameter("sendrgb", sendrgb_);
    this->get_parameter("sendimu", sendimu_);
    this->get_parameter("sendodom", sendodom_);
    this->get_parameter("senddtof", senddtof_);
    this->get_parameter("sendcloudslam", sendcloudslam_);
    this->get_parameter("sendcloudrender", sendcloudrender_);
    this->get_parameter("sendrgbcompressed", sendrgbcompressed_);
    this->get_parameter("senddepth", senddepth_);
    this->get_parameter("recorddata", recorddata_);

    RCLCPP_INFO(this->get_logger(), "Parameters loaded, streamctrl: %d, sendrgb: %d, sendimu: %d, sendodom: %d, senddtof: %d, sendcloudslam: %d, sendcloudrender: %d, sendrgbcompressed: %d, senddepth: %d, recorddata: %d",
                streamctrl_, sendrgb_, sendimu_, sendodom_, senddtof_, sendcloudslam_, sendcloudrender_, sendrgbcompressed_, senddepth_, recorddata_);
}

void Odin1Driver::setupPublishers()
{
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("odin1/imu", 10);
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("odin1/image", 10);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("odin1/cloud_raw", 10);
    xyzrgbacloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("odin1/cloud_slam", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odin1/odometry", 10);
    compressed_rgb_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("odin1/image/compressed", 10);
    RCLCPP_INFO(this->get_logger(), "Publishers setup completed.");
}

void Odin1Driver::setupServices()
{
    stream_control_service_ = this->create_service<std_srvs::srv::SetBool>(
        "odin1/stream_control",
        std::bind(&Odin1Driver::streamControlCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Services setup completed.");
}

bool Odin1Driver::setupSDKCallbacks()
{
    lidar_log_set_level(LIDAR_LOG_DEBUG);
    if (lidar_system_init(&Odin1Driver::lidarDeviceCallbackStatic))
    {
        RCLCPP_ERROR(this->get_logger(), "Lidar system init failed");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Callbacks setup completed.");
    return true;
}

void Odin1Driver::setupMessageFilters()
{
    if (!sendcloudrender_)
        return;
    // init renderer
    std::string config_dir = ament_index_cpp::get_package_share_directory("odin1_ros2_driver") + "/config";
    std::string calib_file = config_dir + "/calib.yaml";
    if (std::filesystem::exists(calib_file))
    {
        render_ = std::make_shared<rawCloudRender>();
        if (!render_->init(calib_file))
        {
            RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Failed to initialize point cloud renderer");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Point cloud renderer initialized");
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("device_cb"), "Renderer config file not found: %s", calib_file.c_str());
        return;
    }
    // Create subscribers with message filters
    image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "odin1/image");
    cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, "odin1/cloud_raw");
    // Create synchronizer with proper policy initialization
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *image_sub_, *cloud_sub_);
    sync_->setAgePenalty(0.5);
    sync_->registerCallback(std::bind(&Odin1Driver::syncCallback, this, std::placeholders::_1, std::placeholders::_2));
    // Create publisher for RGB point cloud
    rgbcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("odin1/cloud_render", 10);
}

// Static callback function for C API
void Odin1Driver::lidarDeviceCallbackStatic(const lidar_device_info_t *device, bool attach)
{
    if (g_driver_instance)
    {
        g_driver_instance->lidarDeviceCallback(device, attach);
    }
}

void Odin1Driver::lidarDeviceCallback(const lidar_device_info_t *device, bool attach)
{
    if (!device)
    {
        RCLCPP_WARN(this->get_logger(), "Received null device info in lidarDeviceCallback.");
        return;
    }

    if (attach == true)
    {
        RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Hardware connected, starting software connection...");
        if (!isUsb3OrHigher(TARGET_VENDOR, TARGET_PRODUCT))
        {
            RCLCPP_FATAL(rclcpp::get_logger("device_cb"),
                            "Device connected to USB 2.0 port. This device requires USB 3.0 or higher. Exiting program.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Device connected to USB 3.0 or higher port.");

        if (lidar_create_device(const_cast<lidar_device_info_t *>(device), &odin_device_))
        {
            RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Create device failed");
            return;
        }

        if (lidar_open_device(odin_device_))
        {
            RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Open device failed");
            lidar_destory_device(odin_device_);
            odin_device_ = nullptr;
            return;
        }

        std::string config_dir = ament_index_cpp::get_package_share_directory("odin1_ros2_driver") + "/config";
        RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Calibration files will be saved to: %s", config_dir.c_str());
        
        if (lidar_get_version(odin_device_))
        {
            RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Get version failed");
            closeDevice();
            return;
        }
        
        if (lidar_get_calib_file(odin_device_, config_dir.c_str()))
        {
            RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Failed to get calibration file");
            closeDevice();
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Successfully retrieved calibration files");

        if (lidar_set_mode(odin_device_, slam_mode_))
        {
            RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Set mode failed");
            closeDevice();
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Software connection successful");

        lidar_data_callback_info_t data_callback_info;
        data_callback_info.data_callback = lidarDataCallbackStatic;
        data_callback_info.user_data = &odin_device_;

        if (lidar_register_stream_callback(odin_device_, data_callback_info))
        {
            RCLCPP_ERROR(rclcpp::get_logger("device"), "Register callback failed");
            closeDevice();
            return;
        }

        if (!activeStream(false))
        {
            RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Failed to activate stream");
            closeDevice();
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Stream ready to be activated");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Device detaching...");
        // clear_all_queues();
        RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Waiting for device reconnection...");
    }
}

// Static callback function for C API
void Odin1Driver::lidarDataCallbackStatic(const lidar_data_t *data, void *user_data) {
    if (g_driver_instance)
    {
        g_driver_instance->lidarDataCallback(data, user_data);
    }
}

void Odin1Driver::lidarDataCallback(const lidar_data_t *data, void *user_data) {
    device_handle *dev_handle = static_cast<device_handle *>(user_data);
    if(!dev_handle || !data) {
        RCLCPP_WARN(this->get_logger(), "Invalid device handle or data.");
        return;
    }
    // RCLCPP_INFO(this->get_logger(), "Received data packet of type %d", data->type);
    switch (data->type)
    {
    case LIDAR_DT_NONE:
        RCLCPP_WARN(this->get_logger(), "Received empty data packet");
        break;
    case LIDAR_DT_RAW_RGB:
        if (sendrgb_) {
            publishRgb((capture_Image_List_t *)&data->stream);
        }
        break;
    case LIDAR_DT_RAW_IMU:
        if (sendimu_) {
            imu_convert_data_t *imudata = (imu_convert_data_t *)data->stream.imageList[0].pAddr;
            publishImu(imudata);
        }
        break;
    case LIDAR_DT_RAW_DTOF:
        if (senddtof_) {
            publishIntensityCloud((capture_Image_List_t *)&data->stream, 1);
        }
        break;
    case LIDAR_DT_SLAM_CLOUD:
        if (sendcloudslam_) {
            publishPC2XYZRGBA((capture_Image_List_t *)&data->stream, 0);
        }
        break;
    case LIDAR_DT_SLAM_ODOMETRY:
        // if (sendodom_) {
        //     publishOdometry((capture_Image_List_t *)&data->stream);
        // }
        break;
    case LIDAR_DT_DEV_STATUS:
        // Handle device status if needed
        break;
    case LIDAR_DT_SLAM_ODOMETRY_HIGHFREQ:
        if (sendodom_) {
            publishOdometry((capture_Image_List_t *)&data->stream);
        }
        break;
    default:
        break;
    }
}

bool Odin1Driver::activeStream(bool active)
{
    if (active && !stream_active_)
    {
        RCLCPP_INFO(this->get_logger(), "Activating streams...");
        uint32_t dtof_subframe_odr = 0;
        if (lidar_start_stream(odin_device_, slam_mode_, dtof_subframe_odr) == 0)
        {
            stream_active_ = true;
            // Reactivate stream types based on parameters
            if (sendrgb_) {
                lidar_activate_stream_type(odin_device_, LIDAR_DT_RAW_RGB);
            }
            if (sendimu_) {
                lidar_activate_stream_type(odin_device_, LIDAR_DT_RAW_IMU);
            }
            if (sendodom_) {
                lidar_activate_stream_type(odin_device_, LIDAR_DT_SLAM_ODOMETRY);
            }
            if (senddtof_) {
                lidar_activate_stream_type(odin_device_, LIDAR_DT_RAW_DTOF);
            }
            if (sendcloudslam_) {
                lidar_activate_stream_type(odin_device_, LIDAR_DT_SLAM_CLOUD);
            }
            RCLCPP_INFO(this->get_logger(), "Stream activated successfully");
        }
    }
    else if (!active && stream_active_)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating streams...");
        lidar_stop_stream(odin_device_, slam_mode_);
        stream_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Stream deactivated successfully");
    } else if (active && stream_active_) {
        RCLCPP_INFO(this->get_logger(), "Stream already active");
    } else if (!active && !stream_active_) {
        RCLCPP_INFO(this->get_logger(), "Stream already inactive");
    }
    return true;
}

void Odin1Driver::streamControlCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                       std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (!odin_device_) {
        response->success = false;
        response->message = "Device not connected";
        RCLCPP_ERROR(this->get_logger(), "Stream control failed: Device not connected");
        return;
    }
    response->success = activeStream(request->data);
    response->message = response->success ? "Stream control successful" : "Stream control failed";
}

void Odin1Driver::closeDevice() {
    if (odin_device_)
    {
        RCLCPP_INFO(this->get_logger(), "Device closing...");
        lidar_stop_stream(odin_device_, slam_mode_);
        lidar_unregister_stream_callback(odin_device_);
        lidar_close_device(odin_device_);
        lidar_destory_device(odin_device_);
        lidar_system_deinit();
        odin_device_ = nullptr;
    }
}

void Odin1Driver::publishRgb(capture_Image_List_t *stream) {
    buffer_List_t &image = stream->imageList[0];
    // old version yuv data
    if (image.length == image.width * image.height * 3 / 2) {
        const int height_nv12 = image.height * 3 / 2;
        cv::Mat nv12_mat(height_nv12, image.width, CV_8UC1, image.pAddr);
        cv::Mat bgr;
        cv::cvtColor(nv12_mat, bgr, cv::COLOR_YUV2BGR_NV12);

        if (bgr.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert NV12 to BGR");
            return;
        }

        //Create ROS image message
        auto header = std::make_shared<std_msgs::msg::Header>();
        header->stamp = ns_to_ros_time(image.timestamp); // Offset compensation
        //RCLCPP_INFO(rclcpp::get_logger("device_cb"), "image rgb %ld",image.timestamp);
        header->frame_id = "camera_rgb_frame";
        auto cv_image = std::make_shared<cv_bridge::CvImage>(*header, "bgr8", bgr);
        auto msg = cv_image->toImageMsg();
        // Publish original image message
        rgb_pub_->publish(*msg);
        
        // Create compressed image message
        auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        compressed_msg->header = *header;
        compressed_msg->format = "jpeg";
        // Set compression parameters
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(80);
        
        // Compress image
        cv::imencode(".JPEG", bgr, compressed_msg->data, compression_params);
        compressed_rgb_pub_->publish(*compressed_msg);
    } else {// new version jpeg data
        std::vector<uint8_t> jpeg_data(static_cast<uint8_t*>(image.pAddr),
                                        static_cast<uint8_t*>(image.pAddr) + image.length);

        // convert back to bgr8                                                
        cv::Mat decoded_image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

        auto header = std::make_shared<std_msgs::msg::Header>();
        header->stamp = ns_to_ros_time(image.timestamp); // Offset compensation
        header->frame_id = "camera_rgb_frame";
        auto cv_image = std::make_shared<cv_bridge::CvImage>(*header, "bgr8", decoded_image);
        rgb_pub_->publish(*(cv_image->toImageMsg()));

        // original jpeg
        sensor_msgs::msg::CompressedImage jpeg_msg;
        jpeg_msg.header.stamp = ns_to_ros_time(stream->imageList[0].timestamp);
        jpeg_msg.format = "jpeg";
        jpeg_msg.data = jpeg_data;
        compressed_rgb_pub_->publish(jpeg_msg);
    }
}

void Odin1Driver::publishImu(imu_convert_data_t *stream) {
    sensor_msgs::msg::Imu imu_msg;
    
    imu_msg.header.stamp = ns_to_ros_time(stream->stamp);
    imu_msg.header.frame_id = "imu_link";
    
    imu_msg.linear_acceleration.y = -1 * stream->accel_x;
    imu_msg.linear_acceleration.x = stream->accel_y;
    imu_msg.linear_acceleration.z = stream->accel_z;

    imu_msg.angular_velocity.y = -1 * stream->gyro_x;
    imu_msg.angular_velocity.x = stream->gyro_y;
    imu_msg.angular_velocity.z = stream->gyro_z;
    
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    
    imu_pub_->publish(std::move(imu_msg));
}

void Odin1Driver::publishOdometry(capture_Image_List_t *stream) {
    auto msg = nav_msgs::msg::Odometry();
    msg.header.frame_id = "map";
    msg.child_frame_id = "base_link";

    uint32_t data_len = stream->imageList[0].length;
    if (data_len == sizeof(ros_odom_convert_complete_t)) {

        ros_odom_convert_complete_t* odom_data = (ros_odom_convert_complete_t*)stream->imageList[0].pAddr;
        msg.header.stamp = ns_to_ros_time(odom_data->timestamp_ns);

        msg.pose.pose.position.x = static_cast<double>(odom_data->pos[0]) / 1e6;
        msg.pose.pose.position.y = static_cast<double>(odom_data->pos[1]) / 1e6;
        msg.pose.pose.position.z = static_cast<double>(odom_data->pos[2]) / 1e6;

        msg.pose.pose.orientation.x = static_cast<double>(odom_data->orient[0]) / 1e6;
        msg.pose.pose.orientation.y = static_cast<double>(odom_data->orient[1]) / 1e6;
        msg.pose.pose.orientation.z = static_cast<double>(odom_data->orient[2]) / 1e6;
        msg.pose.pose.orientation.w = static_cast<double>(odom_data->orient[3]) / 1e6; // Problems here, orientation quaternion is not correct

        msg.twist.twist.linear.x = static_cast<double>(odom_data->linear_velocity[0]) / 1e6;
        msg.twist.twist.linear.y = static_cast<double>(odom_data->linear_velocity[1]) / 1e6;
        msg.twist.twist.linear.z = static_cast<double>(odom_data->linear_velocity[2]) / 1e6;

        msg.twist.twist.angular.x = static_cast<double>(odom_data->angular_velocity[0]) / 1e6;
        msg.twist.twist.angular.y = static_cast<double>(odom_data->angular_velocity[1]) / 1e6;
        msg.twist.twist.angular.z = static_cast<double>(odom_data->angular_velocity[2]) / 1e6;

        msg.pose.covariance = {
            static_cast<double>(odom_data->cov[0]) / 1e9, static_cast<double>(odom_data->cov[1]) / 1e9, static_cast<double>(odom_data->cov[2]) / 1e9, 0.0, 0.0, 0.0,
            static_cast<double>(odom_data->cov[3]) / 1e9, static_cast<double>(odom_data->cov[4]) / 1e9, static_cast<double>(odom_data->cov[5]) / 1e9, 0.0, 0.0, 0.0,
            static_cast<double>(odom_data->cov[6]) / 1e9, static_cast<double>(odom_data->cov[7]) / 1e9, static_cast<double>(odom_data->cov[8]) / 1e9, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, static_cast<double>(odom_data->cov[9]) / 1e9, static_cast<double>(odom_data->cov[10]) / 1e9, static_cast<double>(odom_data->cov[11]) / 1e9,
            0.0, 0.0, 0.0, static_cast<double>(odom_data->cov[12]) / 1e9, static_cast<double>(odom_data->cov[13]) / 1e9, static_cast<double>(odom_data->cov[14]) / 1e9,
            0.0, 0.0, 0.0, static_cast<double>(odom_data->cov[15]) / 1e9, static_cast<double>(odom_data->cov[16]) / 1e9, static_cast<double>(odom_data->cov[17]) / 1e9,
        };
        odom_publisher_->publish(std::move(msg));
    } 

}

void Odin1Driver::publishIntensityCloud(capture_Image_List_t* stream, int idx) {
    
    // Check index validity
    if (idx < 0 || idx >= 10) {
        RCLCPP_ERROR(this->get_logger(), "Invalid point cloud index: %d", idx);
        return;
    }
    // Check point cloud data validity
    buffer_List_t &cloud = stream->imageList[idx];
    if (!cloud.pAddr) {
        RCLCPP_ERROR(this->get_logger(), "Invalid point cloud: null data pointer at index %d", idx);
        return;
    }
    // Check point cloud dimensions
    if (cloud.width <= 0 || cloud.height <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid point cloud dimensions: %dx%d at index %d", 
                 cloud.width, cloud.height, idx);
        return;
    }

    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // Set message header
    msg->header.frame_id = "map";
    msg->header.stamp = ns_to_ros_time(cloud.timestamp);

    msg->height = cloud.height;
    msg->width = cloud.width;
    msg->is_dense = false;
    msg->is_bigendian = false;

    // Set point cloud fields
    sensor_msgs::PointCloud2Modifier modifier(*msg);
    modifier.setPointCloud2Fields(
        6,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::UINT8,
        "confidence", 1, sensor_msgs::msg::PointField::UINT16,
        "offset_time", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    modifier.resize(msg->height * msg->width);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(*msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_confidence(*msg, "confidence");
    sensor_msgs::PointCloud2Iterator<float> iter_offsettime(*msg, "offset_time");

    float* xyz_data_f = static_cast<float*>(cloud.pAddr);
    int total_points = cloud.height * cloud.width;
	//std::cout << stream->imageCount << std::endl;
    // float dtof_subframe_odr = getRosNodeControl()->getDtofSubframeODR() / 1000.0f;
    float dtof_subframe_odr = 0.0f;
    // printf("dtof_subframe_odr: %f\n", dtof_subframe_odr);
	    
    int valid_points = 0;
    if (stream->imageCount == 4) {

        uint8_t* intensity_data = static_cast<uint8_t*>(stream->imageList[2].pAddr);
        uint16_t* confidence_data = static_cast<uint16_t*>(stream->imageList[3].pAddr);
    
        for (int i = 0; i < total_points; ++i) {
            if (confidence_data[i] < 35) {
                continue;
            }
            // XYZ point
            *iter_x = xyz_data_f[i * 3 + 2] / 1000.0f; ++iter_x;
            *iter_y = -xyz_data_f[i * 3 + 0] / 1000.0f; ++iter_y;
            *iter_z = xyz_data_f[i * 3 + 1] / 1000.0f; ++iter_z;
            
            *iter_intensity = intensity_data[i]; ++iter_intensity;
            *iter_confidence = confidence_data[i]; ++iter_confidence;
            
            if (dtof_subframe_odr > 0.0) {
                int group = i / DTOF_NUM_ROW_PER_GROUP;
                float timestamp_offset = group * 1.0 / dtof_subframe_odr;

                *iter_offsettime = timestamp_offset;
                ++iter_offsettime;
            }

            valid_points++;
    	}
    } else {
        uint16_t* intensity_data = static_cast<uint16_t*>(stream->imageList[2].pAddr);
        
        for (int i = 0; i < total_points; ++i) {
            *iter_x = xyz_data_f[i * 4 + 2] / 1000.0f; ++iter_x;
            *iter_y = -xyz_data_f[i * 4 + 0] / 1000.0f; ++iter_y;
            *iter_z = xyz_data_f[i * 4 + 1] / 1000.0f; ++iter_z;
            
            float intensity = (intensity_data[i] - 10) * 255.0f / (12500 - 10);
            if (intensity > 255) {
                *iter_intensity = 255;
            } else if (intensity < 0) {
                *iter_intensity = 0;
            } else {
                *iter_intensity = static_cast<uint8_t>(intensity);
            }
            ++iter_intensity;
            
            *iter_confidence = 0;
            ++iter_confidence;
            
            valid_points++;
        }
    }

    // Publish point cloud
    cloud_pub_->publish(*msg);
}

void Odin1Driver::publishPC2XYZRGBA(capture_Image_List_t * stream, int idx) {

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ns_to_ros_time(stream->imageList[0].timestamp);

    //RCLCPP_INFO(rclcpp::get_logger("device_cb"), "Point cloudrgba %ld",stream->imageList[0].timestamp);

    size_t pt_size = sizeof(int32_t) * 3 + sizeof(int32_t) * 4;
    uint32_t points = stream->imageList[idx].length / pt_size;

    msg.height = 1;
    msg.width = points;
    msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    modifier.resize(msg.width * msg.height);

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_rgb(msg, "rgb");

    // Shared data processing logic
    int32_t* xyz_data = static_cast<int32_t*>(stream->imageList[idx].pAddr);

    for(uint32_t i = 0; i < points; i++) {
        int32_t* ptr = xyz_data + 7*i;
        
        *iter_x = static_cast<float>(ptr[0]) / 10000.0f; ++iter_x;
        *iter_y = static_cast<float>(ptr[1]) / 10000.0f; ++iter_y;
        *iter_z = static_cast<float>(ptr[2]) / 10000.0f; ++iter_z;
        
        uint8_t r = ptr[3] & 0xff;
        uint8_t g = ptr[4] & 0xff;
        uint8_t b = ptr[5] & 0xff;  
        
        uint32_t packed_rgb = (static_cast<uint32_t>(r) << 16) | 
                            (static_cast<uint32_t>(g) << 8)  | 
                            static_cast<uint32_t>(b);
        
        float rgb_float;
        std::memcpy(&rgb_float, &packed_rgb, sizeof(float));
        
        *iter_rgb = rgb_float; ++iter_rgb;
    }

    xyzrgbacloud_pub_->publish(std::move(msg));
}

void Odin1Driver::syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
    try {
        auto output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        render_->render(image_msg, cloud_msg, output_cloud);
        // Publish the fused cloud
        rgbcloud_pub_->publish(*output_cloud);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in RGB-PointCloud fusion: %s", e.what());
    }
}

} // namespace odin1_ros2_driver

RCLCPP_COMPONENTS_REGISTER_NODE(odin1_ros2_driver::Odin1Driver)