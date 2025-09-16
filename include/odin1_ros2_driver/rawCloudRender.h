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
#ifndef RGBCLOUD_H
#define RGBCLOUD_H

#include <vector>
#include <string>
#include <Eigen/Dense>
#include "lidar_api.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <sensor_msgs/msg/point_field.hpp>
#include <yaml-cpp/yaml.h>

class rawCloudRender
{
public:
bool init(const std::string &yamlFilePath)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(yamlFilePath);
    }
    catch (const YAML::BadFile &e)
    {
        std::cerr << "Error: Could not open file '" << yamlFilePath << "' - " << e.what() << std::endl;
        return false;
    }
    catch (const YAML::ParserException &e)
    {
        std::cerr << "Error: YAML parsing failed - " << e.what() << std::endl;
        return false;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Unexpected error: " << e.what() << std::endl;
        return false;
    }

    // Intrinsic parameters node (cam_0)
    const std::string cam_node_name = "cam_0";
    if (!config[cam_node_name])
    {
        std::cerr << "Error: Missing camera node '" << cam_node_name << "'" << std::endl;
        return false;
    }
    YAML::Node cam_node = config[cam_node_name];

    // Directly access Tcl_0 node
    const std::string tcl_node_name = "Tcl_0";
    if (!config[tcl_node_name])
    {
        std::cerr << "Error: Missing transformation matrix node '" << tcl_node_name << "'" << std::endl;
        return false;
    }
    YAML::Node tclNode = config[tcl_node_name];
    if (tclNode.size() != 16)
    {
        std::cerr << "Error: Transformation matrix must be 4x4 (16 elements)" << std::endl;
        return false;
    }

    // === Read intrinsic parameters ===
    fy_ = cam_node["k2"].as<float>();
    k3_ = cam_node["k3"].as<float>();
    k4_ = cam_node["k4"].as<float>();
    k5_ = cam_node["k5"].as<float>();
    k6_ = cam_node["k6"].as<float>();
    k7_ = cam_node["k7"].as<float>();

    fx_ = cam_node["A11"].as<float>();
    skew_ = cam_node["A12"].as<float>();
    fy_ = cam_node["A22"].as<float>();

    cx_ = cam_node["u0"].as<float>();
    cy_ = cam_node["v0"].as<float>();

    // === Read extrinsic transformation matrix ===
    T_camera_lidar_ << tclNode[0].as<float>(), tclNode[1].as<float>(), tclNode[2].as<float>(), tclNode[3].as<float>(),
        tclNode[4].as<float>(), tclNode[5].as<float>(), tclNode[6].as<float>(), tclNode[7].as<float>(),
        tclNode[8].as<float>(), tclNode[9].as<float>(), tclNode[10].as<float>(), tclNode[11].as<float>(),
        tclNode[12].as<float>(), tclNode[13].as<float>(), tclNode[14].as<float>(), tclNode[15].as<float>();

    // === Display key parameters concisely ===
    std::cout << "=== Camera Calibration Parameters ===" << std::endl;
    std::cout << "Intrinsics:" << std::endl;
    std::cout << "  fx: " << fx_
                << ", fy: " << fy_
                << ", cx: " << cx_
                << ", cy: " << cy_ << std::endl;
    std::cout << "Distortion: k2=" << k2_
                << ", k3=" << k3_ << std::endl;

    Eigen::Vector3f translation = T_camera_lidar_.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation = T_camera_lidar_.block<3, 3>(0, 0);
    std::cout << "Extrinsics:" << std::endl;
    std::cout << "  Translation: [" << translation.x() << ", "
                << translation.y() << ", " << translation.z() << "]" << std::endl;
    std::cout << "  Rotation (euler angles): "
                << rotation.eulerAngles(0, 1, 2).transpose() * 180 / M_PI << "°" << std::endl;

    return true;
}

void render(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
            sensor_msgs::msg::PointCloud2::SharedPtr &output_cloud)
{
    // Verify input image format
    if (image_msg->encoding != "bgr8")
    {
        std::cerr << "Unsupported image format: " << image_msg->encoding << ". Only bgr8 is supported." << std::endl;
        return;
    }

    const Eigen::Matrix4f &T = T_camera_lidar_;

    // Precompute matrix elements
    const float T00 = T(0, 0), T01 = T(0, 1), T02 = T(0, 2), T03 = T(0, 3);
    const float T10 = T(1, 0), T11 = T(1, 1), T12 = T(1, 2), T13 = T(1, 3);
    const float T20 = T(2, 0), T21 = T(2, 1), T22 = T(2, 2), T23 = T(2, 3);

    // Initialize distortion lookup table
    static std::array<float, 10000> dist_table;
    [&]()
    {
        for (size_t i = 0; i < dist_table.size(); ++i)
        {
            float theta = i * (M_PI / 2) / dist_table.size();
            dist_table[i] = theta * (1 + theta * (k2_ + theta * (k3_ + theta * (k4_ + theta * (k5_ + theta * (k6_ + theta * k7_))))));
        }
        return true;
    }();

    // Get image data
    const uint8_t *bgr_data = image_msg->data.data();
    const int img_width = image_msg->width;
    const int img_height = image_msg->height;
    const int img_step = image_msg->step;

    // Prepare output cloud with maximum possible size
    output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    output_cloud->header.frame_id = "map";
    output_cloud->header.stamp = image_msg->header.stamp;
    output_cloud->height = 1;
    output_cloud->width = cloud_msg->width * cloud_msg->height; // Start with max size

    sensor_msgs::PointCloud2Modifier modifier(*output_cloud);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);

    // Create iterators for direct writing
    sensor_msgs::PointCloud2Iterator<float> out_x(*output_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(*output_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(*output_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> out_rgb(*output_cloud, "rgb");

    // Process each point in the cloud and write directly to output
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    size_t valid_points = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        // Get point coordinates
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;

        // Skip invalid points
        if (std::abs(x) < 1e-5f && std::abs(y) < 1e-5f && std::abs(z) < 1e-5f)
        {
            continue;
        }

        // Transform point to camera frame
        const float x1 = T00 * x + T01 * y + T02 * z + T03;
        const float y1 = T10 * x + T11 * y + T12 * z + T13;
        const float z1 = T20 * x + T21 * y + T22 * z + T23;

        // Skip points behind camera
        if (z1 <= 0.0f)
            continue;

        // Calculate projection with distortion
        const float dist = std::sqrt(x1 * x1 + y1 * y1);
        if (dist < 1e-7f)
            continue;

        const float norm = std::sqrt(x1 * x1 + y1 * y1 + z1 * z1);
        if (norm < 1e-7f)
            continue;

        const float cost = z1 / norm;
        const float theta = std::acos(cost);

        // Lookup distortion correction
        const size_t table_idx = static_cast<size_t>(theta * (2.0f / M_PI) * dist_table.size());
        const size_t safe_idx = std::min(table_idx, dist_table.size() - 1);
        const float thetad = dist_table[safe_idx];

        const float scaling = thetad / dist;
        const float xd = x1 * scaling;
        const float yd = y1 * scaling;

        // Project to image coordinates
        const float u_f = xd * fx_ + yd * skew_ + cx_;
        const float v_f = yd * fy_ + cy_;

        const int u = static_cast<int>(u_f);
        const int v = static_cast<int>(v_f);

        // Check if projection is within image bounds
        if (u < 0 || u >= img_width || v < 0 || v >= img_height)
        {
            continue;
        }

        // Get RGB color from image
        const int pixel_idx = v * img_step + u * 3;
        const uint8_t b = bgr_data[pixel_idx];
        const uint8_t g = bgr_data[pixel_idx + 1];
        const uint8_t r = bgr_data[pixel_idx + 2];

        // Pack RGB as float
        const uint32_t rgb_int = (r << 16) | (g << 8) | b;
        float rgb_float;
        std::memcpy(&rgb_float, &rgb_int, sizeof(float));

        // Write directly to output cloud
        *out_x = x;
        ++out_x;
        *out_y = y;
        ++out_y;
        *out_z = z;
        ++out_z;
        *out_rgb = rgb_float;
        ++out_rgb;
        valid_points++;
    }

    // Resize to actual number of valid points
    output_cloud->width = valid_points;
    modifier.resize(valid_points);
}

private:
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    float skew_;
    float k2_;
    float k3_;
    float k4_;
    float k5_;
    float k6_;
    float k7_;
    Eigen::Matrix4f T_camera_lidar_;
};

#endif // RGBCLOUD_H
