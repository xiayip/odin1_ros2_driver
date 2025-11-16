#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <mutex>

namespace odin1_ros2_driver
{

class IncrementalMapNode : public rclcpp::Node
{
public:
  explicit IncrementalMapNode(const rclcpp::NodeOptions & options);
  ~IncrementalMapNode() = default;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  void accumulateAndPublishIncremental(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
    const std_msgs::msg::Header & header);

  // ROS communication
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr incremental_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_pub_;

  // PCL data structures
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_map_;
  std::unique_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>> octree_;
  
  // Parameters
  double voxel_leaf_size_;
  double octree_resolution_;
  double novelty_threshold_;
  int max_accumulated_points_;

  // Thread safety
  std::mutex map_mutex_;
  
  // Statistics
  size_t total_points_received_{0};
  size_t total_incremental_points_{0};
};

}  // namespace odin1_ros2_driver
