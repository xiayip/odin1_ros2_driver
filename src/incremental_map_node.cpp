#include "odin1_ros2_driver/incremental_map_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace odin1_ros2_driver
{

IncrementalMapNode::IncrementalMapNode(const rclcpp::NodeOptions & options)
: Node("incremental_map_node", options)
{
  // Declare parameters
  this->declare_parameter("voxel_leaf_size", 0.05);
  this->declare_parameter("octree_resolution", 0.1);
  this->declare_parameter("novelty_threshold", 0.05);
  this->declare_parameter("max_accumulated_points", 1000000);

  // Get parameters
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  octree_resolution_ = this->get_parameter("octree_resolution").as_double();
  novelty_threshold_ = this->get_parameter("novelty_threshold").as_double();
  max_accumulated_points_ = this->get_parameter("max_accumulated_points").as_int();

  // Initialize PCL structures
  accumulated_map_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  octree_ = std::make_unique<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>>(octree_resolution_);
  octree_->setInputCloud(accumulated_map_);

  // Create QoS profile compatible with both intra-process and inter-process communication
  // Use RELIABLE reliability for intra-process, VOLATILE durability for zero-copy
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  // Create subscriber and publishers
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "odin1/cloud_slam",
    qos,
    std::bind(&IncrementalMapNode::cloudCallback, this, std::placeholders::_1));

  incremental_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "odin1/cloud_incremental",
    qos);

  accumulated_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "odin1/cloud_accumulated",
    qos);

  RCLCPP_INFO(this->get_logger(), "IncrementalMapNode initialized:");
  RCLCPP_INFO(this->get_logger(), "  - Voxel leaf size: %.3f", voxel_leaf_size_);
  RCLCPP_INFO(this->get_logger(), "  - Octree resolution: %.3f", octree_resolution_);
  RCLCPP_INFO(this->get_logger(), "  - Novelty threshold: %.3f", novelty_threshold_);
  RCLCPP_INFO(this->get_logger(), "  - Max accumulated points: %d", max_accumulated_points_);
}

void IncrementalMapNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert ROS message to PCL
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // Check if the message has RGB fields
  bool has_rgb = false;
  for (const auto& field : msg->fields) {
    if (field.name == "rgb" || field.name == "rgba") {
      has_rgb = true;
    }
  }

  if (has_rgb) {
    // Direct conversion if RGB is available
    // RCLCPP_INFO(this->get_logger(), "Received point cloud with RGB data, size: %zu", msg->data.size());
    pcl::fromROSMsg(*msg, *input_cloud);
  }

  if (input_cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
    return;
  }

  total_points_received_ += input_cloud->size();

  // Process and publish incremental points
  accumulateAndPublishIncremental(input_cloud, msg->header);
}

void IncrementalMapNode::accumulateAndPublishIncremental(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
  const std_msgs::msg::Header & header)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr incremental_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  incremental_cloud->header = input_cloud->header;

  // Apply voxel grid filter to input to reduce density
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_input(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (voxel_leaf_size_ > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.filter(*filtered_input);
  } else {
    *filtered_input = *input_cloud;
  }

  // Check each point for novelty
  for (const auto & point : filtered_input->points)
  {
    // Skip invalid points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    bool is_new = false;

    if (accumulated_map_->empty()) {
      is_new = true;
    } else {
      // Search for nearby points in the octree
      std::vector<int> point_idx_radius_search;
      std::vector<float> point_radius_squared_distance;

      if (octree_->radiusSearch(point, novelty_threshold_,
                                 point_idx_radius_search,
                                 point_radius_squared_distance) == 0)
      {
        is_new = true;
      }
    }

    if (is_new) {
      incremental_cloud->push_back(point);
      
      // Add to accumulated map
      accumulated_map_->push_back(point);
      octree_->addPointToCloud(point, accumulated_map_);
      
      total_incremental_points_++;
    }
  }

  // // Check if we need to downsample the accumulated map
  // if (accumulated_map_->size() > static_cast<size_t>(max_accumulated_points_)) {
  //   RCLCPP_INFO(this->get_logger(), 
  //               "Accumulated map exceeds max size (%zu > %d), downsampling...",
  //               accumulated_map_->size(), max_accumulated_points_);
    
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
  //   pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  //   voxel_filter.setInputCloud(accumulated_map_);
  //   voxel_filter.setLeafSize(voxel_leaf_size_ * 2, voxel_leaf_size_ * 2, voxel_leaf_size_ * 2);
  //   voxel_filter.filter(*downsampled);
    
  //   accumulated_map_ = downsampled;
  //   octree_->deleteTree();
  //   octree_->setInputCloud(accumulated_map_);
  //   octree_->addPointsFromInputCloud();
    
  //   RCLCPP_INFO(this->get_logger(), "Map downsampled to %zu points", accumulated_map_->size());
  // }

  // Publish incremental cloud
  float update_percentage = 100.0 * static_cast<float>(incremental_cloud->size()) / static_cast<float>(filtered_input->size());
  if (update_percentage > 0.5 && !incremental_cloud->empty()) {
    sensor_msgs::msg::PointCloud2 incremental_msg;
    pcl::toROSMsg(*incremental_cloud, incremental_msg);
    incremental_msg.header = header;
    incremental_pub_->publish(incremental_msg);

    // RCLCPP_INFO(this->get_logger(), 
    //              "Published %zu incremental points (%.1f%% new)",
    //              incremental_cloud->size(),
    //              update_percentage);
  }

  // Publish accumulated map periodically
  if (accumulated_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 accumulated_msg;
    pcl::toROSMsg(*accumulated_map_, accumulated_msg);
    accumulated_msg.header = header;
    accumulated_msg.header.frame_id = "map";
    accumulated_pub_->publish(accumulated_msg);
}

  // Log statistics every 100 clouds
  static int callback_count = 0;
  if (++callback_count % 100 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Statistics: Accumulated %zu points, %zu total incremental (%.1f%% novelty rate)",
                accumulated_map_->size(),
                total_incremental_points_,
                100.0 * total_incremental_points_ / total_points_received_);
  }
}

}  // namespace odin1_ros2_driver

RCLCPP_COMPONENTS_REGISTER_NODE(odin1_ros2_driver::IncrementalMapNode)
