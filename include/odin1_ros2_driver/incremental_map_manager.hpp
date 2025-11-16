#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/logger.hpp>
#include <memory>
#include <mutex>
#include <cmath>

namespace odin1_ros2_driver
{

/**
 * @brief Header-only incremental map manager using octree for efficient novelty detection
 * 
 * This class manages a global point cloud map and filters incoming point clouds
 * to only publish novel (new) points that haven't been seen before.
 */
class IncrementalMapManager
{
public:
  /**
   * @brief Constructor
   * @param logger ROS logger for status messages
   * @param octree_resolution Octree voxel size for spatial indexing (meters)
   * @param voxel_leaf_size Voxel filter size for input downsampling (meters, 0 = no filtering)
   * @param novelty_threshold Radius to check for existing points (meters)
   * @param max_accumulated_points Maximum points before warning (not enforced)
   */
  explicit IncrementalMapManager(
    const rclcpp::Logger & logger,
    double octree_resolution = 0.05,
    double voxel_leaf_size = 0.0,
    double novelty_threshold = 0.05,
    size_t max_accumulated_points = 5000000)
  : logger_(logger)
  , octree_resolution_(octree_resolution)
  , voxel_leaf_size_(voxel_leaf_size)
  , novelty_threshold_(novelty_threshold)
  , max_accumulated_points_(max_accumulated_points)
  , total_points_received_(0)
  , total_incremental_points_(0)
  {
    accumulated_map_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    octree_ = std::make_unique<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>>(octree_resolution_);
    octree_->setInputCloud(accumulated_map_);
    
    RCLCPP_INFO(logger_, "IncrementalMapManager initialized:");
    RCLCPP_INFO(logger_, "  - Octree resolution: %.3f m", octree_resolution_);
    RCLCPP_INFO(logger_, "  - Voxel filter size: %.3f m", voxel_leaf_size_);
    RCLCPP_INFO(logger_, "  - Novelty threshold: %.3f m", novelty_threshold_);
    RCLCPP_INFO(logger_, "  - Max accumulated points: %zu", max_accumulated_points_);
  }

  /**
   * @brief Process incoming point cloud and extract novel points
   * @param input_cloud Input point cloud to process
   * @param output_cloud Output containing only novel points
   * @return Number of novel points found
   */
  size_t processCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud)
  {
    if (!input_cloud || input_cloud->empty()) {
      return 0;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);

    total_points_received_ += input_cloud->size();
    output_cloud->clear();
    output_cloud->header = input_cloud->header;

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
        output_cloud->push_back(point);
        
        // Add to accumulated map
        accumulated_map_->push_back(point);
        octree_->addPointToCloud(point, accumulated_map_);
        
        total_incremental_points_++;
      }
    }

    // Warn if map is getting large (simple check without throttle to avoid clock dependency)
    static size_t last_warning_size = 0;
    if (accumulated_map_->size() > max_accumulated_points_ && 
        accumulated_map_->size() - last_warning_size > 100000) {
      RCLCPP_WARN(logger_,
                  "Accumulated map exceeds max size: %zu > %zu",
                  accumulated_map_->size(), max_accumulated_points_);
      last_warning_size = accumulated_map_->size();
    }

    return output_cloud->size();
  }

  /**
   * @brief Clear the accumulated map
   */
  void clear()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    accumulated_map_->clear();
    octree_->deleteTree();
    octree_->setInputCloud(accumulated_map_);
    total_points_received_ = 0;
    total_incremental_points_ = 0;
    
    RCLCPP_INFO(logger_, "Incremental map cleared");
  }

  /**
   * @brief Get the accumulated map
   * @return Const pointer to accumulated point cloud
   */
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr getAccumulatedMap() const
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return accumulated_map_;
  }

  /**
   * @brief Get statistics
   */
  void getStatistics(size_t & total_received, size_t & total_incremental, 
                    size_t & map_size, float & novelty_rate) const
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    total_received = total_points_received_;
    total_incremental = total_incremental_points_;
    map_size = accumulated_map_->size();
    novelty_rate = (total_points_received_ > 0) ? 
                   (100.0f * total_incremental_points_ / total_points_received_) : 0.0f;
  }

  /**
   * @brief Log current statistics
   */
  void logStatistics() const
  {
    size_t total_received, total_incremental, map_size;
    float novelty_rate;
    getStatistics(total_received, total_incremental, map_size, novelty_rate);
    
    RCLCPP_INFO(logger_,
                "Map Statistics: %zu points accumulated, %zu total incremental (%.1f%% novelty rate)",
                map_size, total_incremental, novelty_rate);
  }

private:
  const rclcpp::Logger logger_;
  
  // PCL data structures
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_map_;
  std::unique_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>> octree_;
  
  // Parameters
  const double octree_resolution_;
  const double voxel_leaf_size_;
  const double novelty_threshold_;
  const size_t max_accumulated_points_;
  
  // Statistics
  size_t total_points_received_;
  size_t total_incremental_points_;
  
  // Thread safety
  mutable std::mutex map_mutex_;
};

}  // namespace odin1_ros2_driver
