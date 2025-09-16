#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "odin1_ros2_driver/odin1_driver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Create the node with default options
  rclcpp::NodeOptions options;
  auto node = std::make_shared<odin1_ros2_driver::Odin1Driver>(options);
  
  // Spin the node
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
