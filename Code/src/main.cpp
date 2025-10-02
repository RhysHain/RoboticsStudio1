#include "rclcpp/rclcpp.hpp"
#include "drone_node.h"
#include "husky_node.h"

int main(int argc, char **argv)
{
  /**
   * Initialize ROS 2
   */
  rclcpp::init(argc, argv);

  // Create two nodes
  auto node1 = std::make_shared<DroneNode>();
  auto node2 = std::make_shared<HuskyNode>(); 

  // Create an executor
  rclcpp::executors::MultiThreadedExecutor executor;

  // Add nodes to the executor
  executor.add_node(node1);
  executor.add_node(node2);

  // Spin the executor
  executor.spin();

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}