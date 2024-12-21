#include "rclcpp/rclcpp.hpp"
#include "euclidean_cluster_node.hpp"

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create an executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Instantiate your node
  auto node = std::make_shared<lidar_object_detection::euclidean_cluster::EuclideanClusterNode>(rclcpp::NodeOptions());

  // Add the node to the executor
  executor->add_node(node);

  // Spin the executor
  executor->spin();

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}

