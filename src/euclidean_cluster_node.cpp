// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pcl_conversions/pcl_conversions.h"

#include "euclidean_cluster_node.hpp"
#include "lidar_object_detection/utils.hpp"

#include <cstdlib>
#include <memory>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <vector>
#include <vision_msgs/msg/detail/detection3_d_array__struct.hpp>

namespace lidar_object_detection::euclidean_cluster {
EuclideanClusterNode::EuclideanClusterNode(const rclcpp::NodeOptions &options)
    : Node("euclidean_cluster_node", options) {
  // Declare parameters and log their values
  const bool use_height = this->declare_parameter("use_height", true);
  RCLCPP_INFO(this->get_logger(), "use_height: %s", use_height ? "true" : "false");

  const int min_cluster_size = this->declare_parameter("min_cluster_size", 10);
  RCLCPP_INFO(this->get_logger(), "min_cluster_size: %d", min_cluster_size);

  const int max_cluster_size = this->declare_parameter("max_cluster_size", 5000);
  RCLCPP_INFO(this->get_logger(), "max_cluster_size: %d", max_cluster_size);

  const double distance_threshhold_seg = this->declare_parameter("distance_threshhold_seg", 0.07);
  RCLCPP_INFO(this->get_logger(), "distance_threshhold_seg: %f", distance_threshhold_seg);

  const float tolerance = this->declare_parameter("tolerance", 0.1);
  RCLCPP_INFO(this->get_logger(), "tolerance: %f", tolerance);

  cluster_ = std::make_shared<EuclideanCluster>(
      use_height, min_cluster_size, max_cluster_size, distance_threshhold_seg,
      tolerance);

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&EuclideanClusterNode::onPointCloud, this, _1));

  clustered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "clustered_points", rclcpp::QoS{1});

  detection_obj_visual_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("detected_objects", 10);
}

void EuclideanClusterNode::onPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg) {
  // convert ros to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_msg, *raw_pointcloud_ptr);

  // clustering
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  cluster_->cluster(raw_pointcloud_ptr, clusters);

  for (const auto &cluster : clusters) {
    sensor_msgs::msg::PointCloud2 tmp;
    pcl::toROSMsg(cluster, tmp);
    clustered_pub_->publish(tmp);
  }

  std::cout << clusters.size() << std::endl;

/*   pcl::PCDWriter writer;*/
   /*int j = 0;*/
   /*for (const auto& cloud_cluster : clusters) {*/
    /*std::cout << "PointCloud representing the Cluster: " << cloud_cluster.size () << " data points." << std::endl;*/
     /*std::stringstream ss;*/
     /*ss << std::setw(4) << std::setfill('0') << j;*/
     /*writer.write<pcl::PointXYZ> ("cloud_cluster_" + ss.str () + ".pcd", cloud_cluster, false); **/
     /*j++;*/
   /*}*/

   /*std::exit(0);*/

  vision_msgs::msg::Detection3DArray detections;
  detections.header.stamp = this->now();
  detections.header.frame_id = "map";

  // Use the utility function to populate the detections
  convertPointCloudsToDetection3DArray(clusters, detections);

  // Publish the detections
  detection_obj_visual_->publish(detections);
}

} // namespace lidar_object_detection::euclidean_cluster
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
    lidar_object_detection::euclidean_cluster::EuclideanClusterNode)
