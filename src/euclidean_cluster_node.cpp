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

  const std::string input_topic = this->declare_parameter("input_topic", "velodyne_points");
  RCLCPP_INFO(this->get_logger(), "Input Topic: %s", input_topic.c_str());
  
  const std::string output_topic = this->declare_parameter("output_topic", "detected_objects");
  RCLCPP_INFO(this->get_logger(), "Output Topic: %s", output_topic.c_str());
 
  const std::string output_topic_clustered = this->declare_parameter("output_topic_clustered", "clustered_points");
  RCLCPP_INFO(this->get_logger(), "Output Topic Clustered: %s", output_topic.c_str());

  // Height
  const double max_height = this->declare_parameter("max_height", 100.0);
  RCLCPP_INFO(this->get_logger(), "max_height: %f", max_height);

  const double min_height = this->declare_parameter("min_height", 1.0);
  RCLCPP_INFO(this->get_logger(), "min_height: %f", min_height);

  filter_ops.max_height = max_height;
  filter_ops.min_height = min_height;

  // width
  const double max_width = this->declare_parameter("max_width", 100.0);
  RCLCPP_INFO(this->get_logger(), "max_width: %f", max_width);

  const double min_width = this->declare_parameter("min_width", 1.0);
  RCLCPP_INFO(this->get_logger(), "min_width: %f", min_width);

  filter_ops.max_width = max_width;
  filter_ops.min_width = min_width;

  // depth
  const double max_depth = this->declare_parameter("max_depth", 100.0);
  RCLCPP_INFO(this->get_logger(), "max_depth: %f", max_depth);

  const double min_depth = this->declare_parameter("min_depth", 1.0);
  RCLCPP_INFO(this->get_logger(), "min_depth: %f", min_depth);

  filter_ops.max_depth = max_depth;
  filter_ops.min_depth = min_depth;

  // zx
  const double max_zx = this->declare_parameter("max_zx", 1000.0);
  RCLCPP_INFO(this->get_logger(), "max_zx: %f", max_zx);

  const double min_zx = this->declare_parameter("min_zx", 10.0);
  RCLCPP_INFO(this->get_logger(), "min_zx: %f", min_zx); 

  filter_ops.max_zx = max_zx;
  filter_ops.min_zx = min_zx;

  // zy
  const double max_zy = this->declare_parameter("max_zy", 1000.0);
  RCLCPP_INFO(this->get_logger(), "max_zy: %f", max_zy);

  const double min_zy = this->declare_parameter("min_zy", 10.0);
  RCLCPP_INFO(this->get_logger(), "min_zy: %f", min_zy); 

  filter_ops.max_zy = max_zy;
  filter_ops.min_zy = min_zy;

  cluster_ = std::make_shared<EuclideanCluster>(
      use_height, min_cluster_size, max_cluster_size, distance_threshhold_seg,
      tolerance);

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&EuclideanClusterNode::onPointCloud, this, _1));

  clustered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_clustered, rclcpp::QoS{1});

  detection_obj_visual_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(output_topic, 10);
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

  std::cout << "Number of detected objects: " << clusters.size() << std::endl;

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

  vision_msgs::msg::Detection3DArray post_processed_objs;
  filter_objects(detections, post_processed_objs, filter_ops);
  // Publish the detections
  std::cout << "Number of detected objects after post process: " << post_processed_objs.detections.size() << std::endl;
  detection_obj_visual_->publish(post_processed_objs);
}

} // namespace lidar_object_detection::euclidean_cluster
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
    lidar_object_detection::euclidean_cluster::EuclideanClusterNode)
