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

#include "lidar_object_detection/cluster.h"
#include "pcl_conversions/pcl_conversions.h"

#include "dbscan_cluster_node.hpp"
#include "lidar_object_detection/utils.hpp"

#include <cstdlib>
#include <memory>
#include <pcl/impl/point_types.hpp>
#include <vector>

namespace lidar_object_detection::dbscan {
DBScanClusterNode::DBScanClusterNode(const rclcpp::NodeOptions &options)
    : Node("dbscan_cluster_node", options) {
  // Declare parameters and log their values
  const int octreeResolution = this->declare_parameter("octree_resolution", 1);
  RCLCPP_INFO(this->get_logger(), "octreeResolution: %d", octreeResolution);
  octreeResolution_ = octreeResolution;

  const float eps = this->declare_parameter("eps", 0.2);
  RCLCPP_INFO(this->get_logger(), "eps: %f", eps);
  eps_ = eps;

  const int min_pts = this->declare_parameter("min_pts", 10);
  RCLCPP_INFO(this->get_logger(), "min_pts: %d", min_pts);
  min_pts_ = min_pts;

  const int min_pts_aux = this->declare_parameter("min_pts_aux", 10);
  RCLCPP_INFO(this->get_logger(), "min_pts_aux: %d", min_pts_aux);
  min_pts_aux_ = min_pts_aux;

  cluster_ = std::make_shared<dbscan>();

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&DBScanClusterNode::onPointCloud, this, _1));

  clustered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "clustered_points", rclcpp::QoS{1});

  detection_obj_visual_ =
      this->create_publisher<vision_msgs::msg::Detection3DArray>(
          "detected_objects", 10);
}

void DBScanClusterNode::onPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg) {
  // convert ros to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_msg, *raw_pointcloud_ptr);

  std::cout << raw_pointcloud_ptr->size() << std::endl;
  
  cluster_->init(raw_pointcloud_ptr, octreeResolution_, eps_, min_pts_aux_, min_pts_);

  cluster_->generateClusters();

  std::cout << cluster_->getClusters().size() << std::endl;

  auto clusters = lidar_object_detection::dbscan::convertClustersToPointClouds(
      cluster_->getClusters());

/*  pcl::PCDWriter writer;*/
  /*int j = 0;*/
  /*for (const auto &cloud_cluster : clusters) {*/
    /*std::cout << "PointCloud representing the Cluster: " << cloud_cluster.size()*/
              /*<< " data points. width: " << cloud_cluster.width << " height: "<< cloud_cluster.height << " total: " << cloud_cluster.points.size() << std::endl;*/
    /*std::stringstream ss;*/
    /*ss << std::setw(4) << std::setfill('0') << j;*/
    /*writer.write<pcl::PointXYZ>("cloud_cluster_" + ss.str() + ".pcd",*/
                                /*cloud_cluster, false);*/
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

} // namespace lidar_object_detection::dbscan
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
    lidar_object_detection::dbscan::DBScanClusterNode)
