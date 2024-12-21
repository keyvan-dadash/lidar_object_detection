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

#pragma once

#include "lidar_object_detection/euclidean_cluster_interface.hpp"

#include <pcl/point_types.h>

#include <vector>

namespace lidar_object_detection::euclidean_cluster
{
class EuclideanCluster : public EuclideanClusterInterface
{
public:
  EuclideanCluster();
  EuclideanCluster(bool use_height, int min_cluster_size, int max_cluster_size, double distance_threshhold_seg);
  EuclideanCluster(bool use_height, int min_cluster_size, int max_cluster_size, double distance_threshhold_seg, float tolerance);

  bool cluster(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters) override;

  bool cluster(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud) override;
  void setTolerance(float tolerance) { tolerance_ = tolerance; }
  void preprocess(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud);

private:
  double distance_threshhold_seg;
  float tolerance_;
};

}  // namespace autoware::euclidean_cluster

