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

#include "lidar_object_detection/euclidean_cluster.hpp"

#include <pcl/common/eigen.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <vector>

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

namespace lidar_object_detection::euclidean_cluster {

EuclideanCluster::EuclideanCluster() {}

EuclideanCluster::EuclideanCluster(bool use_height, int min_cluster_size,
                                   int max_cluster_size, double distance_threshhold_seg)
    : EuclideanClusterInterface(use_height, min_cluster_size,
                                max_cluster_size), distance_threshhold_seg(distance_threshhold_seg) {}

EuclideanCluster::EuclideanCluster(bool use_height, int min_cluster_size,
                                   int max_cluster_size, double distance_threshhold_seg, float tolerance)
    : EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size),
      tolerance_(tolerance),
    distance_threshhold_seg(distance_threshhold_seg){}
// TODO(badai-nguyen): implement input field copy for euclidean_cluster.cpp
bool EuclideanCluster::cluster(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg) {
  (void)pointcloud_msg;
  return false;
}

void EuclideanCluster::removeLowestZPoints(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud) {
  // Find the minimum Z value
  float min_z = std::numeric_limits<float>::max();
  for (const auto &point : input_cloud->points) {
    if (point.z < min_z) {
      min_z = point.z;
    }
  }

  // Filter out points with the minimum Z value
  for (const auto &point : input_cloud->points) {
    if (point.z > min_z) {
      output_cloud->points.push_back(point);
    }
  }
}

void EuclideanCluster::preprocess(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud) {

  // Step 1: Remove the lowest Z points
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  removeLowestZPoints(cloud, filtered_cloud);

  // Step 2: Perform plane segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setInputCloud(filtered_cloud);
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(distance_threshhold_seg);
  segmentation.setOptimizeCoefficients(true);
  segmentation.setMaxIterations(30000);

  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  segmentation.segment(*planeIndices, *coefficients);

  // Extract points not belonging to the plane
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(filtered_cloud);
  extract.setIndices(planeIndices);
  extract.setNegative(true);
  extract.filter(*outputPointCloud);
}

bool EuclideanCluster::cluster(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pointcloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>> &clusters) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessed_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  preprocess(pointcloud, preprocessed_point_cloud);

  // convert 2d pointcloud
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (!use_height_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &point : preprocessed_point_cloud->points) {
      pcl::PointXYZ point2d;
      point2d.x = point.x;
      point2d.y = point.y;
      point2d.z = 0.0;
      pointcloud_2d_ptr->push_back(point2d);
    }
    pointcloud_ptr = pointcloud_2d_ptr;
  } else {
    pointcloud_ptr = pointcloud;
  }

  // create tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_ptr);

  // clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  // pcl::ConditionalEuclideanClustering<pcl::PointXYZ> pcl_euclidean_cluster;
  // pcl_euclidean_cluster.setConditionFunction (&customRegionGrowing);
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(min_cluster_size_);
  pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);
  // pcl_euclidean_cluster.segment(IndicesClusters &clusters)
  //
  
  // build output
  {
    for (const auto &cluster : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto &point_idx : cluster.indices) {
        cloud_cluster->points.push_back(preprocessed_point_cloud->points[point_idx]);
      }
      clusters.push_back(*cloud_cluster);
      clusters.back().width = cloud_cluster->points.size();
      clusters.back().height = 1;
      clusters.back().is_dense = false;
    }
  }

  return true;
}

} // namespace lidar_object_detection::euclidean_cluster
  //

