#include "lidar_object_detection/dbScan.h"

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

#

namespace lidar_object_detection {
namespace dbscan {
    
dbscan::dbscan() {
  octreeGenIn = new htr::OctreeGenerator();
  octreeGen = new htr::OctreeGenerator();
}

dbscan::~dbscan() {
  delete octreeGenIn;
  delete octreeGen;
}

/// Initializes the point cloud from a file, and the octree.
///@param[in] filename          Location of the file that has the data.
///@param[in] octreeResolution_ Resolution with which the octree is initialized.
///@param[in] eps_              The search radius for the octree.
///@param[in] minPtsAux_        Minimum points for the initial clusters.
///@param[in] minPts_           Minimum points for the final clusters.

void dbscan::generateClusters_one_step() {
  clusters.clear();
  clustersAux.clear();
  clustersCentroids.clear();

  // A first set of clusters is generated. This first set has a large number of
  // small clusters.
  DBSCAN_Octree_fast_one_step(octreeGenIn, eps, minPts);

  //// The clusters centroids are calculated and used to generate a second
  ///octree.
  // for (lidar_object_detection::dbscan::cluster cluster : clustersAux)
  //	clustersCentroids.push_back(cluster.centroid);

  // octreeGen->initCloudFromVector<pcl::mod_pointXYZ>(clustersCentroids);
  // octreeGen->initOctree(octreeResolution);

  //// Using the second octree and the centroids of the clusters, a new set of
  ///clusters is
  /// generated.
  //// These are the final clusters.
  // DBSCAN_Octree_merge(octreeGen, 2 * eps, minPts);

  for (int i = 0; i < clusters.size(); i++)
    clusters[i].toPoint3D();
}

void CloudToVector(const std::vector<pcl::mod_pointXYZ> &inPointVector,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outPointCloud) {
  for (const pcl::mod_pointXYZ &point : inPointVector) {
    outPointCloud->points.push_back(point);
  }
}

void ConvertPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inPointCloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &outPointCloud) {
  for (const auto &pointRGB : inPointCloud->points) {
    pcl::PointXYZ point;
    point.x = pointRGB.x;
    point.y = pointRGB.y;
    point.z = pointRGB.z;
    outPointCloud->points.push_back(point);
  }

  outPointCloud->width = outPointCloud->points.size();
  outPointCloud->height = 1;  // Since it's a 1D point cloud
  outPointCloud->is_dense = inPointCloud->is_dense;
}

void dbscan::remove_height(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &point : input_cloud->points) {
      pcl::PointXYZ point2d;
      point2d.x = point.x;
      point2d.y = point.y;
      point2d.z = 0.0;
      pointcloud_2d_ptr->push_back(point2d);
    }
    input_cloud = pointcloud_2d_ptr;
}

void dbscan::removeLowestZPoints(
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

void dbscan::preprocess(
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
  segmentation.setDistanceThreshold(0.07);
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

/// Initializes the point cloud from a vector, and the octree.
///@param[in] points            Points from which the cloud will be initialized.
///@param[in] octreeResolution_ Resolution with which the octree is initialized.
///@param[in] eps_              The search radius for the octree.
///@param[in] minPtsAux_        Minimum points for the initial clusters.
///@param[in] minPts_           Minimum points for the final clusters.
void dbscan::init(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                  const int octreeResolution_, const float eps_,
                  const int minPtsAux_, const int minPts_) {
  eps = eps_;
  minPts = minPts_;
  minPtsAux = minPtsAux_;
  octreeResolution = octreeResolution_;

  //remove_height(input_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessed_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  preprocess(input_cloud, preprocessed_point_cloud);

  std::cout << preprocessed_point_cloud->size() << std::endl;

  octreeGenIn->initCloudFromVector(preprocessed_point_cloud);
  octreeGenIn->initOctree(octreeResolution);

  // centroid = octreeGenIn->getCloudCentroid();
}

template <typename T>
void dbscan::generateClusters(std::vector<std::vector<T>> *clustersOut) {
  // A first set of clusters is generated. This first set has a large number of
  // small clusters.
  DBSCAN_Octree(octreeGenIn, eps, minPtsAux);

  // The clusters centroids are calculated and used to generate a second octree.
  for (lidar_object_detection::dbscan::cluster cluster : clustersAux)
    clustersCentroids.push_back(cluster.centroid);

  // octreeGen->initCloudFromVector<pcl::mod_pointXYZ>(clustersCentroids);
  octreeGen->initOctree(octreeResolution);

  // Using the second octree and the centroids of the clusters, a new set of
  // clusters is generated. These are the final clusters.
  DBSCAN_Octree_merge(octreeGen, 2 * eps, minPts);

  //       for(int i = 0; i<clusters.size(); i++)
  //           clusters[i].toPoint3D();
}

/// Generates the clusters from the loaded data.
void dbscan::generateClusters() {
  clusters.clear();
  clustersAux.clear();
  clustersCentroids.clear();

  // A first set of clusters is generated. This first set has a large number of
  // small clusters.
  DBSCAN_Octree(octreeGenIn, eps, minPtsAux);

  // The clusters centroids are calculated and used to generate a second octree.
  for (lidar_object_detection::dbscan::cluster cluster : clustersAux)
    clustersCentroids.push_back(cluster.centroid);

  // octreeGen->initCloudFromVector<pcl::mod_pointXYZ>(clustersCentroids);
  octreeGen->initOctree(octreeResolution);

  // Using the second octree and the centroids of the clusters, a new set of
  // clusters is generated. These are the final clusters.
  DBSCAN_Octree_merge(octreeGen, 2 * eps, minPts);

  for (int i = 0; i < clusters.size(); i++)
    clusters[i].toPoint3D();
}

/// Generates the clusters from the loaded data.
void dbscan::generateClusters_fast() {
  clusters.clear();
  clustersAux.clear();
  clustersCentroids.clear();

  // A first set of clusters is generated. This first set has a large number of
  // small clusters. DBSCAN_Octree_fast2(octreeGenIn, minPtsAux);
  DBSCAN_Octree_fast(octreeGenIn, eps, minPtsAux);

  //        printf("\n Aux clusters size:%d\n\n", clustersAux.size());
  // The clusters centroids are calculated and used to generate a second octree.
  for (lidar_object_detection::dbscan::cluster cluster : clustersAux)
    clustersCentroids.push_back(cluster.centroid);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  CloudToVector(clustersCentroids, new_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud_normal(
      new pcl::PointCloud<pcl::PointXYZ>());

  ConvertPointCloud(new_cloud, new_cloud_normal);

  octreeGen->initCloudFromVector(new_cloud_normal);

  octreeGen->initOctree(octreeResolution);

  // Using the second octree and the centroids of the clusters, a new set of
  // clusters is generated. These are the final clusters.
  // DBSCAN_Octree_merge(octreeGen, 2*eps, minPts);
  DBSCAN_Octree_merge(octreeGen, 2 * eps, minPts);

  //        printf("\n Clusters size:%d\n\n", clusters.size());
  for (int i = 0; i < clusters.size(); i++)
    clusters[i].toPoint3D();
}

/// Calculates the centroid form a vector of points.
///@param[in] group             Vector that contains the point that will be
///processed.
void dbscan::calculateCentroid(std::vector<pcl::mod_pointXYZ> group) {
  for (pcl::mod_pointXYZ point : group) {
    centroid.x += point.x;
    centroid.y += point.y;
    centroid.z += point.z;
  }
  centroid.x /= group.size();
  centroid.y /= group.size();
  centroid.z /= group.size();
}

/// Does a radius search for the K nearest neighbors of a point.
///@param[in] octreeGen         The octree to be searched.
///@param[in] searchPoint       The point around which the search will be
///conducted.
///@param[in] eps_              The search radius for the octree.
///@param[in] retKeys_          Vector that stores the indices of the nearest
///points.
void dbscan::octreeRegionQuery(htr::OctreeGenerator *octreeGen,
                               pcl::mod_pointXYZ &searchPoint, double eps,
                               std::vector<int> *retKeys) {
  retKeys->clear();
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  octreeGen->getOctree()->radiusSearch(searchPoint, eps, *retKeys,
                                       pointRadiusSquaredDistance);
}

/// Merges a set of clusters.
///@param[in] octreeGen         The octree to be searched.
///@param[in] eps_              The search radius for the octree.
///@param[in] clustersIn        The clusters that will be merged.
///@param[in] clustersOut       Vector that stores all merged clusters.
void dbscan::DBSCAN_Octree_merge(htr::OctreeGenerator *octreeGen, float eps,
                                 int minPts) {
  clusters.clear();
  cluster pointQueue;
  std::vector<pcl::mod_pointXYZ> clusterPoints;

  // The amount of aux clusters
  int noKeys = clustersAux.size();
  std::vector<bool> visited(noKeys, false);

  std::vector<int> noise;
  std::vector<int> neighborPts(noKeys, -1);

  for (int i = 0; i < noKeys; i++) {
    if (!visited[i]) {
      clusterPoints.push_back(clustersAux.at(i).centroid);

      pointQueue.clusterPoints.insert(pointQueue.clusterPoints.end(),
                                      clustersAux.at(i).clusterPoints.begin(),
                                      clustersAux.at(i).clusterPoints.end());

      visited[i] = true;

      for (int j = 0; j < clusterPoints.size(); j++) {
        octreeRegionQuery(octreeGen, clusterPoints.at(j), eps, &neighborPts);

        for (int k = 0; k < neighborPts.size(); k++) {
          if (!visited[neighborPts[k]]) {
            visited[neighborPts[k]] = true;

            clusterPoints.push_back(clustersAux.at(neighborPts[k]).centroid);

            pointQueue.clusterPoints.insert(
                pointQueue.clusterPoints.end(),
                clustersAux.at(neighborPts[k]).clusterPoints.begin(),
                clustersAux.at(neighborPts[k]).clusterPoints.end());
          }
        }
      }

      if (pointQueue.clusterPoints.size() >= minPts) {
        pointQueue.calculateCentroid();
        clusters.push_back(pointQueue);
        pointQueue.clusterPoints.clear();
      }
    }
  }

  //       clustersAux.clear();
}

/// Generates a set of clusters.
///@param[in] octreeGen         The octree to be searched.
///@param[in] eps_              The search radius for the octree.
///@param[in] clusters          Vector that stores all the generated clusters.
void dbscan::DBSCAN_Octree(htr::OctreeGenerator *octreeGen, float eps,
                           int minPts) {
  clustersAux.clear();
  cluster pointQueue;
  pcl::mod_pointXYZ auxCentroid;

  int noKeys = octreeGen->getCloud()->points.size();
  std::vector<bool> visited(noKeys, false);
  std::vector<int> classification(noKeys, 0);

  std::vector<int> noise;
  std::vector<int> neighborPts;

  for (int i = 0; i < noKeys; i++) {
    if (!visited[i]) {
      pointQueue.clusterPoints.push_back(octreeGen->getCloud()->points.at(i));
      visited[i] = true;

      octreeRegionQuery(octreeGen, pointQueue.clusterPoints.at(0), eps,
                        &neighborPts);

      if (neighborPts.size() < minPtsAux)
        noise.push_back(i);
      else {
        for (int k = 0; k < neighborPts.size(); k++) {
          if (!visited[neighborPts[k]]) {
            visited[neighborPts[k]] = true;
            pcl::mod_pointXYZ auxPoint =
                octreeGen->getCloud()->points.at(neighborPts[k]);
            pointQueue.clusterPoints.push_back(auxPoint);
          }
        }

        if (pointQueue.clusterPoints.size() >= minPtsAux) {
          pointQueue.calculateCentroid();
          clustersAux.push_back(pointQueue);
          pointQueue.clusterPoints.clear();
        }
      }
    }
  }
}

/// Generates a set of clusters.
///@param[in] octreeGen         The octree to be searched.
///@param[in] eps_              The search radius for the octree.
///@param[in] clusters          Vector that stores all the generated clusters.
void dbscan::DBSCAN_Octree_fast(htr::OctreeGenerator *octreeGen, float eps,
                                int minPts) // O(nlogn)
{
  clustersAux.clear();

  cluster pointQueue;
  pcl::mod_pointXYZ auxCentroid;

  // The number of points
  int noKeys = octreeGen->getCloud()->points.size();
  std::vector<bool> visited(noKeys, false);
  std::vector<int> classification(noKeys, 0);

  std::vector<int> noise;
  std::vector<int> neighborPts;

  for (int i = 0; i < noKeys; i++) // O(n)
  {
    if (!visited[i]) // O(log n)
    {
      pointQueue.clusterPoints.push_back(octreeGen->getCloud()->points.at(i));
      visited[i] = true;

      octreeGen->getOctree()->voxelSearch(pointQueue.clusterPoints.at(0),
                                          neighborPts);

      if (neighborPts.size() < minPtsAux)
        noise.push_back(i);
      else {
        for (int k = 0; k < neighborPts.size(); k++) {
          if (!visited[neighborPts[k]]) {
            visited[neighborPts[k]] = true;
            pcl::mod_pointXYZ auxPoint =
                octreeGen->getCloud()->points.at(neighborPts[k]);
            pointQueue.clusterPoints.push_back(auxPoint);
          }
        }

        if (pointQueue.clusterPoints.size() >= minPtsAux) {
          pointQueue.calculateCentroid();
          clustersAux.push_back(pointQueue);
          pointQueue.clusterPoints.clear();
        }
      }
    }
  }
}

/// Generates a set of clusters.
///@param[in] octreeGen         The octree to be searched.
///@param[in] eps_              The search radius for the octree.
///@param[in] clusters          Vector that stores all the generated clusters.
void dbscan::DBSCAN_Octree_fast_one_step(htr::OctreeGenerator *octreeGen,
                                         float eps, int minPts) {
  int noKeys = octreeGen->getCloud()->points.size();
  std::vector<bool> clustered(noKeys, false);
  std::vector<bool> visited(noKeys, false);

  std::vector<int> noise;

  std::vector<int> neighborPts;
  std::vector<int> neighborPts_;

  int c = 0;

  // for each unvisted point P in dataset keypoints
  for (int i = 0; i < noKeys; i++) {
    if (!visited[i]) {
      // Mark P as visited
      visited[i] = true;
      octreeRegionQuery(octreeGen, octreeGen->getCloud()->points.at(i), eps,
                        &neighborPts);

      if (neighborPts.size() < minPts)
        // Mark P as Noise
        noise.push_back(i);
      else {
        clusters.push_back(cluster());

        // expand cluster, add P to cluster c
        clustered[i] = true;
        clusters.at(c).clusterPoints.push_back(
            octreeGen->getCloud()->points.at(i));

        // for each point P' in neighborPts, Expand cluster
        for (int j = 0; j < neighborPts.size(); j++) {
          //                        if P' is not visited
          if (!visited[neighborPts[j]]) {
            // Mark P' as visited
            visited[neighborPts[j]] = true;
            octreeRegionQuery(octreeGen,
                              octreeGen->getCloud()->points.at(neighborPts[j]),
                              eps, &neighborPts_);
            //
            if (neighborPts_.size() >= minPts)
              neighborPts.insert(neighborPts.end(), neighborPts_.begin(),
                                 neighborPts_.end());
          }
          // if P' is not yet a member of any cluster, add P' to cluster c
          if (!clustered[neighborPts[j]]) {
            clustered[neighborPts[j]] = true;
            clusters.at(c).clusterPoints.push_back(
                octreeGen->getCloud()->points.at(neighborPts[j]));
          }
        }
        c++;
      }
    }
  }
}

/// Generates a set of clusters.
///@param[in] octreeGen         The octree to be searched.
///@param[in] eps_              The search radius for the octree.
///@param[in] clusters          Vector that stores all the generated clusters.
void dbscan::DBSCAN_Octree_fast2(htr::OctreeGenerator *octreeGen, int minPts) {
  // Clear aux clusters
  clustersAux.clear();

  if (!octreeGen->getCloud()->points.empty()) {
    // Create array of tree iterators
    std::vector<htr::OctreeGenerator::LeafNodeIterator> leafIterators;

    std::vector<int> tempVec;
    cluster tempPointCluster;
    std::vector<std::vector<int>> neighborPts;

    htr::OctreeGenerator::LeafNodeIterator it(octreeGen->getOctree().get());

    while (*(it)) {
      neighborPts.push_back(tempVec);
      clustersAux.push_back(tempPointCluster);

      it.getLeafContainer().getPointIndices(neighborPts.back());
      for (int k = 0; k < neighborPts.back().size(); ++k) {
        clustersAux.back().clusterPoints.push_back(
            octreeGen->getCloud()->points.at(neighborPts.back().at(k)));
      }
      clustersAux.back().calculateCentroid();

      ++it;
    }
  }
}
} // namespace dbscan
} // namespace lidar_object_detection
