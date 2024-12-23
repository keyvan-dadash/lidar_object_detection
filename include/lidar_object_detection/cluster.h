/**
 *@file cluster.h
 *Cluster for 3d points.
 */

#ifndef CLUSTER
#define CLUSTER

#include "HTRBasicDataStructures.h"
#include "OctreeGenerator.h"

namespace lidar_object_detection {
namespace dbscan {
class cluster {
public:
  std::vector<pcl::mod_pointXYZ> clusterPoints;
  std::vector<htr::Point3D> clusterPoints3D;

  pcl::mod_pointXYZ centroid;
  htr::Point3D centroid3D;
  bool visited;

  cluster();
  void calculateCentroid();
  void toPoint3D();

private:
};

static std::vector<pcl::PointCloud<pcl::PointXYZ>> convertClustersToPointClouds(
    const std::vector<cluster>& clusters) {
    // Vector to store the resulting PointClouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>> pointClouds;

    for (const auto& cl : clusters) {
        // Create a new PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;

        // Fill the PointCloud with points from clusterPoints
        for (const auto& point : cl.clusterPoints) {
            pcl::PointXYZ pclPoint;
            pclPoint.x = point.x;
            pclPoint.y = point.y;
            pclPoint.z = point.z;
            cloud.points.push_back(pclPoint);
        }

        // Set the width and height of the PointCloud
        cloud.width = cloud.points.size();  // Number of points in the cloud
        cloud.height = 1;                   // Unorganized cloud
        cloud.is_dense = true;              // Assume dense cloud (no NaN points)

        // Add the PointCloud to the vector
        pointClouds.push_back(std::move(cloud));
    }

    return pointClouds;
}


} // namespace dbscan
} // namespace lidar_object_detection

#endif // CLUSTER
