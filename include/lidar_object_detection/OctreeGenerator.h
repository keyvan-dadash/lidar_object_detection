/**
 *@class OctreeGenerator
 *Creates an octree from the point cloud data provided.
 *
 */

#ifndef OCTREE_GENERATOR_H
#define OCTREE_GENERATOR_H
// #include <pcl/octree/octree_impl.h>

#include <pcl/common/centroid.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <vector>

#include "lidar_object_detection/HTRBasicDataStructures.h"

namespace htr {

class OctreeGenerator {
 public:
  typedef pcl::PointCloud<pcl::mod_pointXYZ> CloudXYZ;
  typedef pcl::octree::OctreePointCloudSearch<pcl::mod_pointXYZ> OctreeXYZSearch;
  typedef pcl::octree::OctreePointCloudSearch<pcl::mod_pointXYZ>::LeafNode LeafNode;
  typedef pcl::octree::OctreePointCloudSearch<pcl::mod_pointXYZ>::LeafNodeIterator LeafNodeIterator;
  typedef htr::Point3D Point3D;

  struct Voxel {
    Point3D position;
    float size;
  };

  OctreeGenerator();
  ~OctreeGenerator();

  inline CloudXYZ::Ptr getCloud() { return cloud; }
  inline pcl::mod_pointXYZ getCloudCentroid() { return cloudCentroid; }
  inline std::vector<Voxel> &getVoxels() { return octreeVoxels; }
  inline std::vector<Point3D> &getCentroids() { return octreeCentroids; }
  inline OctreeXYZSearch::Ptr getOctree() { return octree_p; }

  void initRandomCloud(const float width, const float height, const float depth, const int numOfPoints);

  void initCloudFromVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);

  void initOctree(const int resolution);

  void extractPointsAtLevel(const int depth);
  void stepExtractionLevel(const int step);

 private:
  unsigned int currentExtractionLevel;
  pcl::PointCloud<pcl::mod_pointXYZ>::Ptr cloud;
  OctreeXYZSearch::Ptr octree_p;

  pcl::mod_pointXYZ cloudCentroid;

  std::vector<Voxel> octreeVoxels;
  std::vector<Point3D> octreeCentroids;

  void calculateCloudCentroid();
};
}  // namespace htr

#endif
