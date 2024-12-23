#ifndef DBSCAN
#define DBSCAN

#include "HTRBasicDataStructures.h"
#include "OctreeGenerator.h"
#include "cluster.h"

namespace lidar_object_detection {
namespace dbscan {
class dbscan {
public:
  dbscan();
  ~dbscan();

  void remove_height(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);
  void preprocess(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud);

  void removeLowestZPoints(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud); 

  void init(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
            const int octreeResolution_, const float eps_, const int minPtsAux_,
            const int minPts_);

  inline std::vector<cluster> &getClusters() { return clusters; }
  inline htr::OctreeGenerator::CloudXYZ::Ptr getCloudPoints() {
    return octreeGenIn->getCloud();
  }
  inline pcl::mod_pointXYZ getCentroid() { return centroid; }

  template <typename T>
  void generateClusters(std::vector<std::vector<T>> *clusters);

  void generateClusters_one_step();
  void generateClusters();
  void generateClusters_fast();

private:
  float eps;
  int minPtsAux, minPts, octreeResolution;

  std::vector<cluster> clustersAux, clusters;
  htr::OctreeGenerator *octreeGenIn, *octreeGen;
  std::vector<pcl::mod_pointXYZ> clustersCentroids;
  pcl::mod_pointXYZ centroid;

  void calculateCentroid(std::vector<pcl::mod_pointXYZ> group);
  void octreeRegionQuery(htr::OctreeGenerator *octreeGen,
                         pcl::mod_pointXYZ &searchPoint, double eps,
                         std::vector<int> *retKeys);
  void DBSCAN_Octree_merge(htr::OctreeGenerator *octreeGen, float eps,
                           int minPts);
  void DBSCAN_Octree(htr::OctreeGenerator *octreeGen, float eps, int minPts);
  void DBSCAN_Octree_fast(htr::OctreeGenerator *octreeGen, float eps,
                          int minPts);
  void DBSCAN_Octree_fast_one_step(htr::OctreeGenerator *octreeGen, float eps,
                                   int minPts);
  void DBSCAN_Octree_fast2(htr::OctreeGenerator *octreeGen, int minPts);
};
} // namespace dbscan
} // namespace lidar_object_detection

#endif
