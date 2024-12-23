#ifndef DBSCAN_CLUSTER_NODE_H_
#define DBSCAN_CLUSTER_NODE_H_

#include "lidar_object_detection/dbScan.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>


namespace lidar_object_detection::dbscan {
class DBScanClusterNode : public rclcpp::Node {
private:
  void onPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr
      detection_obj_visual_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;

  std::shared_ptr<dbscan> cluster_;

  int octreeResolution_;
  float eps_;
  int min_pts_;
  int min_pts_aux_;

public:
  explicit DBScanClusterNode(const rclcpp::NodeOptions & options);
};
} // namespace lidar_object_detection::dbscan

#endif /* DBSCAN_CLUSTER_NODE_H_ */
