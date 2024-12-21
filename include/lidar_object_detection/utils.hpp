#ifndef UTILS_H_
#define UTILS_H_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detail/object_hypothesis_with_pose__struct.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

// Utility function to convert a vector of PointCloud2 to a Detection3DArray
void convertPointCloudsToDetection3DArray(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>> &clusters,
    vision_msgs::msg::Detection3DArray &detections)
{
    detections.detections.clear();

    for (const auto &cluster : clusters) {
        vision_msgs::msg::Detection3D detection;

        // Add BoundingBox3D (for now, dummy values; compute actual values as needed)
        vision_msgs::msg::BoundingBox3D bbox;

        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();
        float max_z = std::numeric_limits<float>::lowest();

        for (const auto &point : cluster.points) {
            min_x = std::min(min_x, point.x);
            min_y = std::min(min_y, point.y);
            min_z = std::min(min_z, point.z);
            max_x = std::max(max_x, point.x);
            max_y = std::max(max_y, point.y);
            max_z = std::max(max_z, point.z);
        }

        bbox.center.position.x = (min_x + max_x) / 2.0;
        bbox.center.position.y = (min_y + max_y) / 2.0;
        bbox.center.position.z = (min_z + max_z) / 2.0;
        bbox.size.x = max_x - min_x;
        bbox.size.y = max_y - min_y;
        bbox.size.z = max_z - min_z;

        detection.bbox = bbox;

        // Add classification hypothesis (optional, set dummy values for now)
        // // Add ObjectHypothesisWithPose
        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        hypothesis.hypothesis.class_id = "object";  // Replace with actual class ID if available
        hypothesis.hypothesis.score = 0.8;    // Replace with confidence score

        // Set hypothesis pose (use the bounding box center for now)
        hypothesis.pose.pose.position.x = bbox.center.position.x;
        hypothesis.pose.pose.position.y = bbox.center.position.y;
        hypothesis.pose.pose.position.z = bbox.center.position.z;
        hypothesis.pose.pose.orientation.w = 1.0;  // Identity quaternion
        hypothesis.pose.pose.orientation.x = 0.0;
        hypothesis.pose.pose.orientation.y = 0.0;
        hypothesis.pose.pose.orientation.z = 0.0;

                detection.results.push_back(hypothesis);

        // Add to detections array
        detections.detections.push_back(std::move(detection));
    }
}

#endif  // UTILS_H_
