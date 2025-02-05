#ifndef UTILS_H_
#define UTILS_H_

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <vision_msgs/msg/detail/object_hypothesis_with_pose__struct.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

// Utility function to convert a vector of PointCloud2 to a Detection3DArray
static void convertPointCloudsToDetection3DArray(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>> &clusters,
    vision_msgs::msg::Detection3DArray &detections) {
  detections.detections.clear();

  for (const auto &cluster : clusters) {
    vision_msgs::msg::Detection3D detection;

    // Add BoundingBox3D (for now, dummy values; compute actual values as
    // needed)
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
    hypothesis.hypothesis.class_id =
        "object"; // Replace with actual class ID if available
    hypothesis.hypothesis.score = 0.8; // Replace with confidence score

    // Set hypothesis pose (use the bounding box center for now)
    hypothesis.pose.pose.position.x = bbox.center.position.x;
    hypothesis.pose.pose.position.y = bbox.center.position.y;
    hypothesis.pose.pose.position.z = bbox.center.position.z;
    hypothesis.pose.pose.orientation.w = 1.0; // Identity quaternion
    hypothesis.pose.pose.orientation.x = 0.0;
    hypothesis.pose.pose.orientation.y = 0.0;
    hypothesis.pose.pose.orientation.z = 0.0;

    detection.results.push_back(hypothesis);

    // Add to detections array
    detections.detections.push_back(std::move(detection));
  }
}

typedef struct {
    double max_height;
    double min_height;
    double max_width;
    double min_width;
    double max_depth;
    double min_depth;
    double max_zx;
    double min_zx;
    double max_zy;
    double min_zy;
} filter_options_t;

static void
filter_objects(vision_msgs::msg::Detection3DArray &input_detections,
               vision_msgs::msg::Detection3DArray &output_detections,
               filter_options_t filters)
{
  // Copy header or other metadata as needed
  output_detections.header = input_detections.header;

  // Clear any existing data in output
  output_detections.detections.clear();

  for (const auto &detection : input_detections.detections)
  {
    const auto &bbox = detection.bbox;

    // Interpreting size.x = width, size.y = depth, size.z = height
    double width  = bbox.size.x;
    double depth  = bbox.size.y;
    double height = bbox.size.z;

    // Compute the relevant areas.
    // area_zx = height * width   (Z-X plane)
    // area_zy = height * depth   (Z-Y plane)
    double area_zx = height * width;
    double area_zy = height * depth;

    // ---------------------------------------------------
    // 1. Width check with area override
    // ---------------------------------------------------
    // Normal condition: width >= min_width
    // If width < min_width, allow if area_zy in [min_zy, max_zy].
    bool passWidthMin = 
      (width >= filters.min_width) ||
      (
        (width < filters.min_width) &&
        (area_zy >= filters.min_zy && area_zy <= filters.max_zy)
      );
    // Still enforce max_width
    bool passWidthMax = (width <= filters.max_width);
    bool passWidth    = (passWidthMin && passWidthMax);

    // ---------------------------------------------------
    // 2. Depth check with area override
    // ---------------------------------------------------
    // Normal condition: depth >= min_depth
    // If depth < min_depth, allow if area_zx in [min_zx, max_zx].
    bool passDepthMin = 
      (depth >= filters.min_depth) ||
      (
        (depth < filters.min_depth) &&
        (area_zx >= filters.min_zx && area_zx <= filters.max_zx)
      );
    // Still enforce max_depth
    bool passDepthMax = (depth <= filters.max_depth);
    bool passDepth    = (passDepthMin && passDepthMax);

    // ---------------------------------------------------
    // 3. Height check (normal)
    // ---------------------------------------------------
    bool passHeight = 
      (height >= filters.min_height && height <= filters.max_height);

    // ---------------------------------------------------
    // Final check: must pass all
    // ---------------------------------------------------
    if (passWidth && passDepth && passHeight)
    {
      output_detections.detections.push_back(detection);
    }
  }
}


#endif // UTILS_H_
