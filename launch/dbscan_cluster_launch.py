import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def load_composable_node_param(param_path, context):
    """Helper function to load parameters from a YAML file."""
    try:
        yaml_path = LaunchConfiguration(param_path).perform(context)
        with open(yaml_path, "r") as f:
            return yaml.safe_load(f)["dbscan_cluster_node"]["ros__parameters"]
    except Exception as e:
        raise RuntimeError(f"Failed to load YAML file at '{yaml_path}': {e}")


def launch_setup(context, *args, **kwargs):
    # Package name
    pkg = "lidar_object_detection"

    # Define the DBSCAN Cluster Node
    dbscan_cluster_component = ComposableNode(
        package=pkg,
        plugin="lidar_object_detection::dbscan::DBScanClusterNode",
        name="dbscan_cluster_node",
        remappings=[
            ("velodyne_points", LaunchConfiguration("input_pointcloud")),
            ("clustered_points", LaunchConfiguration("output_clusters")),
            ("detected_objects", LaunchConfiguration("output_detections")),
        ],
        parameters=[load_composable_node_param("dbscan_param_path", context)],
    )

    # Define the container
    container = ComposableNodeContainer(
        name="dbscan_cluster_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[dbscan_cluster_component],
        output="screen",
    )

    return [container]


def generate_launch_description():
    """Generate the launch description."""
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    return LaunchDescription(
        [
            add_launch_arg("input_pointcloud", "/velodyne_points"),
            add_launch_arg("output_clusters", "/output/clustered_points"),
            add_launch_arg("output_detections", "/output/detected_objects"),
            add_launch_arg(
                "dbscan_param_path",
                [
                    FindPackageShare("lidar_object_detection"),
                    "/cfg/cfg/dbscan_cluster_node_params.yaml",
                ],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
