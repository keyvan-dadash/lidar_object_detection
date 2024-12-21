from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Define the component node
    euclidean_cluster_node = ComposableNode(
        package='lidar_object_detection',
        plugin='lidar_object_detection::euclidean_cluster::EuclideanClusterNode',
        name='euclidean_cluster_node',
        parameters=[
            {
                'use_height': True,
                'min_cluster_size': 10,
                'max_cluster_size': 1000,
                'distance_threshhold_seg': 0.1,
                'tolerance': 0.1
            }
        ],
        remappings=[
            ('velodyne_points', 'velodyne_points'),
            ('clustered_points', '/output/clustered_points')
        ]
    )

    # Create a container to load the component
    container = ComposableNodeContainer(
        name='euclidean_cluster_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded container
        composable_node_descriptions=[euclidean_cluster_node],
        output='screen',
    )

    return LaunchDescription([container])
