from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare path to the parameters YAML file
    param_file = LaunchConfiguration('param_file', default=os.path.join(
        os.path.dirname(__file__), '../config/euclidean_cluster_node_params.yaml'))

    # Declare the parameter file as a launch argument
    declare_param_file = DeclareLaunchArgument(
        'param_file',
        default_value=param_file,
        description='Path to the parameter YAML file'
    )

    # Define the node
    euclidean_cluster_node = Node(
        package='lidar_object_detection',
        executable='euclidean_cluster_node',
        name='euclidean_cluster_node',
        parameters=[param_file],
        remappings=[
            ('velodyne_points', '/velodyne_points'),
            ('clustered_points', '/output/clustered_points')
        ]
    )

    return LaunchDescription([
        declare_param_file,
        euclidean_cluster_node
    ])
