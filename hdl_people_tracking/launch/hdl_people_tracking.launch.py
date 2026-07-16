from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='hdl_people_tracking_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hdl_people_tracking',
                plugin='hdl_people_tracking::HdlPeopleDetectionNode',
                name='hdl_people_detection_node',
                parameters=[{
                    'downsample_resolution': 0.02,
                    'min_detection_range': 0.5,
                    'max_detection_range': 5.0,
                    'min_detection_height': -0.25,
                    'max_detection_height': 0.25,
                    'front_fov_deg': 60.0,
                    'forward_axis': 'x',
                    'lateral_axis': 'y',
                    'height_axis': 'z',
                    'cluster_min_pts': 10,
                    'cluster_max_pts': 2048,
                    'cluster_min_size_x': 0.2,
                    'cluster_min_size_y': 0.2,
                    'cluster_min_size_z': 0.3,
                    'cluster_max_size_x': 0.5,
                    'cluster_max_size_y': 0.5,
                    'cluster_max_size_z': 1.0,
                    'euclidean_cluster_tolerance': 0.2,
                    'dpmeans_split_threshold': 0.45,
                    'enable_shape_filter': True,
                    'shape_min_width': 0.18,
                    'shape_max_width': 0.90,
                    'shape_min_depth': 0.02,
                    'shape_min_radius': 0.10,
                    'shape_max_radius': 0.45,
                    'shape_max_fit_rmse': 0.05,
                    'shape_max_linearity_ratio': 80.0,
                    'enable_classification': True
                }],
                remappings=[
                    ('points', '/rslidar_points')
                ]
            ),
            ComposableNode(
                package='hdl_people_tracking',
                plugin='hdl_people_tracking::HdlPeopleTrackingNode',
                name='hdl_people_tracking_node',
                parameters=[{
                    'remove_trace_thresh': 1.0,
                    'human_radius': 0.4
                }]
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])
