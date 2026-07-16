from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    points_topic = LaunchConfiguration('points_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all hdl_people_tracking nodes and topics')
    declare_points_topic = DeclareLaunchArgument(
        'points_topic',
        default_value='rslidar_points',
        description='PointCloud2 input topic, relative to namespace unless absolute')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    container = ComposableNodeContainer(
        name='hdl_people_tracking_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hdl_people_tracking',
                plugin='hdl_people_tracking::HdlPeopleDetectionNode',
                name='hdl_people_detection_node',
                namespace=namespace,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'downsample_resolution': 0.02,
                    'min_detection_range': 1.0,
                    'max_detection_range': 5.0,
                    'min_detection_height': -0.3,
                    'max_detection_height': 1.0,
                    'front_fov_deg': 70.0,
                    'forward_axis': 'x',
                    'lateral_axis': 'y',
                    'height_axis': 'z',
                    'cluster_min_pts': 10,
                    'cluster_max_pts': 2048,
                    'cluster_min_size_x': 0.2,
                    'cluster_min_size_y': 0.2,
                    'cluster_min_size_z': 0.2,
                    'cluster_max_size_x': 1.0,
                    'cluster_max_size_y': 1.0,
                    'cluster_max_size_z': 2.0,
                    'euclidean_cluster_tolerance': 0.2,
                    'dpmeans_split_threshold': 0.45,
                    'enable_classification': False
                }],
                remappings=[
                    ('points', points_topic)
                ]
            ),
            ComposableNode(
                package='hdl_people_tracking',
                plugin='hdl_people_tracking::HdlPeopleTrackingNode',
                name='hdl_people_tracking_node',
                namespace=namespace,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'remove_trace_thresh': 1.0,
                    'human_radius': 0.4,
                    'track_single_target_mode': True,
                    'track_init_centerline_only': True,
                    'track_init_centerline_angle_deg': 5.0,
                    'track_init_min_range': 2.0,
                    'track_init_max_range': 4.0,
                    'track_init_preferred_range': 3.0,
                    'track_association_max_gap_sec': 0.5,
                    'track_association_max_angle_delta_deg': 15.0
                }]
            ),
            ComposableNode(
                package='hdl_people_tracking',
                plugin='hdl_people_tracking::HdlPeopleFollowTargetNode',
                name='hdl_people_follow_target_node',
                namespace=namespace,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'base_frame': 'b2/base_link',
                    'tracks_topic': 'hdl_people_tracking/tracks',
                    'follow_transform_topic': 'hdl_people_tracking/follow_person_transform',
                    'track_frame_prefix': 'person_track_',
                    'transform_timeout_sec': 0.05
                }],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ]
            )
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_namespace,
        declare_points_topic,
        declare_use_sim_time,
        container,
    ])
