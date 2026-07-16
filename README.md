# hdl_people_tracking

ROS 2 people detection and tracking from a 3D LiDAR point cloud.

The detector crops the input cloud to the robot front sector before clustering:

- default field of view: 90 degrees, 45 degrees each side
- default range: 0.1 m to 5.0 m
- default forward axis: `+x`
- default lateral axis: `+y`

The tracker consumes detected human clusters and publishes constant-velocity
Kalman tracks.

## Build

```bash
colcon build --packages-select hdl_people_tracking_msgs hdl_people_tracking
```

## Run

```bash
ros2 launch hdl_people_tracking hdl_people_tracking.launch.py
```

The launch file remaps the detector input `points` topic to `/b2/rslidar_points`.

## Input

- `points` (`sensor_msgs/msg/PointCloud2`): input 3D LiDAR cloud

## Outputs

- `clusters` (`hdl_people_tracking_msgs/msg/ClusterArray`): detected clusters
- `tracks` (`hdl_people_tracking_msgs/msg/TrackArray`): tracked people
- `markers` (`visualization_msgs/msg/MarkerArray`): tracked-people markers
- `detection_markers` (`visualization_msgs/msg/MarkerArray`): detection markers
- `cropped_points` (`sensor_msgs/msg/PointCloud2`): cropped/downsampled candidate points
- `cluster_points` (`sensor_msgs/msg/PointCloud2`): all clustered points
- `human_points` (`sensor_msgs/msg/PointCloud2`): clustered points accepted as human

## Key Parameters

- `max_detection_range` default `5.0`
- `front_fov_deg` default `90.0`
- `downsample_resolution` default `0.1`
- `enable_classification` default `false`
- `euclidean_cluster_tolerance` default `0.2`
- `dpmeans_split_threshold` default `0.45`
- `cluster_min_pts` default `10`
- `cluster_max_pts` default `2048`
