# hdl_people_tracking

ROS 2 people detection and tracking from a 3D LiDAR point cloud.

The detector crops the input cloud to the robot front sector before clustering:

- default field of view: 90 degrees, 45 degrees each side
- default range: 0.1 m to 5.0 m
- default height slice: 0.5 m to 1.8 m
- default forward axis: `+x`
- default lateral axis: `+y`
- default height axis: `+z`

The tracker consumes detected human clusters and publishes constant-velocity
Kalman tracks.
By default, new tracks are initialized only when an unmatched human cluster is
near the forward centerline; existing tracks can still update anywhere inside
the detector FOV.

After clustering, the detector can reject clusters whose top-down footprint does
not look like a person-sized cylinder front arc. This is intended to suppress
wall fragments and other line-like objects while keeping the existing clustering
logic unchanged.

## Build

```bash
colcon build --packages-select hdl_people_tracking_msgs hdl_people_tracking
```

## Run

```bash
ros2 launch hdl_people_tracking hdl_people_tracking.launch.py
```

The launch file remaps the detector input `points` topic to `/rslidar_points`.

## Input

- `points` (`sensor_msgs/msg/PointCloud2`): input 3D LiDAR cloud

## Outputs

- `hdl_people_tracking/clusters` (`hdl_people_tracking_msgs/msg/ClusterArray`): detected clusters
- `hdl_people_tracking/tracks` (`hdl_people_tracking_msgs/msg/TrackArray`): tracked people
- `hdl_people_tracking/markers` (`visualization_msgs/msg/MarkerArray`): tracked-people markers
- `hdl_people_tracking/detection_markers` (`visualization_msgs/msg/MarkerArray`): detection markers
- `hdl_people_tracking/cropped_points` (`sensor_msgs/msg/PointCloud2`): cropped/downsampled candidate points
- `hdl_people_tracking/cluster_points` (`sensor_msgs/msg/PointCloud2`): all clustered points
- `hdl_people_tracking/human_points` (`sensor_msgs/msg/PointCloud2`): clustered points accepted as human

## Key Parameters

- `max_detection_range` default `5.0`
- `min_detection_height` default `0.5`
- `max_detection_height` default `1.8`
- `front_fov_deg` default `90.0`
- `downsample_resolution` default `0.1`
- `enable_classification` default `false`
- `enable_shape_filter` launch default `true`
- `shape_min_width` default `0.18`
- `shape_max_width` default `0.90`
- `shape_min_radius` default `0.10`
- `shape_max_radius` default `0.45`
- `shape_max_fit_rmse` default `0.08`
- `shape_max_linearity_ratio` default `80.0`
- `euclidean_cluster_tolerance` default `0.2`
- `dpmeans_split_threshold` default `0.45`
- `cluster_min_pts` default `10`
- `cluster_max_pts` default `2048`
- `track_init_centerline_only` launch default `true`
- `track_init_centerline_angle_deg` launch default `5.0`
