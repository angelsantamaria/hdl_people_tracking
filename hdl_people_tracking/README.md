# hdl_people_tracking

ROS 2 composable nodes for 3D LiDAR people detection and tracking.

This package contains the runtime nodes. Message definitions live in
`hdl_people_tracking_msgs`.

## Nodes

- `hdl_people_tracking::HdlPeopleDetectionNode`
- `hdl_people_tracking::HdlPeopleTrackingNode`

## Detection Pipeline

1. Subscribe to `points`.
2. Crop to a configurable forward sector, default 90 degrees, 5 m maximum range, and 0.5-1.8 m height.
3. Downsample with a voxel grid.
4. Cluster foreground candidates with Euclidean clustering and the Haselich/Marcel-style DP-means splitter.
5. Optionally reject non-human-shaped clusters with an XY arc/linearity filter.
6. Optionally run the Kidono classifier.
7. Publish clusters and debug point clouds.

Classification defaults to disabled because the bundled model is LiDAR-specific
and can reject valid people when used with a different sensor/intensity profile.

## Launch

```bash
ros2 launch hdl_people_tracking hdl_people_tracking.launch.py
```

The included launch remaps `points` to `/rslidar_points`.
Published detector and tracker topics are relative and use the
`hdl_people_tracking/` prefix.

Useful detector parameters include `euclidean_cluster_tolerance` default `0.2`
and `dpmeans_split_threshold` default `0.45`. The height slice is controlled by
`min_detection_height` default `0.5` and `max_detection_height` default `1.8`.
The launch enables `enable_shape_filter`, which rejects clusters whose XY
footprint is too flat, too linear, or incompatible with a person-sized cylinder
front arc.
