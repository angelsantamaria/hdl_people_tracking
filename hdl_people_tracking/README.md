# hdl_people_tracking

ROS 2 composable nodes for 3D LiDAR people detection and tracking.

This package contains the runtime nodes. Message definitions live in
`hdl_people_tracking_msgs`.

## Nodes

- `hdl_people_tracking::HdlPeopleDetectionNode`
- `hdl_people_tracking::HdlPeopleTrackingNode`

## Detection Pipeline

1. Subscribe to `points`.
2. Crop to a configurable forward sector, default 90 degrees and 5 m maximum range.
3. Downsample with a voxel grid.
4. Cluster foreground candidates with Euclidean clustering and the Haselich/Marcel-style DP-means splitter.
5. Optionally run the Kidono classifier.
6. Publish clusters and debug point clouds.

Classification defaults to disabled because the bundled model is LiDAR-specific
and can reject valid people when used with a different sensor/intensity profile.

## Launch

```bash
ros2 launch hdl_people_tracking hdl_people_tracking.launch.py
```

The included launch remaps `points` to `/b2/rslidar_points`.

Useful detector parameters include `euclidean_cluster_tolerance` default `0.2`
and `dpmeans_split_threshold` default `0.45`.
