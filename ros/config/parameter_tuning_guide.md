# GenZ-ICP Parameter Tuning Guide

GenZ-ICP is designed to perform well across diverse environments. For optimal performance on each dataset, parameter tuning is recommended. 
This section provides tips for tuning GenZ-ICP's parameters.

## :star2: Key Parameters

### `voxel_size`
: Voxel size of local map (default: `0.25`)
+ Larger **_voxel_size_** speed up processing but reduce accuracy.
+ It is recommended to adjust **_voxel_size_** proportionally to the **scale** of the environment: **larger** values for wide **outdoor** spaces and **smaller** values for narrow **indoor** spaces.

### `max_points_per_voxel`
: Maximum points in a single voxel (default: `1`)
+ Lower **_max_points_per_voxel_** speed up processing but reduce accuracy.
+ Similar to **_voxel_size_**, this parameter should also be adjusted proportionally to the **scale** of the environment.

### `planarity_threshold`
: Threshold for planarity classification (default: `0.1`)
+ Lower **_planarity_threshold_** classifies planars pair more strictly.
+ In narrow **indoor** environments, a smaller **_max_points_per_voxel_** is typically used, which reduces the number of neighboring points available for covariance calculation during planarity classification. As a result, even planar surfaces can exhibit relatively high local surface variation.
  To prevent rejecting valid planar pairs due to this, a relatively higher **_planarity_threshold_** is recommended for indoor environments.
+ Conversely, in wide **outdoor** environments, a larger **_max_points_per_voxel_** increases the number of neighboring points, resulting in more reliable local surface variation values.
  Therefore, in outdoor environments, lowering the planarity_threshold is recommended to achieve stricter planarity classification.

## :zap: Minor Parameters

### `deskew`
: Enables or disables deskewing of LiDAR scans (default: `false`)
+ When the platform exhibits aggressive motion, enabling deskewing can lead to inaccuracies.
+ Additionally, the effect of deskewing diminishes as the platform's speed decreases.
+ Therefore, for platforms like hand-held devices or quadruped robots that exhibit slow or aggressive motion, setting **_deskew_** to `false` is recommended.
+ Conversely, for platforms with high-speed and smooth motion, such as vehicles used in datasets like KITTI or MulRan, setting **_deskew_** to `true` is recommended.

### `map_cleanup_radius`
: Radius of local map (default: `max_range`)
+ The default value for **_map_cleanup_radius_** is equal to the LiDAR's **_max_range_**.
+ In spaces larger than the LiDAR's **_max_range_**, where the platform revisits previously visited areas, it is recommended to increase the **_map_cleanup_radius_**.
+ However, excessively high values can consume a lot of memory and may lead to inaccurate results. Therefore, it is recommended to keep the value below `300`.

### `max_points_per_voxelized_scan`
: Maximum required points in a voxelized scan (default: `2000`)
+ If the number of points in the voxelized LiDAR input is too large, it can cause CPU overload during the normal estimation process.
+ This issue primarily occurs when transitioning from narrow indoor spaces to wide outdoor spaces.
+ Therefore, if the number of points in the voxelized LiDAR input exceeds **_max_points_per_voxelized_scan_**, GenZ-ICP automatically adjusts the number of downsampled points by incrementally increasing the downsampling leaf_size until the count falls below the **_max_points_per_voxelized_scan_**.

### `min_points_per_voxelized_scan`
: Minimum required points in a voxelized scan (default: `1800`)
+ Conversely, if the number of points in the voxelized LiDAR input is too small, it may lead to inaccurate results.
+ This typically occurs when transitioning from wide spaces to narrow spaces.
+ Therefore, if the number of points in the voxelized LiDAR input is less than **_min_points_per_voxelized_scan_**, GenZ-ICP automatically adjusts the number of downsampled points by incrementally decreasing the downsampling leaf size until the count exceeds the **_min_points_per_voxelized_scan_**.