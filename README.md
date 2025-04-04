<div align="center">
    <h1>GenZ-ICP (Optimized)</h1>
    <a href="https://github.com/cocel-postech/genz-icp/tree/master/cpp/genz_icp"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/cocel-postech/genz-icp/tree/master/ros"><img src="https://img.shields.io/badge/ROS1-Noetic-blue" /></a>
    <a href="https://github.com/cocel-postech/genz-icp/tree/master/ros"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href=""><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://github.com/cocel-postech/genz-icp/blob/master/LICENSE"><img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License: MIT" /></a>
    <a href="https://ieeexplore.ieee.org/document/10753079"><img src="https://img.shields.io/badge/DOI-10.1109/LRA.2024.3498779-004088.svg"/>
    <br />
    <br />
    <a href="https://www.youtube.com/watch?v=EyTJbdC_AA4">Demo</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://www.youtube.com/watch?v=CU6aAiTIO6Y">Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/cocel-postech/genz-icp/blob/master/README.md">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/cocel-postech/genz-icp/tree/master/ros">ROS</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://arxiv.org/abs/2411.06766">Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/cocel-postech/genz-icp/issues">Contact Us</a>
  <br />
  <br />
  <p align="center"><img src=pictures/GenZ-ICP.gif alt="animated" width="500" /></p>

  [GenZ-ICP][arXivlink] is a **Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting**
</div>

[arXivlink]: https://arxiv.org/abs/2411.06766

---
## :anger: Optimizations:
| **File**             | **Optimization**                              | **Description**                                                                                              | **Estimated CPU Usage Reduction**         |
|----------------------|-----------------------------------------------|--------------------------------------------------------------------------------------------------------------|-------------------------------------------|
| *Registration.cpp* | Vectorized `TransformPoints`                  | Use Eigen matrix operations instead of per-point `std::transform`                                         | 5–15% of total runtime                   |
| *Registration.cpp* | Precomputed `kernel_squared`                  | Compute `kernel * kernel` once instead of repeatedly in `Weight`                                           | 0.5–1% of total runtime                  |
| *Registration.cpp* | Improved Weight Function                      | Reduce operations in weight computation from 4 to 2 multiplications + 1 addition                          | 2–5% of total runtime                    |
| *VoxelHashMap.cpp* | Squared Distance in `GetClosestNeighbor`      | Use `.squaredNorm()` for comparisons, with one `sqrt` at the end                                           | 10–20% of total runtime                  |
| *VoxelHashMap.cpp* | Precomputed Statistics in `GetClosestNeighbor`| Use `sum_points` and `sum_outer` for O(27) computation vs. O(27 * P) per query                            | 15–25% of total runtime                  |
| *VoxelHashMap.cpp* | Efficient `AddPoints`                         | Simplify with `for` loop and `map_.at()`, maintaining precomputed stats                                  | 1–3% of total runtime                    |
| *VoxelHashMap.cpp* | Optimized `RemovePointsFarFromLocation`       | Use iterator-based `erase` for safe and efficient removal                                                 | 1–2% of total runtime                    |
| *Overall*          | Combined Effect                               | Cumulative impact across all optimizations, varying with point cloud size and voxel density               | **20–50% of total runtime**              |
---

## :gear: How to build & run

### ROS1

#### How to build

You should not need any extra dependency, just clone and build:
    
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/cocel-postech/genz-icp.git
cd ..
catkin build genz_icp --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash
```

#### How to run

#### Option 1

If you want to use a pre-tuned parameter set, you need to provide the **config file** with the **topic name** as arguments:

```sh
roslaunch genz_icp odometry.launch topic:=<topic_name> config_file:=<config_file_name>.yaml
```
```sh
rosbag play <rosbag_file_name>.bag
```

Examples and download links for **demo datasets** can be found [here][ros_readme_link]

[ros_readme_link]: https://github.com/cocel-postech/genz-icp/blob/master/ros/README.md

#### Option 2

Otherwise, the only required argument to provide is the **topic name**:

```sh
roslaunch genz_icp odometry.launch topic:=<topic_name>
```
```sh
rosbag play <rosbag_file_name>.bag
```
 
Check out the tuning guide for the parameters of GenZ-ICP at this [link][tuning_guide_link]

[tuning_guide_link]: https://github.com/cocel-postech/genz-icp/blob/master/ros/config/parameter_tuning_guide.md

### ROS2

#### How to build

You should not need any extra dependency, just clone and build:
    
```sh
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/cocel-postech/genz-icp.git
cd ..
colcon build --packages-select genz_icp --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/colcon_ws/install/setup.bash
```

#### How to run

The only required argument to provide is the **topic name**:

```sh
ros2 launch genz_icp odometry.launch.py topic:=<topic_name>
```

and then,

```sh
ros2 bag play <rosbag_file_name>.mcap
```

Check out the tuning guide for the parameters of GenZ-ICP at this [link][tuning_guide_link]

## :pushpin: Todo list
  ✔️ Code optimization to reduce CPU load
- [ ] Python support for GenZ-ICP


## :pencil: Citation

If you use our codes, please cite our paper ([arXiv][arXivLink], [IEEE *Xplore*][genzicpIEEElink])
```
@ARTICLE{lee2024genzicp,
  author={Lee, Daehan and Lim, Hyungtae and Han, Soohee},
  journal={IEEE Robotics and Automation Letters (RA-L)}, 
  title={{GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting}}, 
  year={2025},
  volume={10},
  number={1},
  pages={152-159},
  keywords={Localization;Mapping;SLAM},
  doi={10.1109/LRA.2024.3498779}
}
```

[genzicpIEEElink]: https://ieeexplore.ieee.org/document/10753079

## :pray: Acknowledgement

Many thanks to KISS team—[Ignacio Vizzo][nacholink], [Tiziano Guadagnino][guadagninolink], [Benedikt Mersch][merschlink]—to provide outstanding LiDAR odometry codes!

Please refer to [KISS-ICP][kissicplink] for more information

[nacholink]: https://github.com/nachovizzo
[guadagninolink]: https://github.com/tizianoGuadagnino
[merschlink]: https://github.com/benemer
[kissicplink]: https://github.com/PRBonn/kiss-icp

## :mailbox: Contact information

If you have any questions, please do not hesitate to contact us
* [Daehan Lee][dhlink] :envelope: daehanlee `at` postech `dot` ac `dot` kr
* [Hyungtae Lim][htlink] :envelope: shapelim `at` mit `dot` edu
* [Ali Pahlevani][aplink] :envelope: a.pahlevani1998 `at` gmail `dot` com
(Not a main author - Contributor)

[dhlink]: https://github.com/Daehan2Lee
[htlink]: https://github.com/LimHyungTae
[aplink]: https://github.com/ali-pahlevani
