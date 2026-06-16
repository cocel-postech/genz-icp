<div align="center">
    <h1>GenZ-ICP</h1>
    <a href="https://github.com/cocel-postech/genz-icp/tree/master/cpp/genz_icp"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/cocel-postech/genz-icp/tree/master/python/genz_icp"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/cocel-postech/genz-icp/tree/master/ros"><img src="https://img.shields.io/badge/ROS1-blue" /></a>
    <a href="https://github.com/cocel-postech/genz-icp/tree/master/ros"><img src="https://img.shields.io/badge/ROS2-blue" /></a>
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

  <p align="center">
    <strong>(May 19, 2026)</strong> pip installation is now live:
    <br/>
    <a href="https://pypi.org/project/genz-icp/"><img src="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=18&duration=2500&pause=800&center=true&vCenter=true&width=320&height=30&lines=%24+pip+install+genz-icp" alt="pip install genz-icp"/></a>
</p>

  [GenZ-ICP][arXivlink] is a **Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting**
</div>

[arXivlink]: https://arxiv.org/abs/2411.06766

## Install

```sh
pip install genz-icp
```

Next, follow the instructions on how to run the system by typing:

```sh
genz_icp_pipeline --help
```

This should print a help message with supported dataloaders and options.

For advanced instructions on the Python package please see [this README](https://github.com/cocel-postech/genz-icp/blob/master/python/README.md)

## ROS support

<details>
<summary>ROS 1</summary>

### Build

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/cocel-postech/genz-icp.git
cd ..
catkin build genz_icp --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash
```

### Run with pre-tuned config

```sh
roslaunch genz_icp odometry.launch topic:=<topic_name> config_file:=<config_file_name>.yaml
rosbag play <rosbag_file_name>.bag
```

### Run with only topic

```sh
roslaunch genz_icp odometry.launch topic:=<topic_name>
rosbag play <rosbag_file_name>.bag
```

Examples and download links for demo datasets are available in [ros/README.md](https://github.com/cocel-postech/genz-icp/blob/master/ros/README.md).

Parameter tuning guide: [ros/config/parameter_tuning_guide.md](https://github.com/cocel-postech/genz-icp/blob/master/ros/config/parameter_tuning_guide.md)

</details>

<details>
<summary>ROS 2</summary>

### Build

```sh
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/cocel-postech/genz-icp.git
cd ..
colcon build --packages-select genz_icp --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ~/colcon_ws/install/setup.bash
```

### Run with pre-tuned config

```sh
ros2 launch genz_icp odometry.launch.py topic:=<topic_name> config_file:=<config_file_name>.yaml
ros2 bag play <rosbag_file_name>.db3
```

### Run with only topic

```sh
ros2 launch genz_icp odometry.launch.py topic:=<topic_name>
ros2 bag play <rosbag_file_name>.db3
```

Examples and download links for demo datasets are available in [ros/README.md](https://github.com/cocel-postech/genz-icp/blob/master/ros/README.md).

Parameter tuning guide: [ros/config/parameter_tuning_guide.md](https://github.com/cocel-postech/genz-icp/blob/master/ros/config/parameter_tuning_guide.md)

</details>

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
For the LiDAR-inertial odometry (LIO) extension of GenZ-ICP, please also refer to [GenZ-LIO][genzlioarxivlink].
```
@article{lee2026genzlio,
  title={{GenZ-LIO: Generalizable LiDAR-Inertial Odometry Beyond Confined--Open Boundaries}},
  author={Lee, Daehan and Lim, Hyungtae and Kim, Seongjun and Rho, Soonbin and Lee, Changhyeon and Park, Sanghyun and Hong, Junwoo and Choi, Eunseon and Jo, Hyunyoung and Han, Soohee},
  journal={arXiv preprint arXiv:2603.16273},
  year={2026}
}
```

[genzicpIEEElink]: https://ieeexplore.ieee.org/document/10753079
[genzlioarxivlink]: https://arxiv.org/abs/2603.16273

## :sparkles: Contributors

Like [KISS-ICP](https://github.com/PRBonn/kiss-icp),
we envision GenZ-ICP as a community-driven project, we love to see how the project is growing thanks to the contributions from the community. We would love to see your face in the list below, just open a Pull Request!

<a href="https://github.com/cocel-postech/genz-icp/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=cocel-postech/genz-icp" />
</a>

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

[dhlink]: https://github.com/Daehan2Lee
[htlink]: https://github.com/LimHyungTae
