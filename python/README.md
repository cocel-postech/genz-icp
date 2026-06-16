<div align="center">
    <h1>GenZ-ICP</h1>
    <a href="https://github.com/cocel-postech/genz-icp/releases"><img src="https://img.shields.io/github/v/release/cocel-postech/genz-icp?label=version" /></a>
    <a href="https://github.com/cocel-postech/genz-icp/blob/master/LICENSE"><img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License: MIT" /></a>
    <a href="https://github.com/cocel-postech/genz-icp"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <br />
    <br />
    <a href="https://www.youtube.com/watch?v=EyTJbdC_AA4">Demo</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/cocel-postech/genz-icp/blob/master/README.md#install">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/cocel-postech/genz-icp/tree/master/ros">ROS</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://arxiv.org/abs/2411.06766">Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/cocel-postech/genz-icp/issues">Contact Us</a>
  <br />
  <br />
  <p align="center"><img src=../pictures/GenZ-ICP_visualizer.gif alt="animated" width="1100" /></p>

[GenZ-ICP](https://arxiv.org/abs/2411.06766) is a **Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting**
</div>

<hr />

## Install

```sh
pip install genz-icp
```

If you also want to install all the dependencies, like Open3D/Polyscope for visualization *(recommended)*:

```sh
pip install "genz-icp[all]"
```

## Running the system

Next, follow the instructions on how to run the system by typing:

```sh
genz_icp_pipeline --help
```

This should print a help message with supported dataloaders and options.

### Config

You can generate a default config `genz_icp.yaml` by typing:

```sh
genz_icp_dump_config
```

Now, you can modify the parameters and pass the file to `--config` when running `genz_icp_pipeline` as follows:

```sh
genz_icp_pipeline --config /path/to/genz_icp.yaml --visualize /path/to/data --topic /pointcloud_topic
```

### Built-in pre-tuned configs (installed with pip)

Pre-tuned YAML files are packaged together with `genz-icp`, so you can use them by filename directly.

Available pre-tuned configs:

- `corridor.yaml`: Ground-Challenge dataset, `Corridor1` and `Corridor2` sequences (topic: `/velodyne_points`)
- `exp07.yaml`: Hilti-Oxford dataset, `Exp07` sequence (topic: `/hesai/pandar`)
- `kitti.yaml`: KITTI odometry dataset (topic: `/velodyne_points`)
- `long_corridor.yaml`: SubT-MRS dataset, `Long_Corridor` sequence (topic: `/velodyne_points`)
- `newer_college.yaml`: Newer College dataset, `01_short_experiment`, `02_long_experiment`, `07_parklandmount` sequences (topic: `/os1_cloud_node/points`)
- `indoor.yaml`: General indoor environments (topic set by user input)
- `outdoor.yaml`: General outdoor environments (topic set by user input)

Example:

```sh
genz_icp_pipeline --config long_corridor.yaml --visualize /path/to/data --topic /velodyne_points
```

You can still pass your own local YAML path:

```sh
genz_icp_pipeline --config /path/to/my_config.yaml --visualize /path/to/data --topic /pointcloud_topic
```

<details>
<summary><strong>⚠️ Python version compatibility for built-in pre-tuned configs (important for Python < 3.9)</strong></summary>

Above built-in pre-tuned configs can be used by filename (without specifying a full path) only when using **Python >= 3.9**.

This is because the package relies on `importlib.resources.files`, which is available starting from Python 3.9.
<br>

* **Python >= 3.9 (recommended)**
  You can directly use config names:

  ```sh
  genz_icp_pipeline --config kitti.yaml --visualize /path/to/data --topic /velodyne_points
  ```

* **Python < 3.9**
  Built-in configs cannot be resolved by name.
  Instead, clone the repository and provide the path manually:

  ```sh
  git clone https://github.com/cocel-postech/genz-icp.git

  genz_icp_pipeline \
    --config /path/to/genz-icp/python/genz_icp/config/pretuned/kitti.yaml \
    --visualize /path/to/data \
    --topic /velodyne_points
  ```

</details>

### Install Python API (developer mode)

If you plan to modify the code, the main requirements are a modern C++ compiler and `pip`.
In Ubuntu-based systems:

```sh
sudo apt install g++ python3-pip
```

Then clone and install editable mode:

```sh
git clone https://github.com/cocel-postech/genz-icp.git
cd genz-icp
python3 -m pip install -e ./python --no-build-isolation
```

### Install Python API (expert mode)

If you want more control over the build, install CMake and dependencies explicitly:

```sh
sudo apt install build-essential libeigen3-dev libtbb-dev pybind11-dev ninja-build
```

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
[arXivLink]: https://arxiv.org/abs/2411.06766
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
