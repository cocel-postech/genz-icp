# GenZ-ICP ROS wrappers

## :gear: How to build & Run

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

For example,

1. **Long_Corridor** sequence of SubT-MRS dataset

```sh
roslaunch genz_icp odometry.launch topic:=/velodyne_points config_file:=long_corridor.yaml
```
```sh
rosbag play subt_mrs_long_corridor.bag
```

The original bagfile for the **Long_Corridor** sequence of SubT-MRS dataset can be downloaded from [here][long_corridor_original_link]

`subt_mrs_long_corridor.bag` includes only the `/velodyne_points` topic and can be downloaded from [here][long_corridor_link]

[long_corridor_original_link]: https://superodometry.com/iccv23_challenge_Mul
[long_corridor_link]: https://postechackr-my.sharepoint.com/:u:/g/personal/daehanlee_postech_ac_kr/EduGeuaT5ypBvSQY539XFEgBRSsQSUopeNjalk9jJqyq5Q?e=yFaLEN

Alternatively, you can download it using the following command:

```sh
wget https://postechackr-my.sharepoint.com/:u:/g/personal/daehanlee_postech_ac_kr/EduGeuaT5ypBvSQY539XFEgBRSsQSUopeNjalk9jJqyq5Q?e=2IM8ed&download=1 -O subt_mrs_long_corridor.zip
```

2. **Exp07** Long Corridor sequence of HILTI-Oxford dataset

```sh
roslaunch genz_icp odometry.launch topic:=/hesai/pandar config_file:=exp07.yaml
```
```sh
rosbag play exp07_long_corridor.bag
```

The bagfile for the **Exp07** Long Corridor sequence of HILTI-Oxford dataset can be downloaded from [here][exp07_link]

[exp07_link]: https://hilti-challenge.com/dataset-2022.html

3. **Corridor1** and **Corridor2** sequences of Ground-Challenge dataset

```sh
roslaunch genz_icp odometry.launch topic:=/velodyne_points config_file:=corridor.yaml
```
```sh
rosbag play corridor1.bag && rosbag play corridor2.bag
```

The bagfile for the **Corridor1** and **Corridor2** sequences of Ground-Challenge dataset can be downloaded from [here][ground_challenge_link]

[ground_challenge_link]: https://github.com/sjtuyinjie/Ground-Challenge

4. **short experiment** sequence of Newer-College dataset

```sh
roslaunch genz_icp odometry.launch topic:=/os1_cloud_node/points config_file:=newer_college.yaml
```
```sh
rosbag play rooster_2020-03-10-10-36-30_0.bag && rosbag play rooster_2020-03-10-10-39-18_1.bag && rosbag play rooster_2020-03-10-10-42-05_2.bag && rosbag play rooster_2020-03-10-10-44-52_3.bag && rosbag play rooster_2020-03-10-10-47-39_4.bag && rosbag play rooster_2020-03-10-10-50-26_5.bag && rosbag play rooster_2020-03-10-10-53-13_6.bag && rosbag play rooster_2020-03-10-10-56-00_7.bag && rosbag play rooster_2020-03-10-10-58-47_8.bag && rosbag play rooster_2020-03-10-11-01-34_9.bag
```

The bagfile for the **short experiment** sequence of Newer-College dataset can be downloaded from [here][newer_college_link]

[newer_college_link]: https://ori-drs.github.io/newer-college-dataset/

5. **Seq. 00** of KITTI odometry dataset

```sh
roslaunch genz_icp odometry.launch topic:=/velodyne_points config_file:=kitti.yaml
```
```sh
rosbag play 00.bag
```

The bagfile for the **Seq. 00** of KITTI odometry dataset can be downloaded from [here][00_link]

[00_link]: https://postechackr-my.sharepoint.com/:u:/g/personal/daehanlee_postech_ac_kr/EVxChaGoOiJIr5vTy44DNUIBlAs4Mbld5wj94qJYyWwAKg?e=QjhpLE

Alternatively, you can download it using the following command:

```sh
wget https://postechackr-my.sharepoint.com/:u:/g/personal/daehanlee_postech_ac_kr/EVxChaGoOiJIr5vTy44DNUIBlAs4Mbld5wj94qJYyWwAKg?e=ZPE6JT&download=1 -O 00.zip
```

If you need bag files for other sequences, you can download the original files from [here][kitti_link] and convert them using [lidar2rosbag_kitti][lidar2rosbag_kitti_link]

[kitti_link]: https://www.cvlibs.net/datasets/kitti/eval_odometry.php
[lidar2rosbag_kitti_link]: https://github.com/AbnerCSZ/lidar2rosbag_KITTI

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
colcon build --packages-select genz_icp --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ~/colcon_ws/install/setup.bash
```

#### How to run

#### Option 1

If you want to use a pre-tuned parameter set, you need to provide the **config file** with the **topic name** as arguments:

```sh
ros2 launch genz_icp odometry.launch.py topic:=<topic_name> config_file:=<config_file_name>.yaml
```
```sh
ros2 bag play <rosbag_file_name>.mcap
```

#### Option 2

Otherwise, the only required argument to provide is the **topic name**:

```sh
ros2 launch genz_icp odometry.launch.py topic:=<topic_name>
```
```sh
ros2 bag play <rosbag_file_name>.mcap
```

Check out the tuning guide for the parameters of GenZ-ICP at this [link][tuning_guide_link]
