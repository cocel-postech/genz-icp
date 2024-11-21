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
[long_corridor_link]: https://cocel.synology.me:5001/sharing/JZQalfEqQ


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
rosbag play corridor1.bag
```

The bagfile for the **Corridor1** and **Corridor2** sequences of Ground-Challenge dataset can be downloaded from [here][ground_challenge_link]

[ground_challenge_link]: https://github.com/sjtuyinjie/Ground-Challenge

#### Option 2

Otherwise, the only required argument to provide is the **topic name**:

```sh
roslaunch genz_icp odometry.launch topic:=<topic_name>
```
```sh
rosbag play <rosbag_file_name>.bag
```

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