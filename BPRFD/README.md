# BPRFD

## A robust method to detect balcony's plane and extract robust feature.

## Prerequisites

**Ubuntu** and **ROS**

- Ubuntu  18.04.
- ROS Melodic.    [ROS Installation](http://wiki.ros.org/ROS/Installation)

## Usage

It is easy for you to run bprfd, just do as follows:

```
# make sure you have installed git in your computer. if not, you should do 'sudo apt-get install git'
git clone https://github.com/THU-UAV-GROUP/Drone_simulation.git

# you should move the folder 'BPRFD' to your build folder (such as ".../catkin_ws/src/") before the following steps.
cd catkin_ws/src

catkin build bprfd

source devel/setup.bash

# you should change 'bag_file' in bprfd.launch to your rosbag's absolutely path, then:
roslaunch bprfd bprfd.launch
```

## Dataset

Note that I've changed the dataset's name to "kitti.bag" after downloaded the dataset.

Link: [kitti_2011_09_26_drive_0005_synced.bag](https://pan.baidu.com/share/init?surl=sYWHzF11RpyEW25cQ_iNGA)
Password: b6pd
