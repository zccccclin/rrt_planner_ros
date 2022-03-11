# RRT for Path Planning in ROS

## Introduction
This repository contains my C++ implementation of Rapidly Exploring Random Trees (RRT) algorithm in ROS to plan collision-free paths in a 2D environment.

My implementation includes the following elements:
1. RRT algorithm (C++ planner node that implements basic RRT with configurable bias direction towards goal)
2. RRT* algorithm (RRT* Improvement implemented within planner node)
3. Map server (Map server node for loading of maps or creation of maps)

## How to run it?
Download the modified rrt_planner_ros.zip and extract content to src folder after `cd src` use gitclone in next step. Ignore this step if using `git clone`.
#### 1. Package setup
```
cd ~
mkdir rrt_ws
cd rrt_ws
mkdir src
cd src
git clone https://github.com/zccccclin/rrt_planner_ros.git  
cd ..
catkin build
```
#### 2. Make python node executable and source workspace
`source devel/setup.bash` is always necessary for new terminals
```
source devel/setup.ash
cd src/rrt_planner_ros/src
chmod +x map_server.py
```
###3. Running the Package
```
roslaunch rrt_planner rrt_planner.launch
```
## How to use it?
The package includes a custom map server node that allows for creation of own map as well as the original ROS map server node that simply reads the maps.
To use the **original ROS map server** node set custom_map_server argument to 0 during launch step: `roslaunch rrt_planner rrt_planner.launch custom_map_server:=0`. The map parameters can be changed at [/cfg/map.yaml](https://github.com/zccccclin/rrt_planner_ros/blob/473e0e9d37aec447e17b8709e759ba014eef21f4/cfg/map.yaml).

For the **custom map server node** roslaunch normally or set custom_map_server argument to 1. Following sections will explain the features for custom map server node.
### 1. Loading example/preconfigured maps
Move preconfigured maps into [/resource](https://github.com/zccccclin/rrt_planner_ros/tree/main/resources) folder.
After `roslaunch rrt_planner rrt_planner.launch`, on the same terminal enter **1** for loading of map.
Enter **map file name** (eg. map1.png). Image window will appear and Rviz window should load up the occupancy grid as well.
![Loaded images](https://github.com/zccccclin/rrt_planner_ros/blob/eca1040a989f796766302cef5ad7ce52150e04a6/README_img/load.png)

