# Overfill Detection

## Overview
This ROS package implements a straightforward overfill detection algorithm using a 2D Lidar sensor. The primary objective is to determine whether a tote has been overfilled by the operator. For demonstration purposes, a 2D Lidar sensor is placed on the box in such a way that its scanning plane is positioned just above the height of the tote. In the event that any part of an object protrudes from the tote, the Lidar sensor will detect it. The effectiveness of this solution has been validated through testing with objects positioned at various locations. Notably, this solution is capable of detecting objects of nearly any size and shape. 

## Dependencies
* ROS 2 Humble
* Ubuntu 22.04
* Gazebo

## Build Instructions
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/NehaMadhekar09/overfill_detection.git

cd ..

rosdep install -i --from-path src --rosdistro humble -y

colcon build 

. install/setup.bash

```
## Run instructions
### 1. Go to directory
```
cd ros2_ws
```
### 2. Source the workspace
```
. install/setup.bash
```
### 3. Run walker node with launch file
```
ros2 launch overfill_detection launch.py
```
## Test instructions

To test the solution, objects of varying sizes are spawned in Gazebo at different locations. If overfill is detected, the solution will print a message indicating so.

Below are the commands to spawn objects in Gazebo. Objects can be deleted after the test.

```
ros2 service call /spawn_entity gazebo_msgs/SpawnEntity "{name: 'my_box1', xml: '<sdf version=\"1.7\"><model name=\"my_box\"><pose>0.6 0 0.75 0 0 0</pose><static>false</static><link name=\"link\"><collision name=\"collision\"><geometry><box><size>0.2 0.2 1.5</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.2 0.2 1.5</size></box></geometry></visual></link></model></sdf>'}"

```

```
ros2 service call /spawn_entity gazebo_msgs/SpawnEntity "{name: 'my_box2', xml: '<sdf version=\"1.7\"><model name=\"my_box\"><pose>0.6 0 0.75 0.1 0 0</pose><static>false</static><link name=\"link\"><collision name=\"collision\"><geometry><box><size>0.3 0.3 1.5</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.3 0.3 1.5</size></box></geometry></visual></link></model></sdf>'}"
```

```
ros2 service call /spawn_entity gazebo_msgs/SpawnEntity "{name: 'my_box3', xml: '<sdf version=\"1.7\"><model name=\"my_box\"><pose>1 0 0.75 0 0 0</pose><static>false</static><link name=\"link\"><collision name=\"collision\"><geometry><box><size>0.3 0.3 1.5</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.3 0.3 1.5</size></box></geometry></visual></link></model></sdf>'}"
```

```
ros2 service call /spawn_entity gazebo_msgs/SpawnEntity "{name: 'my_box4', xml: '<sdf version=\"1.7\"><model name=\"my_box\"><pose>0.3 0.3 0.75 0 0 0</pose><static>false</static><link name=\"link\"><collision name=\"collision\"><geometry><box><size>0.1 0.1 1.5</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.1 0.1 1.5</size></box></geometry></visual></link></model></sdf>'}"
```

```
ros2 service call /spawn_entity gazebo_msgs/SpawnEntity "{name: 'my_box5', xml: '<sdf version=\"1.7\"><model name=\"my_box\"><pose>0.5 0.35 0.75 0 0 0</pose><static>false</static><link name=\"link\"><collision name=\"collision\"><geometry><box><size>0.1 0.1 1.5</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.1 0.1 1.5</size></box></geometry></visual></link></model></sdf>'}"
```

```
ros2 service call /spawn_entity gazebo_msgs/SpawnEntity "{name: 'my_box6', xml: '<sdf version=\"1.7\"><model name=\"my_box\"><pose>0.7 -0.35 0.75 0 0 0</pose><static>false</static><link name=\"link\"><collision name=\"collision\"><geometry><box><size>0.1 0.1 1.5</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.1 0.1 1.5</size></box></geometry></visual></link></model></sdf>'}"
```