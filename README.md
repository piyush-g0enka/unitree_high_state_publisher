### Unitree Go1 HighState publisher

This is a ROS1 Melodic package for publishing HighState messages at 10 Hz in Unitree's ROS network. 
This package can be installed in any of the three Jetson nano.

#### Dependencies:

1. unitree_legged_msgs
2. rospy

#### Build the package

First, clone/download the repo into your unitree's desired nano computer. Make sure unitree_legged_msgs package is present in your ROS environment.

Depending on if you have python3.6 or python3.8 you would need to change the robot_interface.so file in the scripts folder.

If python 3.6 then you do not need to make nay change.
If python 3.8 then navigate to unitree_legged_sdk/libs/python/arm64. You would find a file similar to "robot_interface.cpython-38-aarch64-linux-gnu.so".
Copy that file and replace the robot_interface.so in unitree_high_state_publisher/scripts with this file and change its name to robot_interface.so.
Build the package using catkin_make.

#### Run

```bash
$ roslaunch unitree_high_state_publisher unitree_high_state_publisher.launch
```

You should see a topic "/unitree/robot_state" with a frequency of 10 Hz.