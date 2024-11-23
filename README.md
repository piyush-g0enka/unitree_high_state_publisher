### Unitree Go1 HighState publisher

This is a ROS1 Melodic package for publishing HighState messages at 10 Hz in Unitree's ROS network. 
This package can be installed in any of the three Jetson nano computers in your Go1 robot.

#### Dependencies:

1. unitree_legged_msgs
2. rospy

#### Build the package

First, clone/download the repo into your unitree's desired nano computer. Make sure unitree_legged_msgs package is present in your ROS environment.

Depending on if you have python3.6 or python3.8 you would need to change the robot_interface.so file in the scripts folder.

If your jetson nano has python 3.6 installed then you do not need to make any change.

Else if your jetson nano has python 3.8 installed then navigate to unitree_legged_sdk/libs/python/arm64. You would find a file similar to "robot_interface.cpython-38-aarch64-linux-gnu.so".
Copy that file and replace the robot_interface.so in unitree_high_state_publisher/scripts with this file and change its name to robot_interface.so.
Build the package using catkin_make.

#### Run

```bash
$ roslaunch unitree_high_state_publisher unitree_high_state_publisher.launch
```

You should see HighState messages being published to topic "/unitree/robot_state" with a frequency of 10 Hz.

#### Scripts

Since the unitree python sdk supports python3 and ROS melodic supports python 2.7, two scripts were created.

1. get_data.py is a python3 script which uses unitree sdk to get the robot's state data.
2. publish_to_ros.py is a python2.7 script which publishes state data onto the ROS network.

In between, the two scripts use socket communication to pass on the state data from python3 to python 2.7.

#### HighState message structure

uint8[2] head\
uint8 levelFlag\
uint8 frameReserve\
\
uint32[2] SN\
uint32[2] version\
uint16 bandWidth\
IMU imu\
MotorState[20] motorState\
BmsState bms\
int16[4] footForce\
int16[4] footForceEst\
uint8 mode\
float32 progress\
uint8 gaitType\  
float32 footRaiseHeight\		  
float32[3] position\ 
float32 bodyHeight\		  
float32[3] velocity\ 
float32 yawSpeed\			   
float32[4] rangeObstacle\
Cartesian[4] footPosition2Body\
Cartesian[4] footSpeed2Body\
\
uint8[40] wirelessRemote\
uint32 reserve\
\
uint32 crc\

Note that motorState consists of state of 20 motors out of which first 12 represent actual motor data and remaining 8 represent dummy data. \
\
The motor data order is as follows:\
Front-Right 0, 1, 2 --> Front-Left 0, 1, 2 -->  Rear-Right 0, 1, 2 --> Rear-Left 0, 1, 2 


Foot data order is as follows:
Front-Right --> Front-Left -->  Rear-Right --> Rear-Left
