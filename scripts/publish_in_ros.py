#!/usr/bin/env python

#############################################################################
#   File: publish_in_ros.py
#   Function: To publish Unitree's HighState msg onto the ROS network at 10HZ  
#   Functions in file:
#           write_highstate_msg
#           get_socket_data
#           publish_robot_state
#############################################################################

import rospy
from unitree_legged_msgs.msg import HighState, IMU, MotorState, BmsState, Cartesian
from std_msgs.msg import Header
import time
import socket
import json


#############################################################################
#   Name: write_highstate_msg
#   Input: unitree_data: deserialized unitree state data from socket
#   Returns: HighState message
#   Function: To write unitree state data from dictionary onto a HighState object
#############################################################################

def write_highstate_msg(unitree_data):

    state_msg = HighState()

    state_msg.head = unitree_data["head"]
    state_msg.levelFlag = unitree_data["levelFlag"]
    state_msg.frameReserve =  unitree_data["frameReserve"]
    state_msg.SN =  unitree_data["SN"]
    state_msg.version =  unitree_data["version"]
    state_msg.bandWidth =  unitree_data["bandWidth"]
    state_msg.footForce =  unitree_data["footForce"]
    state_msg.footForceEst =  unitree_data["footForceEst"]
    state_msg.mode =  unitree_data["mode"]
    state_msg.progress =  unitree_data["progress"]
    state_msg.gaitType =  unitree_data["gaitType"]
    state_msg.footRaiseHeight =  unitree_data["footRaiseHeight"]
    state_msg.position =  unitree_data["position"]
    state_msg.bodyHeight =  unitree_data["bodyHeight"]
    state_msg.velocity =  unitree_data["velocity"]
    state_msg.yawSpeed =  unitree_data["yawSpeed"]
    state_msg.rangeObstacle =  unitree_data["rangeObstacle"]
    state_msg.wirelessRemote =  unitree_data["wirelessRemote"]
    state_msg.reserve =  unitree_data["reserve"]
    state_msg.crc =  unitree_data["crc"]
    
    imu = IMU()    
    imu.quaternion = unitree_data["IMU"]["quaternion"]
    imu.gyroscope = unitree_data["IMU"]["gyroscope"]
    imu.accelerometer = unitree_data["IMU"]["accelerometer"]
    imu.rpy = unitree_data["IMU"]["rpy"]
    imu.temperature = unitree_data["IMU"]["temperature"]
    state_msg.imu =  imu

 
    motorState = []
    for motor_id in range (20):
        motor = MotorState()
        motor.mode = unitree_data["motors"][str(motor_id)]["mode"]
        motor.q = unitree_data["motors"][str(motor_id)]["q"]
        motor.dq = unitree_data["motors"][str(motor_id)]["dq"]
        motor.ddq = unitree_data["motors"][str(motor_id)]["ddq"]
        motor.tauEst = unitree_data["motors"][str(motor_id)]["tauEst"]
        motor.q_raw = unitree_data["motors"][str(motor_id)]["q_raw"]
        motor.dq_raw = unitree_data["motors"][str(motor_id)]["dq_raw"]
        motor.ddq_raw = unitree_data["motors"][str(motor_id)]["ddq_raw"]
        motor.temperature = unitree_data["motors"][str(motor_id)]["temperature"]
        motor.reserve = unitree_data["motors"][str(motor_id)]["reserve"]
        motorState.append (motor)

    state_msg.motorState =  motorState

    bms = BmsState()

    bms.version_h = unitree_data["bms"]["version_h"]
    bms.version_l = unitree_data["bms"]["version_l"]
    bms.bms_status = unitree_data["bms"]["status"]
    bms.SOC = unitree_data["bms"]["SOC"]
    bms.current = unitree_data["bms"]["current"]
    bms.cycle = unitree_data["bms"]["cycle"]
    bms.BQ_NTC = unitree_data["bms"]["BQ_NTC"]
    bms.MCU_NTC = unitree_data["bms"]["MCU_NTC"]
    bms.cell_vol = unitree_data["bms"]["cell_vol"]

    state_msg.bms = bms 
    
    foot_positions=[]
    foot_speeds = []
    for leg in range(4):
        foot_position = Cartesian()
        foot_speed = Cartesian()

        foot_position.x = unitree_data["footPosition2Body"][leg][0]
        foot_position.y = unitree_data["footPosition2Body"][leg][1]
        foot_position.z = unitree_data["footPosition2Body"][leg][2]

        foot_speed.x = unitree_data["footSpeed2Body"][leg][0]
        foot_speed.y = unitree_data["footSpeed2Body"][leg][1]
        foot_speed.z = unitree_data["footSpeed2Body"][leg][2]

        foot_positions.append(foot_position)
        foot_speeds.append(foot_speed)

    state_msg.footPosition2Body =  foot_positions
    state_msg.footSpeed2Body =  foot_speeds

    return state_msg
    


#############################################################################
#   Name: get_socket_data
#   Input: conn: socket connection object
#   Returns: state_data: data received from socket
#   Function: To receive robot's state data via socket
#############################################################################

def get_socket_data(conn):

    buffer=""
    state_data = None

    # The complete data is sent in 6 chunks which is then appended into a buffer
    for i in range (0,6):
        data = conn.recv(1024).decode()
        buffer += data

    # Process complete JSON objects
    while '\n' in buffer:
        json_str, buffer = buffer.split('\n',1)

        try:

            # convert from json to dictionary
            state_data = json.loads(json_str)
            return state_data
            
        except:

            pass



#############################################################################
#   Name: publish_robot_state
#   Input: None
#   Returns: None
#   Function: To publish HishState msg in unitree/robot_state topic at 10 Hz
#############################################################################

def publish_robot_state():

    # Initialize the ROS node
    rospy.init_node('unitree_state_publisher', anonymous=True)

    # Create a publisher for the HighState message
    pub = rospy.Publisher('/unitree/robot_state', HighState, queue_size=10)

    # Connect with the unitree data client
    host, port = 'localhost', 50007
    socket_comms = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    socket_comms.bind ((host,port))
    socket_comms.listen(1)
    conn, addr = socket_comms.accept()


    # Main loop to keep publishing data
    while not rospy.is_shutdown():

        try:

            # Get data from socket
            unitree_data = get_socket_data(conn)
            
            # Write data onto message
            robot_state_msg = write_highstate_msg(unitree_data)

            # Publish the message
            pub.publish(robot_state_msg)


        except:
            rospy.loginfo ("Unitree HighState message not publishing. Restart the node!")
            


if __name__ == '__main__':
    try:
        publish_robot_state()
    except rospy.ROSInterruptException:
        pass


