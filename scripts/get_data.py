#!/usr/bin/python3

#############################################################################
#   File: get_data.py
#   Function: To get the robot's HighState data from Raspberry Pi via UDP then
#             serialize it and send it to ROS node via socket comms at 10 Hz    
#############################################################################


import sys
import time
import math
import json
import pickle 
import socket

# robot_interface.so can be found in unitree_legged_sdk/lib/python/(arm64/amd64)
# copy and paste the current amd64/arm64 python3 version of the .so file in /scripts directory
# change name to "robot_interface.so". In this code python 3.6 version is being used.

import robot_interface as sdk

if __name__ == '__main__':


    HIGHLEVEL = 0xee
    LOWLEVEL  = 0xff
    
    # Initially the script is not in communication with ROS network
    ros_node_connection_status = False

    # Initialize UDP to communicate with Raspberry Pi
    udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)

    # Initialize HighCmd object
    cmd = sdk.HighCmd()

    # Initialize HighState object
    state = sdk.HighState()


    udp.InitCmdData(cmd)

    # Set HighCmd data to 0 since we are only interested in getting the state data
    # and don't want to actuate the robot in any way.

    cmd.mode = 0      # 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0
    cmd.speedLevel = 0
    cmd.footRaiseHeight = 0
    cmd.bodyHeight = 0
    cmd.euler = [0, 0, 0]
    cmd.velocity = [0, 0]
    cmd.yawSpeed = 0.0
    cmd.reserve = 0


    # Get the robot's data in infinite loop
    while True:
        
        # Timer is used to keep loop at 10Hz
        start_time = time.time()

        # Check if our script is already connected to ROS node
        # If not, then try connecting, else continue
        if ros_node_connection_status == False:
            try:
                    
                host, port = 'localhost', 50007
                s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((host, port))
                ros_node_connection_status = True

                print ("Connection established to ROS node!")

            except:
                pass

        

        # In oder to rcceive state data, we need to send HighCmd object to RaspPi
        udp.SetSend(cmd)
        udp.Send()
        
        # state object contains HighState data
        udp.Recv()
        udp.GetRecv(state)
        
        # we use state_data to convert state object into a dictionary inorder to send data
        # across socket communication
        state_data = {}

        ###################### IMU data ######################

        state_data["IMU"]= {
            'quaternion':state.imu.quaternion,
            'gyroscope':state.imu.gyroscope,
            'accelerometer':state.imu.accelerometer,
            'rpy':state.imu.rpy,
            'temperature': state.imu.temperature
        }

        ###################### Motor data ######################

        motor_dict = {}

        for motor in range(20):
            
            motor_dict[motor]={

                    "mode":state.motorState[motor].mode,
                    "q":state.motorState[motor].q ,
                    "dq":state.motorState[motor].dq,
                    "ddq":state.motorState[motor].ddq ,
                    "tauEst":state.motorState[motor].tauEst ,
                    "q_raw":state.motorState[motor].q_raw ,
                    "dq_raw":state.motorState[motor].dq_raw ,
                    "ddq_raw":state.motorState[motor].ddq_raw ,
                    "temperature":state.motorState[motor].temperature ,
                    "reserve":state.motorState[motor].reserve
            }

        state_data["motors"]=motor_dict
        
        ###################### Header data ######################

        state_data["head"] = state.head 
        state_data["levelFlag"] = state.levelFlag
        state_data["frameReserve"] = state.frameReserve
        state_data["SN"] = state.SN
        state_data["version"] = state.version
        state_data["bandWidth"] = state.bandWidth
        state_data["mode"] = state.mode
        state_data["progress"] = state.progress
        state_data["gaitType"] = state.gaitType
        state_data["reserve"] = state.reserve
        state_data["crc"] = state.crc

        ###################### Foot/Position/Velocity data ######################

        state_data["footForce"]=[state.footForce[0],state.footForce[1],state.footForce[2],state.footForce[3]]
        state_data["footForceEst"]=[state.footForceEst[0],state.footForceEst[1],state.footForceEst[2],state.footForceEst[3]]
        state_data["footRaiseHeight"] = state.footRaiseHeight
        state_data["bodyHeight"]= state.bodyHeight
        state_data["position"]= [state.position[0], state.position[1], state.position[2]]
        state_data["velocity"]= [state.velocity[0], state.velocity[1], state.velocity[2]]
        state_data["yawSpeed"]= state.yawSpeed
        state_data["rangeObstacle"]= [state.rangeObstacle[0], state.rangeObstacle[1], state.rangeObstacle[2], state.rangeObstacle[3]] # Dummy data
        
        state_data["footPosition2Body"] = [[state.footPosition2Body[0].x, state.footPosition2Body[0].y, state.footPosition2Body[0].z], 
        [state.footPosition2Body[1].x, state.footPosition2Body[1].y, state.footPosition2Body[1].z],
        [state.footPosition2Body[2].x, state.footPosition2Body[2].y, state.footPosition2Body[2].z] ,
        [state.footPosition2Body[3].x, state.footPosition2Body[3].y, state.footPosition2Body[3].z]]

        
        state_data["footSpeed2Body"] = [[state.footSpeed2Body[0].x, state.footSpeed2Body[0].y, state.footSpeed2Body[0].z ], 
        [state.footSpeed2Body[1].x, state.footSpeed2Body[1].y, state.footSpeed2Body[1].z ], 
        [state.footSpeed2Body[2].x, state.footSpeed2Body[2].y, state.footSpeed2Body[2].z ], 
        [state.footSpeed2Body[3].x, state.footSpeed2Body[3].y, state.footSpeed2Body[3].z ]]

        ###################### Remote and BMS data ######################

        state_data["wirelessRemote"] = state.wirelessRemote
        
        state_data["bms"] = {
            "version_h": state.bms.version_h,
            "version_l": state.bms.version_l,
            "status": state.bms.bms_status,
            "SOC": state.bms.SOC,
            "current": state.bms.current,
            "cycle": state.bms.cycle,
            "BQ_NTC": state.bms.BQ_NTC,
            "MCU_NTC": state.bms.MCU_NTC,
            "cell_vol": state.bms.cell_vol
        }
        

        # Convert dictionary to JSON with a "\n" in the end to mark the end of data

        json_data = json.dumps(state_data) + "\n"
        json_data = json_data.encode()


        # Try sending data to ROS node
        try:

            s.sendall(json_data)

        except:
            ros_node_connection_status = False


        # calculate sleep time inorder to keep transmission rate of 10 Hz

        elapsed_time = time.time() -start_time

        # Note: 0.09965 is used instead of 0.1 inorder to account for transmission delays
        time_to_sleep =   0.09965 - elapsed_time 

        if time_to_sleep>=0:
            time.sleep(time_to_sleep)
            