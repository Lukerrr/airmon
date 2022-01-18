#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import BatteryState
from airmon_comm.msg import DroneState

######################################################################################
## A class to handle quadrotor's state values
## and publish a state to the ground station.
##
## Fields list:
##  __movCtrl - CMovementController instance ref to receive a local position data
##  __gps - CGpsSystem instance ref to receive a global position data
##  __mission - CMission instance ref to receive a mission data
##  __batCharge - current battery charge in range [0.0; 1.0]
##  __chargeSub - battery state topic subscriber
##  __droneStatePub - drone state topic publisher
##
## Methods list:
##  SendData - collects all necessary state values and send to the ground station
##  __onBatteryStateChanged - battery state topic callback function
######################################################################################
class CDroneListener:
    def __init__(self, movCtrl, gps, mission):
        self.__movCtrl = movCtrl
        self.__gps = gps
        self.__mission = mission
        self.__batCharge = 0.0
        self.__chargeSub = rospy.Subscriber("mavros/battery", BatteryState, self.__onBatteryStateChanged)
        self.__droneStatePub = rospy.Publisher("airmon_comm/out/drone_state", DroneState, queue_size = 1)
    
    ## 
    # Collect all necessary state values and send to the ground station
    #   systemState - mvoement system state (see ESystemState)
    def SendData(self, systemState):
        state = DroneState()
        state.systemState = systemState
        state.missionHash = self.__mission.hash
        state.height = self.__mission.GetHeight()
        state.tolerance = self.__mission.GetTolerance()
        if(self.__movCtrl.simState.connected):
            state.lat = self.__gps.lat
            state.lon = self.__gps.lon
            state.heading = self.__gps.heading
            state.globalPos = self.__gps.isValid
            state.pos.x = self.__movCtrl.pos.x
            state.pos.y = self.__movCtrl.pos.y
            state.pos.z = self.__movCtrl.pos.z
            state.angles.x = self.__movCtrl.rot.roll
            state.angles.y = self.__movCtrl.rot.pitch
            state.angles.z = self.__movCtrl.rot.yaw
            state.charge = self.__batCharge
            state.armed = self.__movCtrl.simState.armed
        self.__droneStatePub.publish(state)
    
    ## Battery state topic callback function
    def __onBatteryStateChanged(self, bat):
        if(bat.present):
            self.__batCharge = bat.percentage
        else:
            # No battery
            self.__batCharge = 0.0