#!/usr/bin/env python3

import rospy

from helpers.ros_globals import *
from framework.gps import CGpsSystem
from framework.mission import CMission
from framework.mov_ctrl import CMovementController
from framework.drone_listener import CDroneListener
from framework.system_state import ESystemState
from framework.air_sens import CAirSens

from airmon_comm.msg import GsCmdSimple

################################################################################
## The application main class that controlls other subsystems.
##
## Fields list:
###
##  __movCtrl - the movement control subsystem
##  __gps - global position subsystem
##  __mission - the mission management subsystem
##  __droneLst - the drone state listening subsystem
###
##  __rate - working loop execution rate in hertz
##  __systemState - current state of the system
##  __lastState - last known state of the system before disconnect
##  __takeoffPos - a position for takeoff
##  __landingPos - a position for landing
###
##  __armSub - ARM command topic subscriber
##  __disarmSub - DISARM command topic subscriber
##  __startSub - START command topic subscriber
##  __stopSub - STOP command topic subscriber
##
## Methods list:
###
##  Run - the application working function
##  OnExit - ROS node interrupt exception callback
##  __setState - performs the system state transition
###
##  __onCmdArm - cmd ARM callback
##  __onCmdDisarm - cmd DISARM callback
##  __onCmdStart - cmd START callback
##  __onCmdStop - cmd STOP callback
##  __getAirSensBeginSub - cmd GET_AIR_SENS callback
################################################################################
class CSystem:
    def __init__(self):
        rospy.init_node("airmon", anonymous = True)
        rate = rospy.get_param("~rate", default = 50)
        self.__movCtrl = CMovementController()
        self.__gps = CGpsSystem()
        self.__airSens = CAirSens()
        self.__mission = CMission(self.__movCtrl, self.__gps)
        self.__droneLst = CDroneListener(self.__movCtrl, self.__gps, self.__mission, self.__airSens)
        self.__rate = rospy.Rate(rate)
        self.__systemState = ESystemState.DISCONNECTED
        self.__lastState = ESystemState.IDLE
        self.__takeoffPos = None
        self.__landingPos = None
        self.__armSub = rospy.Subscriber("airmon_comm/in/cmd_arm", GsCmdSimple, self.__onCmdArm)
        self.__disarmSub = rospy.Subscriber("airmon_comm/in/cmd_disarm", GsCmdSimple, self.__onCmdDisarm)
        self.__startSub = rospy.Subscriber("airmon_comm/in/cmd_start", GsCmdSimple, self.__onCmdStart)
        self.__stopSub = rospy.Subscriber("airmon_comm/in/cmd_stop", GsCmdSimple, self.__onCmdStop)
        self.__getAirSensBeginSub = rospy.Subscriber("airmon_comm/in/cmd_get_air_sens", GsCmdSimple, self.__onCmdGetAirSens)

    ## The application working function
    def Run(self):
        rospy.loginfo("CSystem: waiting for a timer...")
        # Wait for a timer valid response
        while(True):
            self.__time = now().to_sec()
            self.__rate.sleep()
            if(self.__time > 0.0):
                break

        rospy.loginfo("CSystem: begin working loop")

        while(ok()):
            try:
                self.__workingLoop()
                self.__rate.sleep()
            except Exception as e:
                rospy.logerr(f"Unhandled exception {str(e)}")

    def __workingLoop(self):
        # Get delta time
        curTime = now().to_sec()
        dt = curTime - self.__time
        self.__time = curTime

        # Detect auto-disarm
        if(not self.__movCtrl.simState.armed and self.__systemState.value > ESystemState.IDLE.value):
            rospy.logwarn("CSystem: disarmed by autopilot!")
            self.__setState(ESystemState.IDLE)

        isWorking = self.__systemState.value > ESystemState.IDLE.value

        # Validate mission if working
        if(isWorking and not self.__mission.IsValid()):
            rospy.logwarn("CSystem: mission is invalid, landing...")
            self.__setState(ESystemState.LANDING)

        # Check is connected
        if(self.__systemState != ESystemState.DISCONNECTED):
            if(self.__movCtrl.simState.connected == False):
                rospy.logwarn("CSystem: disconnected from FMU!")
                self.__lastState = self.__systemState
                if(self.__lastState != ESystemState.IDLE):
                    rospy.loginfo("CSystem: reserved state '%s'", self.__lastState.name)
                self.__setState(ESystemState.DISCONNECTED)

        if(dt > 0.0):
            # State DISCONNECTED
            if(self.__systemState == ESystemState.DISCONNECTED):
                if(self.__movCtrl.simState.connected == True):
                    self.__setState(self.__lastState)
                    if(self.__systemState != ESystemState.IDLE):
                        rospy.loginfo("CSystem: restored state '%s'", self.__systemState.name)

            # State IDLE
            elif(self.__systemState == ESystemState.IDLE):
                pass

            # State TAKEOFF
            elif(self.__systemState == ESystemState.TAKEOFF):
                if(self.__takeoffPos == None):
                    self.__takeoffPos = [self.__movCtrl.pos.x, self.__movCtrl.pos.y]
                    self.__movCtrl.SetPos(self.__takeoffPos[0], self.__takeoffPos[1], self.__mission.GetHeight(), arm=True)
                    #rospy.loginfo("CSystem: takeoff at %f, %f, %f", self.__takeoffPos[0], self.__takeoffPos[1], self.__mission.GetHeight())
                if(self.__movCtrl.pos.z >= self.__mission.GetHeight() * 0.9):
                    rospy.loginfo("CSystem: takeoff was done at height %f", self.__movCtrl.pos.z)
                    self.__setState(ESystemState.WORKING)
                    self.__takeoffPos = None

            # State WORKING
            elif(self.__systemState == ESystemState.WORKING):
                if(self.__mission.Update()):
                    rospy.loginfo("CSystem: mission is done, landing...")
                    self.__setState(ESystemState.LANDING)

            # State LANDING
            elif(self.__systemState == ESystemState.LANDING):
                if(self.__landingPos == None):
                    self.__landingPos = [self.__movCtrl.pos.x, self.__movCtrl.pos.y]
                    rospy.loginfo("CSystem: requesting land")
                    self.__movCtrl.Land()
                if(not self.__movCtrl.simState.armed):
                    rospy.loginfo("CSystem: landing was done!")
                    self.__setState(ESystemState.IDLE)
                    self.__landingPos = None
        else:
            rospy.logwarn("CSystem: delta time was invalid (%f) at %f sec.", dt, curTime)

        self.__droneLst.SendData(self.__systemState.value)

    ## ROS node interrupt exception callback
    def OnExit(self):
        self.__movCtrl.Land()

    ## Perform the system state transition
    def __setState(self, st):
        if(self.__systemState != st):
            rospy.loginfo("CSystem::setState: %s -> %s", self.__systemState.name, st.name)
            self.__systemState = st
        else:
            rospy.loginfo("CSystem::setState: '%s' state loop ignored", st.name)

    ## Cmd ARM callback
    def __onCmdArm(self, cmd):
        rospy.loginfo("CSystem: command 'ARM' received")
        if(not self.__movCtrl.simState.armed):
            self.__movCtrl.SetIsArmed(True)
        else:
            rospy.loginfo("CSystem: already armed")

    ## Cmd DISARM callback
    def __onCmdDisarm(self, cmd):
        rospy.loginfo("CSystem: command 'DISARM' received")
        if(self.__systemState.value > ESystemState.IDLE.value):
            rospy.loginfo("CSystem: cannot disarm if working. Starting land...")
            self.__setState(ESystemState.LANDING)
        else:
            if(self.__movCtrl.simState.armed):
                self.__movCtrl.SetIsArmed(False)
            else:
                rospy.loginfo("CSystem: already disarmed")
    
    ## Cmd START callback
    def __onCmdStart(self, cmd):
        rospy.loginfo("CSystem: command 'START' received")
        if(self.__systemState != ESystemState.IDLE):
            rospy.loginfo("CSystem: cannot work from state '%s'", self.__systemState.name)
        elif(not self.__movCtrl.simState.armed):
            rospy.loginfo("CSystem: cannot work - disarmed")
        elif(not self.__mission.IsValid()):
            rospy.loginfo("CSystem: cannot work - invalid mission")
        elif(not self.__mission.Reset()):
            rospy.loginfo("CSystem: cannot work - mission reset failed")
        else:
            self.__airSens.ResetData()
            self.__setState(ESystemState.TAKEOFF)

    ## Cmd STOP callback
    def __onCmdStop(self, cmd):
        rospy.loginfo("CSystem: command 'STOP' received")
        if(self.__systemState.value > ESystemState.IDLE.value):
            if(self.__movCtrl.simState.armed):
                self.__setState(ESystemState.LANDING)
            else:
                rospy.loginfo("CSystem: cannot stop if disarmed")
        else:
            rospy.loginfo("CSystem: invalid working state '%s'", self.__systemState.name)

    ## Cmd GET_AIR_SENS callback
    def __onCmdGetAirSens(self, cmd):
        rospy.loginfo("CSystem: command 'GET_AIR_SENS' received")
        self.__airSens.PublishSensData()
