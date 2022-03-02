#!/usr/bin/env python3

import rospy

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from clover import srv

from math_utils import Vec3, Rotator

import numpy as np


################################################################################
## A class to manage quadrotor's movement.
## Supports velocity and position control.
##
## Fields list:
###
##  simState - current quadrotor state
##  pos - quadrotor location (Vec3 structure, see math_utils)
##  rot - quadrotor rotation (Rotator structure, see math_utils)
##  fwdVec - quadrotor forward vector (Vec3 structure, see math_utils)
##  rgtVec - quadrotor right vector (Vec3 structure, see math_utils)
##  upVec - quadrotor up vector (Vec3 structure, see math_utils)
###
##  __stateSub - state topic subscruber
##  __poseSub - pose topic subscruber
##  __heightSub - ground distanceS topic subscruber
##  __localCtrlPub - local position target topic publisher
##  __armingClient - service client to arm quadrotor
##  __setModeClient - service client to set control mode
##
##
## Methods list:
##  __onStateChanged - state topic callback function
##  SetMode - sets quadrotor control mode
##  SetIsArmed - arms/disarms quadrotor
##  SetPos - sets quadrotor target position
################################################################################
class CMovementController:

    def __init__(self):
        self.simState = State()

        self.__getTelemetrySrv = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.__navigateSrv = rospy.ServiceProxy('navigate', srv.Navigate)
        self.__navigateGlSrv = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.__landSrv = rospy.ServiceProxy('land', Trigger)

        self.__stateSub = rospy.Subscriber("mavros/state", State, self.__onStateChanged)
        self.__armingClient = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.__setModeClient = rospy.ServiceProxy("mavros/set_mode", SetMode)

    ## State topic callback function
    def __onStateChanged(self, newState):
        self.simState.mode = newState.mode
        self.simState.armed = newState.armed
        self.simState.connected = newState.connected

    ## Gets actual telemetry data
    def Update(self):
        pass

    ## Set quadrotor control mode
    def SetMode(self, mode):
        try:
            self.__setModeClient(custom_mode = mode)
        except rospy.ServiceException as e:
            rospy.logerr("SetMode error: %s", e.message)

    ## Arm/disarm quadrotor
    def SetIsArmed(self, isArmed):
        try:
            self.__armingClient(isArmed)
        except rospy.ServiceException as e:
            rospy.logerr("SetIsArmed error: %s", e.message)

    ## Set quadrotor target position
    def SetPos(self, x, y, z, arm = False):
        self.__navigateSrv(x=x, y=y, z=z, yaw=0.0, speed=1.0, auto_arm=arm, frame_id='map')

    ## Set quadrotor target position (global coordinates)
    def SetPosGl(self, lat, lon, z, arm = False):
        self.__navigateGlSrv(lat=lat, lon=lon, z=z, yaw=0.0, speed=1.0, auto_arm=arm, frame_id='map')

    ## Returns a distance vector to the navigate point
    def GetNavigateError(self) -> Vec3:
        telem = self.__getTelemetrySrv(frame_id='navigate_target')
        return Vec3(telem.x, telem.y, telem.z)

    ## Checks is a drone currently at a given point
    def IsAtPos(self, pos: Vec3, eps: float = 0.2) -> bool:
        dist: float = (self.pos - pos).GetLength()
        return dist <= eps

    ## Begin landing
    def Land(self):
        self.__landSrv()

    @property
    def pos(self):
        telem = self.__getTelemetrySrv()
        return Vec3(telem.x, telem.y, telem.z)

    @property
    def rot(self):
        telem = self.__getTelemetrySrv()
        return Rotator(telem.roll, telem.pitch, telem.yaw)

    @property
    def fwdVec(self):
        return Vec3(1., 0., 0.).Rotate(self.rot)

    @property
    def rgtVec(self):
        return Vec3(0., 1., 0.).Rotate(self.rot)

    @property
    def upVec(self):
        return Vec3(0., 0., 1.).Rotate(self.rot)
