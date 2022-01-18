#!/usr/bin/env python3

import rospy

from enum import Enum

from helpers.math_utils import *
from helpers.ros_globals import *

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import ParamSet
from mavros_msgs.srv import CommandLong

################################################################################
## PositionTarget dispatch flags.
################################################################################
class EPFlags(Enum):
    # Set position
    SET_POS = 1 | 2 | 4
    # Set velocity
    SET_VEL = 8 | 16 | 32
    # Set force
    SET_FORCE = 64 | 128 | 256
    # Set acceleration
    SET_ACCEL = 64 | 128 | 256 | 512
    # Set yaw
    SET_YAW = 1024
    # Set yaw rate (angular velocity)
    SET_YAW_RATE = 2048

################################################################################
## A class to manage quadrotor's movement.
## Supports velocity and position control.
##
## Fields list:
###
##  simState - current quadrotor state
##  pose - position data to dispatch
##  pos - quadrotor local location in meters (Vec3 structure, see math_utils)
##  rot - quadrotor rotation in radians (Rotator structure, see math_utils)
##  fwdVec - quadrotor forward vector (Vec3 structure, see math_utils)
##  rgtVec - quadrotor right vector (Vec3 structure, see math_utils)
##  upVec - quadrotor up vector (Vec3 structure, see math_utils)
###
##  __stateSub - state topic subscruber
##  __poseSub - pose topic subscruber
##  __heightSub - ground distanceS topic subscruber
##  __localCtrlPub - local position target topic publisher
##  __setParamClient - service client to set autopilot parameters
##  __setModeClient - service client to set control mode
##  __cmdClient - service client to send a MAVLink command
##
##
## Methods list:
##  __onStateChanged - state topic callback function
##  __onPoseChanged - pose topic callback function
##  SetParam - sets autopilot parameter by ID
##  SetMode - sets quadrotor control mode
##  SetIsArmed - arms/disarms quadrotor
##  SetPos - sets quadrotor target position
##  SetVel - sets quadrotor target velocity
##  SetYaw - sets quadrotor target yaw (in radians)
##  SetYawRate - sets quadrotor target yaw rate (in rad/s)
##  DispatchPosition - sends PositionTarget info to quadrotor
################################################################################
class CMovementController:

    def __init__(self):
        self.simState = State()
        self.pose = PositionTarget(type_mask = 0x0FFF)

        self.pos = Vec3(0., 0., 0.)
        self.rot = Rotator(0., 0., 0.)
        self.fwdVec = Vec3(1., 0., 0.)
        self.rgtVec = Vec3(0., 1., 0.)
        self.upVec = Vec3(0., 0., 1.)

        self.__stateSub = rospy.Subscriber("mavros/state", State, self.__onStateChanged)
        self.__poseSub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.__onPoseChanged)
        self.__localCtrlPub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size = 10)
        self.__setParamClient = rospy.ServiceProxy("mavros/param/set", ParamSet)
        self.__setModeClient = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.__cmdClient = rospy.ServiceProxy("mavros/cmd/command", CommandLong)

    ## State topic callback function
    def __onStateChanged(self, newState):
        self.simState.mode = newState.mode
        self.simState.armed = newState.armed
        self.simState.connected = newState.connected

    ## Pose topic callback function
    def __onPoseChanged(self, pose):
        self.pos = Vec3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        quat = Quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        self.rot = quat.GetRotator()

        # Calculating orientation vectors
        self.fwdVec = Vec3(1., 0., 0.).Rotate(self.rot)
        self.rgtVec = Vec3(0., 1., 0.).Rotate(self.rot)
        self.upVec = Vec3(0., 0., 1.).Rotate(self.rot)

    ## Set autopilot parameter by ID
    def SetParam(self, id, valInt, valFloat):
        try:
            val = ParamValue()
            val.integer = valInt
            val.real = valFloat
            self.__setParamClient(param_id = id, value = val)
        except rospy.ServiceException as e:
            rospy.logerr("CMovementController: SetParam error: %s", str(e))

    ## Set quadrotor control mode
    def SetMode(self, mode):
        rospy.loginfo("CMovementController: SetMode '%s'", mode)
        try:
            self.__setModeClient(custom_mode = mode)
        except rospy.ServiceException as e:
            rospy.logerr("CMovementController: SetMode error: %s", str(e))

    ## Arm/disarm quadrotor
    def SetIsArmed(self, isArmed):
        try:
            if(isArmed):
                paramIsArmed = 1.0
            else:
                paramIsArmed = 0.0
            self.__cmdClient(broadcast=False,\
                command=400,\
                confirmation=0,\
                param1=paramIsArmed,\
                param2=21196.0,\
                param3=0.0,\
                param4=0.0,\
                param5=0.0,\
                param6=0.0,\
                param7=0.0)
        except rospy.ServiceException as e:
            rospy.logerr("CMovementController: SetIsArmed error: %s", str(e))

    ## Set quadrotor target position
    def SetPos(self, x, y, z):
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z
        self.pose.type_mask = self.pose.type_mask & (~EPFlags.SET_POS.value)

    ## Set quadrotor target velocity
    def SetVel(self, x, y, z, add = False):
        if(add == True):
            self.pose.velocity.x += x
            self.pose.velocity.y += y
            self.pose.velocity.z += z
        else:
            self.pose.velocity.x = x
            self.pose.velocity.y = y
            self.pose.velocity.z = z
        self.pose.type_mask = self.pose.type_mask & (~EPFlags.SET_VEL.value)

    ## Set quadrotor target yaw (in radians)
    def SetYaw(self, yaw):
        self.pose.yaw = yaw
        self.pose.type_mask = self.pose.type_mask & (~EPFlags.SET_YAW.value)

    ## Set quadrotor target yaw rate (in rad/s)
    def SetYawRate(self, rate, add = False):
        if(add == True):
            self.pose.yaw_rate += rate
        else:
            self.pose.yaw_rate = rate
        self.pose.type_mask = self.pose.type_mask & (~EPFlags.SET_YAW_RATE.value)

    ## Send PositionTarget info to quadrotor
    def DispatchPosition(self):
        if(self.pose.type_mask == 0x0FFF):
            # Hold position by default
            self.SetVel(0, 0, 0)
        self.pose.header.stamp = rospy.get_rostime()
        self.pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.__localCtrlPub.publish(self.pose)
        self.pose = PositionTarget(type_mask = 0x0FFF)
