#!/usr/bin/env python3

import rospy

from airmon_comm.msg import GsCmdMission
from airmon_comm.msg import GsCmdFloat

from helpers.math_utils import *

import numpy as np

################################################################################
## A class to store the mission and execute quadrotor's movement
## along the given path in geographical coordinates.
##
## Fields list:
###
##  hash - current mission crc32 hash number for validation
##  __targetHeight - flight height in meters
##  __tolerance - distance to reach path points in meters
##  __path - mission geographical points array
##  __curIdx - target point index
##  __gps - CGpsSystem instance ref to receive a global position
##  __movCtrl - CMovementController instance ref to drive the quadrotor
###
##  __missionSub - mission path topic subscriber
##  __heightSub - target height topic subscriber
##  __toleranceSub - tolerance topic subscriber
##
## Methods list:
##  IsValid - returns true if the path and the mission parameters are valid
##  Reset - resets mission to start
##  Update - updates the mission execution
##  GetHeight - returns current target height value
##  GetTolerance - returns current flight tolerance value
##  __incrementTarget - increment target point index
##  __onMissionChanged - mission path topic callback
##  __onHeightChanged - target height topic callback
##  __onToleranceChanged - tolerance topic callback
################################################################################
class CMission:
    def __init__(self, movCtrl, gps):
        self.hash = 0xFFFFFFFF

        self.__gps = gps
        self.__movCtrl = movCtrl

        self.__pathGlobal = []
        self.__path = []
        self.__curIdx = 0
        self.__movementStarted = False

        self.__targetHeight = 1.5
        self.__tolerance = 1.0

        self.__missionSub = rospy.Subscriber("airmon_comm/in/cmd_mission", GsCmdMission, self.__onMissionChanged)
        self.__heightSub = rospy.Subscriber("airmon_comm/in/cmd_height", GsCmdFloat, self.__onHeightChanged)
        self.__toleranceSub = rospy.Subscriber("airmon_comm/in/cmd_tolerance", GsCmdFloat, self.__onToleranceChanged)

    # Returns true if the path and the mission parameters are valid
    def IsValid(self):
        hashValid = self.hash != 0xFFFFFFFF
        pathValid = len(self.__pathGlobal) >= 2
        heightValid = self.__targetHeight >= 0.5
        toleranceValid = self.__tolerance > 0.0
        
        if(not hashValid):
            rospy.logwarn("CMission::IsValid: invalid mission hash")

        if(not pathValid):
            rospy.logwarn("CMission::IsValid: invalid path ('%s')", str(self.__pathGlobal))
        
        if(not heightValid):
            rospy.logwarn("CMission::IsValid: invalid target height (%f)", self.__targetHeight)
        
        if(not toleranceValid):
            rospy.logwarn("CMission::IsValid: invalid tolerance (%f)", self.__tolerance)

        return hashValid and pathValid and heightValid and toleranceValid

    # Reset mission to start
    def Reset(self):
        rospy.loginfo("CMission: requested reset")

        if(not self.__gps.isValid):
            rospy.loginfo("CMission: failed to reset - no GPS data")
            return False

        if(len(self.__pathGlobal) < 2):
            rospy.loginfo("CMission: failed to reset - invalid path")
            return False

        rospy.loginfo("CMission: rebuilding path...")

        # Set current position as the return point
        self.__pathGlobal.append([self.__gps.lat, self.__gps.lon])

        # Reset target point
        self.__curIdx = -1
        self.__incrementTarget()

        rospy.loginfo("CMission: reset is done!")
        return True

    ## Update the mission execution
    def Update(self):
        if(self.__curIdx >= len(self.__pathGlobal)):
            # Last point is reached
            return True

        curPos = [self.__gps.lat, self.__gps.lon]
        trgPt = self.__pathGlobal[self.__curIdx]
        prevPt = self.__pathGlobal[self.__curIdx - 1]

        if not self.__movementStarted:
            # Set target position
            self.__movCtrl.SetPosGl(trgPt[0], trgPt[1], self.__targetHeight)
            self.__movementStarted = True

        # Set target yaw
        """dx = trgPt[0] - prevPt[0]
        dy = trgPt[1] - prevPt[1]
        targetYaw = np.arctan2(dy, dx)
        self.__movCtrl.SetYaw(targetYaw)"""
        
        """xError = abs(curPos[0] - trgPt[0])
        yError = abs(curPos[1] - trgPt[1])"""

        err = self.__movCtrl.GetNavigateError()
        print(err.x, err.y, err.z)
        
        if(err.x <= self.__tolerance and err.y <= self.__tolerance):
            self.__incrementTarget()
            rospy.loginfo("CMission: passed waypoint #%d of #%d", self.__curIdx, len(self.__path))

        return False
    
    ## Returns current target height value
    def GetHeight(self):
        return self.__targetHeight
    
    ## Returns current flight tolerance value
    def GetTolerance(self):
        return self.__tolerance

    ## Increment target point index
    def __incrementTarget(self):
        self.__curIdx += 1
        self.__movementStarted = False
        if(self.__curIdx < len(self.__path)):
            trgPt = self.__path[self.__curIdx]
            rospy.loginfo("CMission: target point updated to [%.3f, %.3f]", trgPt[0], trgPt[1])

    ## Mission path topic callback
    def __onMissionChanged(self, mission):
        self.hash = mission.hash
        self.__pathGlobal.clear()
        for ptGeo in mission.path:
            self.__pathGlobal.append([ptGeo.x, ptGeo.y])
        rospy.loginfo("CMission: mission updated (hash = %#08x)", self.hash)

    ## Target height topic callback
    def __onHeightChanged(self, height):
        self.__targetHeight = height.value
        rospy.loginfo("CMission: target height updated (%f)", self.__targetHeight)

    ## Tolerance topic callback
    def __onToleranceChanged(self, tolerance):
        self.__tolerance = tolerance.value
        rospy.loginfo("CMission: tolerance updated (%f)", self.__tolerance)