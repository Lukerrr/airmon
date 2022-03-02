#!/usr/bin/env python3

import rospy

from airmon_comm.msg import GsCmdMission
from airmon_comm.msg import GsCmdFloat

from helpers.math_utils import *

import numpy as np

class CMission:
    def __init__(self, movCtrl, gps):
        self.hash = 0xFFFFFFFF
        self.__movCtrl = movCtrl

        sp = 5.0
        self.__path =\
        [\
            [sp,0],\
            [sp,sp],\
            [0,sp],\
            [0,0]\
        ]
        self.__curIdx = 0

        self.__targetHeight = 1.5
        self.__tolerance = 1.0

        self.__missionSub = rospy.Subscriber("airmon_comm/in/cmd_mission", GsCmdMission, self.__onMissionChanged)
        self.__heightSub = rospy.Subscriber("airmon_comm/in/cmd_height", GsCmdFloat, self.__onHeightChanged)
        self.__toleranceSub = rospy.Subscriber("airmon_comm/in/cmd_tolerance", GsCmdFloat, self.__onToleranceChanged)

    def IsValid(self):
        pathValid = len(self.__path) >= 2
        heightValid = self.__targetHeight >= 0.5
        toleranceValid = self.__tolerance > 0.0

        if(not pathValid):
            rospy.logwarn("CMissionLocal::IsValid: invalid path ('%s')", str(self.__path))
        
        if(not heightValid):
            rospy.logwarn("CMissionLocal::IsValid: invalid target height (%f)", self.__targetHeight)
        
        if(not toleranceValid):
            rospy.logwarn("CMissionLocal::IsValid: invalid tolerance (%f)", self.__tolerance)

        return pathValid and heightValid and toleranceValid
    
    def Reset(self):
        rospy.loginfo("CMission: requested reset")

        if(len(self.__path) < 2):
            rospy.loginfo("CMission: failed to reset - invalid path")
            return False

        # Reset target point
        self.__curIdx = -1
        self.__incrementTarget()

        rospy.loginfo("CMission: reset is done!")
        return True

    ## Returns current target height value
    def GetHeight(self):
        return self.__targetHeight
    
    ## Returns current flight tolerance value
    def GetTolerance(self):
        return self.__tolerance

    def Update(self):
        if(self.__curIdx >= len(self.__path)):
            # Last point is reached
            return True

        curPos = [self.__movCtrl.pos.x, self.__movCtrl.pos.y]
        trgPt = self.__path[self.__curIdx]
        prevPt = self.__path[self.__curIdx - 1]

        # Set target position
        self.__movCtrl.SetPos(trgPt[0], trgPt[1], self.__targetHeight)

        # Set target yaw
        dx = trgPt[0] - prevPt[0]
        dy = trgPt[1] - prevPt[1]
        targetYaw = np.arctan2(dy, dx)
        self.__movCtrl.SetYaw(targetYaw)
        
        xError = abs(curPos[0] - trgPt[0])
        yError = abs(curPos[1] - trgPt[1])
        
        if(xError <= self.__tolerance and yError <= self.__tolerance):
            self.__incrementTarget()
            rospy.loginfo("CMissionLocal: passed waypoint #%d of #%d", self.__curIdx, len(self.__path))

        return False

    def __incrementTarget(self):
        self.__curIdx += 1
        if(self.__curIdx < len(self.__path)):
            trgPt = self.__path[self.__curIdx]
            rospy.loginfo("CMissionLocal: target point updated to [%.3f, %.3f]", trgPt[0], trgPt[1])

    def __onMissionChanged(self, mission):
        self.hash = mission.hash
        rospy.loginfo("CMissionLocal: mission updated (hash = %#08x)", self.hash)

    def __onHeightChanged(self, height):
        self.__targetHeight = height.value
        self.__movCtrl.SetParam("MIS_TAKEOFF_ALT", 0, self.__targetHeight)
        rospy.loginfo("CMissionLocal: target height updated (%f)", self.__targetHeight)

    def __onToleranceChanged(self, tolerance):
        self.__tolerance = tolerance.value
        rospy.loginfo("CMissionLocal: tolerance updated (%f)", self.__tolerance)