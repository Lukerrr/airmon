#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped
from helpers.ros_globals import *
import tf

################################################################################
## Frames transforms global broadcaster.
## This centralizes transforms publishment to avoid extrapolation issues
##
## Fields list:
##  __pose - last known drone pose
##  __tfBroadcaster - transforms broadcaster (tf module)
##  __poseSub - drone pose topic subscruber
##
## Methods list:
##  __onPoseChanged - pose topic callback function
##  UpdateTransforms - publishes all transforms data
##                      (should be called every tick of the application)
################################################################################
class CTfManager:
    def __init__(self):
        self.__pose = None
        self.__tfBroadcaster = tf.TransformBroadcaster()
        self.__poseSub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.__onPoseChanged)

    ## Pose topic callback function
    def __onPoseChanged(self, pose : PoseStamped):
        self.__pose = pose

    ## Publish transforms data
    def UpdateTransforms(self):
        if(self.__pose != None):
            # Broadcast body frame transform
            translation = (self.__pose.pose.position.x,\
                self.__pose.pose.position.y,\
                self.__pose.pose.position.z)
            rotation = (self.__pose.pose.orientation.x,\
                self.__pose.pose.orientation.y,\
                self.__pose.pose.orientation.z,\
                self.__pose.pose.orientation.w)
            self.__tfBroadcaster.sendTransform(translation, rotation, now(), "base_link", "map")

        # Lidar frame transform
        translation = (0.08, 0.0, -0.09)
        rotation = (0.0, 0.0, 0.0, 1.0)
        self.__tfBroadcaster.sendTransform(translation, rotation, now(), "laser", "base_link")
