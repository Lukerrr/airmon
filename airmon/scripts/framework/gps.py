#!/usr/bin/env python3

import rospy

from clover import srv
from sensor_msgs.msg import NavSatFix

from helpers.math_utils import *

################################################################################
## A class to store quadrotor's global position data via GPS.
##
## Fields list:
##  lat - latitude
##  lon - longitude
##  lon - longitude
##  isValid - is gps data valid
##  __gpsPosSub - gps data topic subscriber
##
##
## Methods list:
##  __onPositionChanged - gps data topic callback
################################################################################
class CGpsSystem:
    def __init__(self):
        self.isValid = False
        self.__getTelemetrySrv = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.__gpsPosSub = rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, self.__onPositionChanged)

    ## Gps data topic callback
    def __onPositionChanged(self, data):
        self.isValid = data.status.status > data.status.STATUS_NO_FIX

    @property
    def lat(self):
        telem = self.__getTelemetrySrv()
        return telem.lat

    @property
    def lon(self):
        telem = self.__getTelemetrySrv()
        return telem.lon

    @property
    def heading(self):
        telem = self.__getTelemetrySrv()
        return Rad2Deg(telem.yaw)
