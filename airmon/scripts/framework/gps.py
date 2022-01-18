#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField

from helpers.math_utils import *
from helpers.filter import CFilter

################################################################################
## A class to store quadrotor's globol position data via GPS.
##
## Fields list:
##  lat - latitude
##  lon - longitude
##  isValid - is gps data valid
##  heading - compass heading in degrees
##  __dt - the system delta time in seconds
##  __movCtrl - CMovementController instance ref to receive rotation values
##  __magFilterX - a filter for magnetometer values (axis X)
##  __magFilterY - a filter for magnetometer values (axis Y)
##  __gpsPosSub - gps data topic subscriber
##  __magSub - magnetometer data topic publisher
##
##
## Methods list:
##  __onPositionChanged - gps data topic callback
##  __onMagneticFieldChanged - magnetometer data topic callback
################################################################################
class CGpsSystem:
    def __init__(self, movCtrl, dt):
        self.lat = 0.
        self.lon = 0.
        self.isValid = False
        self.heading = 0.

        self.__dt = dt

        self.__movCtrl = movCtrl

        T1 = 0.25
        T2 = 0.1
        self.__magFilterX = CFilter(T1, T2)
        self.__magFilterY = CFilter(T1, T2)

        self.__gpsPosSub = rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, self.__onPositionChanged)
        self.__magSub = rospy.Subscriber("mavros/imu/mag", MagneticField, self.__onMagneticFieldChanged)

    ## Gps data topic callback
    def __onPositionChanged(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.isValid = data.status.status > data.status.STATUS_NO_FIX

    ## Magnetometer data topic callback
    def __onMagneticFieldChanged(self, data):
        mag = data.magnetic_field
        magVector = Vec3(mag.x, mag.y, mag.z)
        rotator = Rotator(self.__movCtrl.rot.roll, self.__movCtrl.rot.pitch, 0.0)
        magVectorR = magVector.Rotate(rotator)

        # Filter magnetometer data and calculate heading
        self.heading = -Rad2Deg(np.arctan2(\
            self.__magFilterY.Filter(magVectorR.y, self.__dt),\
            self.__magFilterX.Filter(magVectorR.x, self.__dt)))
        

