#!/usr/bin/env python3

import rospy

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from clover import srv as clover_srv
from smart_nav import srv as smart_nav_srv

from math_utils import Vec3, Rotator
import laser_geometry.laser_geometry as lg
from sensor_msgs.msg import PointCloud2, LaserScan
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

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

        self.__tf_buffer = tf2_ros.Buffer(rospy.Duration(0.1))
        self.__lp = lg.LaserProjection()
        tf2_ros.TransformListener(self.__tf_buffer)

        self.__navigateSrv = rospy.ServiceProxy('smart_nav/navigate', clover_srv.Navigate)
        self.__navigateGlSrv = rospy.ServiceProxy('smart_nav/navigate_global', clover_srv.NavigateGlobal)
        self.__getNavVector = rospy.ServiceProxy('smart_nav/get_nav_vector', smart_nav_srv.GetNavVector)
        self.__landSrv = rospy.ServiceProxy('land', Trigger)

        self.__stateSub = rospy.Subscriber("mavros/state", State, self.__onStateChanged)
        self.__armingClient = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.__setModeClient = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.__lsSub = rospy.Subscriber("scan", LaserScan, self.__onLaserUpdated)
        self.__obstPub = rospy.Publisher("smart_nav/obstacles", PointCloud2, queue_size=1)

        self.__speed = 1.0

    def __get_transformation(self, source_frame, target_frame, tf_cache_duration=2.0):
        transformation = None

        try:
            transformation = self.__tf_buffer.lookup_transform(target_frame,
                    source_frame, rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f'Unable to find the transformation from {source_frame} to {target_frame}: {str(e)}')

        return transformation

    def __onLaserUpdated(self, data):
        pc2_msg: PointCloud2 = self.__lp.projectLaser(data)
        map_tf = self.__get_transformation("rplidar_link", "map")
        if map_tf is not None:
            pc2_msg_map: PointCloud2 = do_transform_cloud(pc2_msg, map_tf)
            self.__obstPub.publish(pc2_msg_map)

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
        self.__navigateSrv(x=x, y=y, z=z, yaw=0.0, speed=self.__speed, auto_arm=arm, frame_id='map')

    ## Set quadrotor target position (global coordinates)
    def SetPosGl(self, lat, lon, z, arm = False):
        self.__navigateGlSrv(lat=lat, lon=lon, z=z, yaw=0.0, speed=self.__speed, auto_arm=arm, frame_id='map')

    ## Returns a distance vector to the navigate point
    def GetNavigateError(self) -> Vec3:
        res = self.__getNavVector()
        return Vec3(res.x, res.y, res.z)

    ## Checks is a drone currently at a given point
    def IsAtPos(self, pos: Vec3, eps: float = 0.2) -> bool:
        dist: float = (self.pos - pos).GetLength()
        return dist <= eps

    ## Begin landing
    def Land(self):
        self.__landSrv()

    ## Returns a telemetry struct from get_telemetry service
    def __getTelemetrySrv(self):
        getTelemetrySrvProxy = rospy.ServiceProxy('get_telemetry', clover_srv.GetTelemetry)
        return getTelemetrySrvProxy()

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
