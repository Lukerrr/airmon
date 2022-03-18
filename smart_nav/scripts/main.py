#!/usr/bin/env python3

import rospy
import ros_numpy
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs as tf2_geometry_msgs

from geographiclib.geodesic import Geodesic

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2

from smart_nav import srv as smart_nav_srv
from clover import srv as clover_srv

import numpy as np
from typing import Optional

def get_transformation(source_frame, target_frame):
    global tf_buffer
    transformation = None
    try:
        transformation = tf_buffer.lookup_transform(target_frame,
                source_frame, rospy.Time(0), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f'Unable to find the transformation from {source_frame} to {target_frame}: {str(e)}')

    return transformation

class navigate_task():

    def __init__(self, pos: np.ndarray, yaw: float, yaw_rate: float, vel_thr: float, auto_arm: bool):
        self.pos: np.ndarray = pos
        self.yaw: float = yaw
        self.yaw_rate: float = yaw_rate
        self.vel_thr: float = vel_thr
        self.auto_arm: bool = auto_arm

class smart_nav_service():

    def __init__(self, zeta: float = 0.5, d_goal_thr: float = 2.0, nu: float = 1.0, d_obst_thr: float = 2.0, ignore_z: bool = True):
        self.__obst_sub = rospy.Subscriber("smart_nav/obstacles", PointCloud2, self.__handle_obstacles)
        self.__nav_srv = rospy.Service('smart_nav/navigate', clover_srv.Navigate, self.__handle_navigate)
        self.__nav_global_srv = rospy.Service('smart_nav/navigate_global', clover_srv.NavigateGlobal, self.__handle_navigate_global)
        self.__get_nav_vector_srv = rospy.Service('smart_nav/get_nav_vector', smart_nav_srv.GetNavVector, self.__handle_get_nav_vector)

        self.__get_telemetry = rospy.ServiceProxy('get_telemetry', clover_srv.GetTelemetry)
        self.__set_velocity = rospy.ServiceProxy('set_velocity', clover_srv.SetVelocity)

        self.__nav_task: Optional[navigate_task] = None
        self.__obst_arr: np.ndarray = np.array([])

        self.__zeta: float = zeta
        self.__d_goal_thr: float = d_goal_thr
        self.__nu: float = nu
        self.__d_obst_thr: float = d_obst_thr
        self.__ignore_z: bool = ignore_z

        self.__cur_pos = None
        self.__cur_lat = None
        self.__cur_lon = None

        rospy.loginfo(f"Initialized smart_nav service, zeta = {zeta}, nu = {nu}, d_goal_thr = {d_goal_thr}, d_obst_thr = {d_obst_thr}, ignore_z = {ignore_z}")

    def update(self):
        telem = self.__get_telemetry(frame_id='map')
        self.__cur_pos = np.array([telem.x, telem.y, telem.z])
        self.__cur_lat = telem.lat
        self.__cur_lon = telem.lon

        if self.__nav_task is None:
            self.__set_velocity(vx=0, vy=0, vz=0, yaw=float('nan'), frame_id='map', auto_arm=False)
            return

        vel: np.ndarray = self.__calc_velocity(self.__cur_pos)
        vel_size: float = np.linalg.norm(vel)

        # Apply velocity threshold
        if vel_size > self.__nav_task.vel_thr:
            vel /= vel_size
            vel *= self.__nav_task.vel_thr

        # Send velocity
        self.__set_velocity(vx=vel[0], vy=vel[1], vz=vel[2],\
            yaw=self.__nav_task.yaw, yaw_rate=self.__nav_task.yaw_rate,\
            frame_id='map', auto_arm=self.__nav_task.auto_arm)

        # Disable auto arm in next updates
        if self.__nav_task.auto_arm:
            self.__nav_task.auto_arm = False

    def __global_to_local(self, lat: float, lon: float, z: float) -> np.ndarray:
        inv_res = Geodesic.WGS84.Inverse(self.__cur_lat, self.__cur_lon, lat, lon)
        distance: float = inv_res['s12']
        azimuth: float = inv_res['azi2']

        azimuth_radians: float = azimuth * np.pi / 180
        x_offset: float = distance * np.sin(azimuth_radians)
        y_offset: float = distance * np.cos(azimuth_radians)

        x: float = self.__cur_pos[0] + x_offset
        y: float = self.__cur_pos[1] + y_offset

        return np.asarray([x, y, z])

    def __calc_attractive(self, pt_cur: np.ndarray) -> np.ndarray:
        if self.__nav_task is None:
            return np.zeros(3)

        v_goal: np.ndarray = self.__nav_task.pos - pt_cur
        d_goal: np.ndarray = np.linalg.norm(v_goal)

        if d_goal > self.__d_goal_thr:
            # Cut velocity growth with distance increasing
            v_goal /= d_goal
            v_goal *= self.__d_goal_thr

        result: np.ndarray = self.__zeta * v_goal

        #if d_goal > self.__d_goal_thr:
            # Cut velocity growth with distance increasing
            #result *= (self.__d_goal_thr / d_goal)

        return result

    def __calc_repulsive(self, pt_cur: np.ndarray, pt_obst: np.ndarray) -> np.ndarray:
        if self.__nav_task is None:
            return np.zeros(3)

        v_obst: np.ndarray = pt_obst - pt_cur 
        d_obst: np.ndarray = np.linalg.norm(v_obst)

        if d_obst > self.__d_obst_thr:
            # Ignore far obstacles
            return np.zeros(3)

        result: np.ndarray = self.__nu * ((1 / self.__d_obst_thr) - (1 / d_obst)) / (d_obst * d_obst) * v_obst

        return result

    def __calc_velocity(self, pt_cur: np.ndarray) -> np.ndarray:
        if self.__nav_task is None:
            return np.zeros(3)

        # Attractive velocity (to goal)
        vel_attr: np.ndarray = self.__calc_attractive(pt_cur)

        # Repulsive velocity (from obstacles)
        if len(self.__obst_arr) > 0:
            vel_rep_arr: np.ndarray =\
                np.array([self.__calc_repulsive(pt_cur, pt_obst) for pt_obst in self.__obst_arr])
            vel_rep: np.ndarray = np.mean(vel_rep_arr.T, axis = 1)
        else:
            vel_rep: np.ndarray = np.zeros(3)

        # Ignoring repulsive Z coordinate
        if self.__ignore_z:
            vel_rep[2] = 0

        return vel_attr + vel_rep

    def __handle_obstacles(self, msg: PointCloud2):
        self.__obst_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    def __handle_navigate(self, req: clover_srv.NavigateRequest):
        rospy.loginfo(f"Handled navigate: {str(req)}")

        tf_to_map = get_transformation(req.frame_id, "map")

        if tf_to_map is None:
            rospy.logerr(f"NO TRANSFORMATION!")
            return clover_srv.NavigateResponse(False, 'BAD_FRAME')

        if self.__cur_pos is None:
            rospy.logerr(f"NO TELEMETRY DATA!")
            return clover_srv.NavigateResponse(False, 'BAD_TELEM')

        pt_local = PointStamped()
        pt_local.point.x = req.x
        pt_local.point.y = req.y
        pt_local.point.z = req.z
        pt_local.header = tf_to_map.header

        # Transform to map frame
        pt_map: PointStamped = tf2_geometry_msgs.do_transform_point(pt_local, tf_to_map)

        pos_map: np.ndarray = np.array([pt_map.point.x, pt_map.point.y, pt_map.point.z])
        self.__nav_task = navigate_task(pos_map, req.yaw, req.yaw_rate, req.speed, req.auto_arm)

        return clover_srv.NavigateResponse(True, 'OK')

    def __handle_navigate_global(self, req: clover_srv.NavigateGlobalRequest):
        rospy.loginfo(f"Handled navigate global: {str(req)}")

        if self.__cur_pos is None:
            rospy.logerr(f"NO TELEMETRY DATA!")
            return clover_srv.NavigateGlobalResponse(False, 'BAD_TELEM')

        pos_map: np.ndarray = np.array(self.__global_to_local(req.lat, req.lon, req.z))
        self.__nav_task = navigate_task(pos_map, req.yaw, req.yaw_rate, req.speed, req.auto_arm)

        return clover_srv.NavigateGlobalResponse(True, 'OK')

    def __handle_get_nav_vector(self, req: smart_nav_srv.GetNavVectorRequest):
        if self.__nav_task is None:
            return smart_nav_srv.GetNavVectorResponse(0, 0, 0)

        nav_v: np.ndarray = self.__nav_task.pos - self.__cur_pos

        return smart_nav_srv.GetNavVectorResponse(nav_v[0], nav_v[1], nav_v[2])

if __name__ == "__main__":
    rospy.init_node("smart_nav", anonymous = True)

    global tf_buffer
    tf_buffer = tf2_ros.Buffer(rospy.Duration(2.0))
    tf2_ros.TransformListener(tf_buffer)

    rate_hz: float = rospy.get_param("~rate", default = 50)
    zeta: float = rospy.get_param("~zeta", default = 0.5)
    d_goal_thr: float = rospy.get_param("~d_goal_thr", default = 2.0)
    nu: float = rospy.get_param("~nu", default = 1.0)
    d_obst_thr: float = rospy.get_param("~d_obst_thr", default = 2.0)
    ignore_z: float = rospy.get_param("~ignore_z", default = True)

    nav_srv: smart_nav_service = smart_nav_service(zeta, d_goal_thr, nu, d_obst_thr, ignore_z)
    rate: rospy.Rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        try:
            nav_srv.update()
        except Exception as e:
            rospy.logerr(f"Unhandled exception: {str(e)}")
        rate.sleep()