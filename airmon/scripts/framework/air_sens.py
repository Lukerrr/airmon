#!/usr/bin/env python3

import rospy
from clover import srv

from std_msgs.msg import Float32MultiArray
from airmon_comm.msg import AirSensData
from airmon_comm.msg import AirSensDataArray

class CAirSens():
    def __init__(self):
        self.__data = []
        self.__airSensSub = rospy.Subscriber("Air_sens", Float32MultiArray, self.__onAirSensChanged)
        self.__airSensPub = rospy.Publisher("airmon_comm/out/air_sens", AirSensDataArray, queue_size = 10)
        self.__getTelemetrySrv = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

    def ResetData(self):
        self.__data = []

    def GetDataCount(self):
        return len(self.__data)

    def PublishSensData(self):
        if self.GetDataCount() > 0:
            msg = AirSensDataArray()
            for dataEntry in self.__data:
                msg.dataArray.append(dataEntry)
            self.__airSensPub.publish(msg)
            rospy.loginfo("CAirSens: published %d data entries to communicator", self.GetDataCount())
        else:
            rospy.loginfo("CAirSens: air data is empty, nothing was published")

    def __onAirSensChanged(self, data):
        telem = self.__getTelemetrySrv()
        airData = AirSensData()
        airData.lat = telem.lat
        airData.lon = telem.lon
        airData.temp = data.data[0]
        airData.hum = data.data[1]
        airData.co = data.data[2]
        airData.co2 = data.data[3]
        airData.nh3 = data.data[4]
        airData.no2 = data.data[5]
        airData.tvoc = data.data[6]
        self.__data.append(airData)
