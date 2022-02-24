#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32MultiArray
from airmon_comm.msg import AirSensData
from airmon_comm.msg import AirSensDataArray

class CAirSens():
    def __init__(self, gps):
        self.__gps = gps
        self.__data = []
        self.__airSensSub = rospy.Subscriber("Air_sens", Float32MultiArray, self.__onAirSensChanged)
        self.__airSensPub = rospy.Publisher("airmon_comm/out/air_sens", AirSensDataArray, queue_size = 10)

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
        airData = AirSensData()
        airData.lat = self.__gps.lat
        airData.lon = self.__gps.lon
        airData.temp = data_array[0]
        airData.hum = data_array[1]
        airData.co = data_array[2]
        airData.co2 = data_array[3]
        airData.nh3 = data_array[4]
        airData.no2 = data_array[5]
        airData.tvoc = data_array[6]
        self.__data.append(airData)

        print(f"""Air data (#{self.GetDataCount()}):
            Lat = {airData.lat};
            Lon = {airData.lon};
            Temperature = {airData.temp};
            Humidity = {airData.hum};
            CO = {airData.co};
            CO2 = {airData.co2};
            NH3 = {airData.nh3};
            NO2 = {airData.no2};
            TVOC = {airData.tvoc}""")
