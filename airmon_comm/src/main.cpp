#include "framework/Communicator.h"
#include "framework/Log.h"

#include "ros/ros.h"
#include "airmon_comm/DroneState.h"
#include "airmon_comm/AirSensDataArray.h"
#include "airmon_comm/GsCmdSimple.h"

#include <sstream>

#define COMM_RATE_DEF 10

void OnDroneStateUpdated(const airmon_comm::DroneState& msg)
{
    SDroneState state;

    state.lat = msg.lat;
    state.lon = msg.lon;
    state.heading = msg.heading;
    state.bGlobalPos = msg.globalPos != 0;

    state.x = msg.pos.x;
    state.y = msg.pos.y;
    state.z = msg.pos.z;

    state.roll = msg.angles.x;
    state.pitch = msg.angles.y;
    state.yaw = msg.angles.z;

    state.systemState = (ESystemState)msg.systemState;
    state.missionHash = msg.missionHash;
    state.height = msg.height;
    state.tolerance = msg.tolerance;
    state.airSensDataCount = msg.airSensDataCount;
    state.bArmed = msg.armed != 0;
    state.charge = msg.charge;

    g_pComm->Send(state);
}

void OnAirSensDataReceived(const airmon_comm::AirSensDataArray& msg)
{
    const airmon_comm::AirSensData* airSensData = msg.dataArray.data();
    int dataCount = msg.dataArray.size();
    while(dataCount > 0)
    {
        SAirSens chunk;
        chunk.size = dataCount > SAirSens::chunkMaxSize ? SAirSens::chunkMaxSize : dataCount;
        memcpy(chunk.data, airSensData, chunk.size * sizeof(airmon_comm::AirSensData));
        airSensData += chunk.size;
        dataCount -= chunk.size;
        g_pComm->Send(chunk);
    }
    SRspAirSensEnd endMsg;
    g_pComm->Send(endMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airmon_comm");

    ros::NodeHandle nh("~");
    int rate = nh.param("rate", COMM_RATE_DEF);

    ros::Subscriber droneStateSub = nh.subscribe("out/drone_state", 1, OnDroneStateUpdated);
    ros::Subscriber airSensSub = nh.subscribe("out/air_sens", 1, OnAirSensDataReceived);

    ros::Rate loopRate(rate);

    while (ros::ok())
    {
        g_pComm->Update();

        ros::spinOnce();
        loopRate.sleep();
    }

    CCommunicator::Destroy();

    return 0;
}
