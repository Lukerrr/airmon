#include "framework/Communicator.h"
#include "framework/Log.h"

#include "ros/ros.h"
#include "airmon_comm/DroneState.h"
#include "airmon_comm/PointCloud.h"
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
    state.density = msg.density;
    state.cloudSize = msg.cloudSize;

    state.bArmed = msg.armed != 0;
    state.charge = msg.charge;

    g_pComm->Send(state);
}

void OnPointCloudReceived(const airmon_comm::PointCloud& msg)
{
    const geometry_msgs::Vector3* cloud = msg.cloud.data();
    int pointsNum = msg.cloud.size();
    while(pointsNum > 0)
    {
        SPointCloud chunk;
        chunk.size = pointsNum > SPointCloud::chunkMaxSize ? SPointCloud::chunkMaxSize : pointsNum;
        memcpy(chunk.cloud, cloud, chunk.size * sizeof(geometry_msgs::Vector3));
        cloud += chunk.size;
        pointsNum -= chunk.size;
        g_pComm->Send(chunk);
    }
    SRspCloudEnd endMsg;
    g_pComm->Send(endMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airmon_comm");

    ros::NodeHandle nh("~");
    int rate = nh.param("rate", COMM_RATE_DEF);

    ros::Subscriber droneStateSub = nh.subscribe("out/drone_state", 1, OnDroneStateUpdated);
    ros::Subscriber pointCloudSub = nh.subscribe("out/point_cloud", 1, OnPointCloudReceived);

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
