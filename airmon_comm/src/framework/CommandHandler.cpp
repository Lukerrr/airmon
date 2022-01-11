#include "CommandHandler.h"
#include "CmdMsgs.h"
#include "Log.h"
#include "Crc32.h"

#include "ros/ros.h"
#include "airmon_comm/GsCmdSimple.h"
#include "airmon_comm/GsCmdMission.h"
#include "airmon_comm/GsCmdFloat.h"

#include <string.h>

static ros::Publisher s_armPub;
static ros::Publisher s_disarmPub;
static ros::Publisher s_startPub;
static ros::Publisher s_stopPub;
static ros::Publisher s_heightPub;
static ros::Publisher s_tolerancePub;
static ros::Publisher s_missionPub;

void SendSimpleCmd(ros::Publisher& pub)
{
    airmon_comm::GsCmdSimple msg;
    msg.header = std_msgs::Header();
    msg.header.frame_id = "";
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
}

void SendFloatCmd(ros::Publisher& pub, float val)
{
    airmon_comm::GsCmdFloat msg;
    msg.header = std_msgs::Header();
    msg.header.frame_id = "";
    msg.header.stamp = ros::Time::now();
    msg.value = val;
    pub.publish(msg); 
}

void OnCmdArm(void* data)
{
    SendSimpleCmd(s_armPub);
}

void OnCmdDisarm(void* data)
{
    SendSimpleCmd(s_disarmPub);
}

void OnCmdStart(void* data)
{
    SendSimpleCmd(s_startPub);
}

void OnCmdStop(void* data)
{
    SendSimpleCmd(s_stopPub);
}

void OnCmdHeight(void* data)
{
    SendFloatCmd(s_heightPub, *((float*)data));
}

void OnCmdTolerance(void* data)
{
    SendFloatCmd(s_tolerancePub, *((float*)data));
}

void OnCmdMission(void* data)
{
    airmon_comm::GsCmdMission msg;
    msg.header = std_msgs::Header();
    msg.header.frame_id = "";
    msg.header.stamp = ros::Time::now();

    SMissionData missionData = *((SMissionData*)data);

    for(int i = 0; i < missionData.pathSize; ++i)
    {
        geometry_msgs::Pose2D pose;
        pose.x = missionData.path[i].x;
        pose.y = missionData.path[i].y;
        pose.theta = 0.0;
        msg.path.push_back(pose);
    }

    msg.hash = GetCrc32((uint8_t*)missionData.path, missionData.pathSize * sizeof(SMissionData::Point));
    s_missionPub.publish(msg);
}

CCommandHandler::CCommandHandler()
{
    m_handlers[CMD_ARM] = OnCmdArm;
    m_handlers[CMD_DISARM] = OnCmdDisarm;
    m_handlers[CMD_START] = OnCmdStart;
    m_handlers[CMD_STOP] = OnCmdStop;
    m_handlers[CMD_HEIGHT] = OnCmdHeight;
    m_handlers[CMD_TOLERANCE] = OnCmdTolerance;
    m_handlers[CMD_MISSION] = OnCmdMission;
    
    ros::NodeHandle nh("~");
    s_armPub = nh.advertise<airmon_comm::GsCmdSimple>("in/cmd_arm", 1);
    s_disarmPub = nh.advertise<airmon_comm::GsCmdSimple>("in/cmd_disarm", 1);
    s_startPub = nh.advertise<airmon_comm::GsCmdSimple>("in/cmd_start", 1);
    s_stopPub = nh.advertise<airmon_comm::GsCmdSimple>("in/cmd_stop", 1);
    s_heightPub = nh.advertise<airmon_comm::GsCmdFloat>("in/cmd_height", 1);
    s_tolerancePub = nh.advertise<airmon_comm::GsCmdFloat>("in/cmd_tolerance", 1);
    s_missionPub = nh.advertise<airmon_comm::GsCmdMission>("in/cmd_mission", 1);
}

CCommandHandler::~CCommandHandler()
{
    memset(m_handlers, 0, sizeof(m_handlers));
}

void CCommandHandler::Invoke(ECmdType type, void* data)
{
    if(type < CMD_MAX)
    {
        TCmdHandler handler = m_handlers[type];
        if(handler != NULL)
        {
            handler(data);
        }
    }
}