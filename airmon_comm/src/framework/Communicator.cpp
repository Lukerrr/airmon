#include "Communicator.h"
#include "CmdMsgs.h"
#include "Log.h"

#include "ros/ros.h"

#include <sys/ioctl.h>
#include <arpa/inet.h>

#define CONN_PORT_DEF 54000
#define CONN_TIMEOUT_DEF 5000
#define CONN_MAX_CLIENTS 1

size_t GetCmdDataSize(ECmdType type)
{
    switch(type)
    {
    case CMD_ARM:               return sizeof(SCmdArm);
    case CMD_DISARM:            return sizeof(SCmdDisarm);
    case CMD_START:             return sizeof(SCmdStart);
    case CMD_STOP:              return sizeof(SCmdStop);
    case CMD_HEIGHT:            return sizeof(SCmdHeight);
    case CMD_TOLERANCE:         return sizeof(SCmdTolerance);
    case CMD_DENSITY:           return sizeof(SCmdDensity);
    case CMD_GET_CLOUD:         return sizeof(SCmdGetCloud);
    case CMD_MISSION:           return sizeof(SCmdMission);
    case CMD_HEARTBEAT:         return sizeof(SCmdHeartbeat);
    default:
        return 0;
    }
}

inline uint64_t Millis()
{
    return ros::Time::now().toNSec() / 1000000;
}

CCommunicator::CCommunicator()
{
    ros::NodeHandle nh("~");
    m_port = nh.param("port", CONN_PORT_DEF);
    m_timeout = nh.param("timeout", CONN_TIMEOUT_DEF);
    m_lastDataStamp = 0;
}

CCommunicator::~CCommunicator()
{
    Invalidate();
}

CCommunicator::SRawPacket::~SRawPacket()
{
    if(payload != NULL)
    {
        delete[] payload;
    }
}

bool CCommunicator::ConstructPacket()
{
    if(m_curPacket.type == CMD_MAX)
    {
        // Begin construct packet
        int dataLen = RecvInternal(m_clSock, (char*)&m_curPacket.type, sizeof(m_curPacket.type));
        if(dataLen <= 0)
        {
            // No data or an error occured
            return false;
        }

        m_curPacket.requiredSize = GetCmdDataSize(m_curPacket.type);
        if(m_curPacket.requiredSize == 0)
        {
            // No payload required
            return true;
        }
        m_curPacket.payload = new uint8_t[m_curPacket.requiredSize];
    }

    // Continue construct packet
    size_t remainSize = m_curPacket.requiredSize - m_curPacket.curSize;
    int dataLen = RecvInternal(m_clSock, (m_curPacket.payload + m_curPacket.curSize), remainSize);

    if(dataLen == -1)
    {
        // An error occured
        return false;
    }

    m_curPacket.curSize += dataLen;

    return m_curPacket.curSize == m_curPacket.requiredSize;
}

void CCommunicator::Update()
{
    if (!m_bIsValid)
    {
        if (!Initialize())
        {
            return;
        }
    }

    if (m_clSock != -1)
    {
        // Read all received packets
        while(ConstructPacket())
        {
            m_lastDataStamp = Millis();
            if(m_curPacket.type < CMD_HEARTBEAT)
            {
                m_cmdHandler.Invoke(m_curPacket.type, m_curPacket.payload);
                ROS_INFO_NAMED(LOG_NAME, "Communicator: received command %d", m_curPacket.type);
            }

            m_curPacket = SRawPacket();
        }

        if(Millis() - m_lastDataStamp > m_timeout)
        {
            ROS_WARN_NAMED(LOG_NAME, "Communicator: disconnecting on timeout...");
            DisconnectClient();
        }
    }
    else
    {
        // Waiting for a new client
        sockaddr_in clAddr;
        socklen_t clAddrLen = sizeof(clAddr);

        m_clSock = accept(m_svSock, (sockaddr *)&clAddr, &clAddrLen);

        if (m_clSock != -1)
        {
            ROS_INFO_NAMED(LOG_NAME, "Communicator: Connected to a ground station '%s:%d' (%d)",
                           inet_ntoa(clAddr.sin_addr), clAddr.sin_port, m_clSock);
            m_lastDataStamp = Millis();
        }
    }
}

int CCommunicator::RecvInternal(int socket, void* buf, size_t len)
{
    u_long availBytes = 0;
    int ioctlRes = ioctl(socket, FIONREAD, &availBytes);

    if(ioctlRes == -1 || availBytes == 0)
    {
        return -1;
    }

    if(len > availBytes)
    {
        ROS_WARN_NAMED(LOG_NAME, "Communicator: requested receive more bytes than available (%lu > %lu)", len, availBytes);
        len = availBytes;
    }
    
    return recv(socket, buf, len, 0);
}

void CCommunicator::SendInternal(char* pData, int len)
{
    if (!m_bIsValid || m_clSock == -1)
    {
        return;
    }

    if (send(m_clSock, pData, len, 0) == -1)
    {
        ROS_ERROR_NAMED(LOG_NAME, "Communicator: Cannot send message (%s).", strerror(errno));
        Initialize();
    }
}

bool CCommunicator::Initialize()
{
    Invalidate();

    m_svSock = socket(AF_INET, SOCK_STREAM, 0);
    if (m_svSock == -1)
    {
        ROS_ERROR_NAMED(LOG_NAME, "Communicator:%s: Socket error: %s", __func__, strerror(errno));
        return false;
    }

    sockaddr_in svSocketInfo;
    svSocketInfo.sin_family = AF_INET;
    svSocketInfo.sin_port = htons(m_port);
    svSocketInfo.sin_addr.s_addr = INADDR_ANY;

    int opt = 1;
    if (setsockopt(m_svSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof (opt)) == -1)
    {
        ROS_WARN_NAMED(LOG_NAME, "Communicator:%s: Set REUSEADDR error: %s", __func__, strerror(errno));
    }

    if (bind(m_svSock, (sockaddr *)&svSocketInfo, sizeof(svSocketInfo)) == -1)
    {
        ROS_ERROR_NAMED(LOG_NAME, "Communicator:%s: Bind error: %s", __func__, strerror(errno));
        return false;
    }

    if (listen(m_svSock, CONN_MAX_CLIENTS) == -1)
    {
        ROS_ERROR_NAMED(LOG_NAME, "Communicator:%s: Listen error: %s", __func__, strerror(errno));
        return false;
    }

    ROS_INFO_NAMED(LOG_NAME, "Communicator: successfully intialized! Socket = %d", m_svSock);

    m_bIsValid = true;

    return true;
}

void CCommunicator::Invalidate()
{
    ROS_INFO_NAMED(LOG_NAME, "Communicator: invalidating...");

    if (m_svSock != -1)
    {
        shutdown(m_svSock, SHUT_RDWR);
        close(m_svSock);
        m_svSock = -1;
    }

    if (m_clSock != -1)
    {
        DisconnectClient();
    }

    m_bIsValid = false;
}

void CCommunicator::DisconnectClient()
{
    ROS_INFO_NAMED(LOG_NAME, "Communicator: disconnecting client");
    shutdown(m_clSock, SHUT_RDWR);
    close(m_clSock);
    m_clSock = -1;
}
