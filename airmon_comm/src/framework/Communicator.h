#pragma once

#include "Singleton.h"
#include "CommandHandler.h"
#include "RspMsgs.h"

#define g_pComm ((&CCommunicator::Instance()))

class CCommunicator : public TSingleton<CCommunicator>
{
    struct SRawPacket
    {
        ~SRawPacket();

        ECmdType type = CMD_MAX;
        uint8_t* payload = NULL;
        size_t requiredSize = 0;
        size_t curSize = 0;
    };
public:
    CCommunicator();
    ~CCommunicator();

    void Update();

    template<typename T>
    void Send(T msg);

private:
    bool Initialize();
    void Invalidate();
    void DisconnectClient();

    bool ConstructPacket();

    int RecvInternal(int socket, void* buf, size_t len);
    void SendInternal(char* pData, int len);

    uint16_t m_port;
    uint64_t m_timeout;
    uint64_t m_lastDataStamp;

    CCommandHandler m_cmdHandler;

    int m_svSock = -1;
    int m_clSock = -1;

    bool m_bIsValid = false;

    SRawPacket m_curPacket;
};

template<typename T>
void CCommunicator::Send(T msg)
{
    int len = sizeof(T) + sizeof(ERspType);
    char* buf = new char[len];
    memcpy(buf, &T::s_type, sizeof(ERspType));
    memcpy(buf + sizeof(ERspType), &msg, sizeof(T));
    SendInternal(buf, len);
    delete[] buf;
}