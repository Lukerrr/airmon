#pragma once

#include "MsgTypes.h"

struct CCommandHandler
{
    // data == NULL if no data provided with the command
    typedef void (*TCmdHandler)(void *data);

public:
    CCommandHandler();
    ~CCommandHandler();

    void Invoke(ECmdType type, void* data);

private:
    TCmdHandler m_handlers[CMD_HEARTBEAT];
};