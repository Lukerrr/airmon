#pragma once

#include <stdint.h>

typedef uint8_t EMsgType;

enum ECmdType : EMsgType
{
    CMD_ARM = 0,
    CMD_DISARM,
    CMD_START,
    CMD_STOP,
    CMD_HEIGHT,
    CMD_TOLERANCE,
    CMD_DENSITY,
    CMD_GET_CLOUD,
    CMD_MISSION,
    CMD_HEARTBEAT,

    CMD_MAX,
};

enum ERspType : EMsgType
{
    RSP_DRONE_STATE = 0,
    RSP_CLOUD_CHUNK,
    RSP_CLOUD_END,

    RSP_MAX,
};