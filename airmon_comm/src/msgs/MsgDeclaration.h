#pragma once

#include "MsgTypes.h"
#include <cstring>

#define DECLARE_MSG(type, name, params)                             \
struct name                                                         \
{                                                                   \
    name() { memset(this, 0, sizeof(name)); }                       \
    params                                                          \
    static const uint8_t s_type = type;                             \
};

#define DECLARE_MSG_NO_PARAMS(type, name) DECLARE_MSG(type, name, )