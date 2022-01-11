/*
* Response messages declaration
* Drone -> GroundStation
*/

#pragma once

#include "MsgDeclaration.h"

enum ESystemState : uint8_t
{
    ST_DISCONNECTED = 0,
    ST_IDLE,
    ST_TAKEOFF,
    ST_WORKING,
    ST_LANDING,
};

DECLARE_MSG(RSP_DRONE_STATE, SRspDroneState,

    // Global position data
    double lat;
    double lon;
    double heading;
    bool bGlobalPos;

    // Position data
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;

    // Service data
    ESystemState systemState;
    bool bArmed;
    float charge;

    // Mission params
    uint32_t missionHash;
    float height;
    float tolerance;
    float density;
)

typedef SRspDroneState SDroneState;