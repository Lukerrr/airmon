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
    uint32_t airSensDataCount;
)

DECLARE_MSG(RSP_AIR_SENS_CHUNK, SRspAirSensChunk,
    struct AirSensData
    {
        float lat;
        float lon;
        float temp;
        float hum;
        float co;
        float co2;
        float nh3;
        float no2;
        float tvoc;
    };

    uint32_t size;

    static const int chunkMaxSize = 64; 
    AirSensData data[chunkMaxSize];
)

DECLARE_MSG_NO_PARAMS(RSP_AIR_SENS_END, SRspAirSensEnd)

typedef SRspDroneState SDroneState;
typedef SRspAirSensChunk SAirSens;
