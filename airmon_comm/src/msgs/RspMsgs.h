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
    uint32_t cloudSize;
    bool bArmed;
    float charge;

    // Mission params
    uint32_t missionHash;
    float height;
    float tolerance;
    float density;
)

DECLARE_MSG(RSP_CLOUD_CHUNK, SRspCloudChunk,
    struct CloudPt
    {
        double x;
        double y;
        double z;
    };

    uint32_t size;

    static const int chunkMaxSize = 64; 
    CloudPt cloud[chunkMaxSize];
)

DECLARE_MSG_NO_PARAMS(RSP_CLOUD_END, SRspCloudEnd)

typedef SRspDroneState SDroneState;
typedef SRspCloudChunk SPointCloud;