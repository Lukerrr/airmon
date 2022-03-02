#!/usr/bin/env python3

from enum import Enum

class ESystemState(Enum):
    DISCONNECTED = 0
    IDLE = 1
    
    # Working states
    TAKEOFF = 2
    WORKING = 3
    LANDING = 4