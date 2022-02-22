#!/usr/bin/env python3

## For python3 - fix paths
import sys
sys.path.append('/home/pi/catkin_ws/src/airmon/airmon/scripts')
##

import rospy

from framework.system import CSystem

if __name__ == "__main__":
    system = CSystem()
    try:
        system.Run()
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROSInterruptException: %s", str(e))
    finally:
        system.OnExit()
