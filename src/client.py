#!/usr/bin/env python

from inspector_software_uav.srv import *

import rospy

def mission_client():
    rospy.wait_for_service('mission_service')
    try:
        mission_service = rospy.ServiceProxy('mission_service', MissionService)
        resp = mission_service (3)
        print resp
    except rospy.ServiceException, e:
        print "serv call failed"


if __name__ == "__main__":
    mission_client()