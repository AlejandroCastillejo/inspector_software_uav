#!/usr/bin/env python

from inspector_software_uav.srv import *

import rospy

def callback():
    print "mission_server"
    return h_d(3)

def mission_server():
    rospy.init_node('mission_server_node')
    s = rospy.Service('m_service', MissionService, callback)
    rospy.spin()

if __name__ == "__main__":
    mission_server()