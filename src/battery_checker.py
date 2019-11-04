#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import BatteryState
from inspector_software_uav.srv import StopService

# rospy.wait_for_service('stop_service')
stop_client = rospy.ServiceProxy ('stop_service', StopService)

def main():
    rospy.init_node('battery_checker')
    rospy.loginfo('battery checker node running')

    # ROS params
    # global autopilot
    autopilot = rospy.get_param('~autopilot', 'dji')
    sec_percentage =rospy.get_param('~sec_percentage', 30.0)
    if autopilot == 'mavros':
        sec_percentage = sec_percentage/100

    print 'autopilot: ', autopilot
    print 'sec_percentage: ', sec_percentage

    def battery_state_cb(data):
        percentage = data.percentage
        # print ('battery percentage: ', percentage)
        # if percentage < 30.0:
        if percentage < sec_percentage:
            try:
                resp = stop_client()
                print "Stop service called"
                return resp
            except rospy.ServiceException, e:
                print "Stop service call failed: %s"%e

    if autopilot == 'dji':
        battery_state_subscriber = rospy.Subscriber("dji_sdk/battery_state", BatteryState, battery_state_cb, queue_size=10)
    elif autopilot == 'mavros':
        battery_state_subscriber = rospy.Subscriber("mavros/battery", BatteryState, battery_state_cb, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass