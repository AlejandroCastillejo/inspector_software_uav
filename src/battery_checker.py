#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import BatteryState
from inspector_software_uav.srv import StopService

rospy.wait_for_service('stop_service')
stop_client = rospy.ServiceProxy ('stop_service', StopService)

def main():
    rospy.init_node('battery_checker')


    def dji_battery_cb(data):
        percentage = data.percentage
        print ('battery percentage: ', percentage)
        if percentage < 30.0:
            try:
                resp = stop_client()
                print "Stop service called"
                return resp
            except rospy.ServiceException, e:
                print "Stop service call failed: %s"%e

        

    dji_battery_subscriber = rospy.Subscriber("dji_sdk/battery_state", BatteryState, dji_battery_cb, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass