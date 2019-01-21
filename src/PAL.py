#!/usr/bin/env python

import rospy
import rosbag

import os.path
import time
import datetime

from std_msgs.msg import Int32, Time
from sensor_msgs.msg import Image, NavSatFix
from inspector_software_uav.srv import WebcamBagService


class PAL:

    def __init__(self):
        self.webcam_record = False
        self.gpsA3_record = False

        # self.rostime = Time()
        self.desktop = os.path.expanduser("~/Desktop")

        if not os.path.exists(self.desktop +'/mission_bags/'):
            os.makedirs(self.desktop +'/mission_bags/')

        rospy.Subscriber('usb_cam/image_raw', Image, self.usb_cam_cb)
        rospy.Subscriber('dji_sdk/gps_position', NavSatFix, self.dji_gps_position_cb)

        webcam_bag_srv = rospy.Service('webcam_bag_service', WebcamBagService, self.webcam_bag_service_cb)
        # gpsA3_bag_srv = rospy.Service('gpsA3_bag_service', GpsA3Service, self.gpsA3_bag_service_cb)
        
        # if not os.path.isfile("webcam_bag.bag"):
        #     self.webcam_bag = rosbag.Bag('webcam_bag.bag', 'w')
        #     self.webcam_bag.close()
        
        # if not os.path.isfile("gpsA3_bag.bag"):
        #     self.webcam_bag = rosbag.Bag('gpsA3_bag.bag', 'w')
        #     self.webcam_bag.close()


    ## Subscribers callbacks

    def usb_cam_cb(self, image_msg):
        if self.webcam_record == True:
            rostime = Time()
            rostime.data = rospy.get_rostime()
            self.webcam_bag.write('rostime', rostime)
            self.webcam_bag.write('webcamImage', image_msg)

    def dji_gps_position_cb(self, gps_msg):
        if self.webcam_record == True:
            rostime = Time()
            rostime.data = rospy.get_rostime()
            self.gpsA3_bag.write('rostime', rostime)
            self.gpsA3_bag.write('geoPosition', gps_msg)


    ## Services callbacks

    def webcam_bag_service_cb(self, req):
        if req.record == True:
            time_now = datetime.datetime.now()
            self.webcam_bag = rosbag.Bag(self.desktop +'/mission_bags/webcam_{0}.bag'.format(time_now.strftime("%Y-%m-%d %H:%M:%S_")), 'w')
            self.gpsA3_bag = rosbag.Bag(self.desktop +'/mission_bags/gpsA3_{0}.bag'.format(time_now.strftime("%Y-%m-%d %H:%M:%S")), 'w')
            self.webcam_record = req.record
        elif req.record == False:
            self.webcam_record = req.record
            self.webcam_bag.close()
            self.gpsA3_bag.close()
        return True


def main():
    rospy.init_node('PAL')
    pal = PAL()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
