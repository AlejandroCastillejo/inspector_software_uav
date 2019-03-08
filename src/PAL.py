#!/usr/bin/env python

import rospy
import rosbag

import socket
import gphoto2 as gp
import os.path
import time
import datetime

from std_msgs.msg import Int32, Time
from sensor_msgs.msg import Image, NavSatFix
from inspector_software_uav.srv import WebcamBagService, CameraCaptureService

WIRIS_IP = '10.0.2.228'
WIRIS_TCP_PORT = 2240
class PAL:

    def __init__(self):
        
        ## Webcam
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

        ## WIRIS thermal camera
        thermal_camera_capture_srv = rospy.Service('thermal_camera_capture_service', CameraCaptureService, self.thermal_camera_capture_service_cb)

        ## SONY rgb camera
        rgb_camera_capture_srv = rospy.Service('rgb_camera_capture_service', CameraCaptureService, self.rgb_camera_capture_service_cb)

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

    def thermal_camera_capture_service_cb(self, req):
        sleep_time = 1/req.rate
        if req.capture == True:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((WIRIS_IP, WIRIS_TCP_PORT))
            s.send(HIWS)
            if s.recv == 'OK':
                rospy.loginfo("WIRIS Camera OK. Starting secuence capture")
                while True:
                    s.send(CIMG)
                    rospy.sleep(sleep_time)
            else:
                rospy.logerr("ERROR. WIRIS Connection Failed")
        if req.capture == False:
            s.close()
        
    def rgb_camera_capture_service_cb(self, req):
        print('Capturing rgb images')
        


def main():
    rospy.init_node('PAL')
    pal = PAL()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
