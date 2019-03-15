#!/usr/bin/env python

import rospy
import rosbag

import socket
import gphoto2 as gp
import threading
import sys
import os.path
import csv
import time
import datetime

from std_msgs.msg import Float64, Int32, Time
from sensor_msgs.msg import Image, NavSatFix
from inspector_software_uav.srv import RecordBagService, CameraCaptureService

# Thermal camera IP address & TCP port
WIRIS_IP = '10.0.2.228'
WIRIS_TCP_PORT = 2240

class PAL:

    def __init__(self):
        
        # Start mission time
        self.start_mission_time = datetime.datetime.now()

        ## 
        self.webcam_record = False
        self.gps_record = False
        self.telemetry_save = False
        self.current_laser_altitude = 0.0
        
        # Desktop path
        self.desktop = os.path.expanduser("~/Desktop")
        # Webcam Bags directory
        self.mission_bags_dir = self.desktop +'/mission_bags/'
        # RGB Images directory
        self.rgb_images_dir = self.desktop +'/RGB_Camera_Images/'
        # Thermal Images directory
        self.thermal_images_dir = self.desktop +'/Thermal_Camera_Images/'
        # Telemetry data directory
        self.telemetry_files_dir = self.desktop + '/Telemetry_Data/'

        # In case a directory doesn't exist yet, then create it
        if not os.path.exists(self.mission_bags_dir):
            os.makedirs(self.mission_bags_dir)

        if not os.path.exists(self.rgb_images_dir):
            os.makedirs(self.rgb_images_dir)
        
        if not os.path.exists(self.thermal_images_dir):
            os.makedirs(self.thermal_images_dir)
        
        if not os.path.exists(self.telemetry_files_dir):
            os.makedirs(self.telemetry_files_dir)

    ## ROStopics Subscribers
        rospy.Subscriber('usb_cam/image_raw', Image, self.usb_cam_cb)
        rospy.Subscriber('dji_sdk/gps_position', NavSatFix, self.dji_gps_position_cb)
        rospy.Subscriber('laser_altitude', Float64, self.laser_altitude_cb)

    ## ROSservices
        # Telemetry: DJI A3 autopilot GPS & Laser altimeter
        gpsA3_bag_srv = rospy.Service('gpsA3_bag_service', RecordBagService, self.gpsA3_bag_service_cb)
        telemetry_data_srv = rospy.Service('telemetry_data_service', RecordBagService, self.telemetry_data_service_cb)

        # Logitech Webcam
        webcam_bag_srv = rospy.Service('webcam_bag_service', RecordBagService, self.webcam_bag_service_cb)
        
        ## WIRIS thermal camera
        thermal_camera_capture_srv = rospy.Service('thermal_camera_capture_service', CameraCaptureService, self.thermal_camera_capture_service_cb)

        ## RGB camera
        rgb_camera_capture_srv = rospy.Service('rgb_camera_capture_service', CameraCaptureService, self.rgb_camera_capture_service_cb)


    ## Subscribers callbacks

    def laser_altitude_cb(self, data):
        self.current_laser_altitude = data.data

    def usb_cam_cb(self, image_msg):
        if self.webcam_record == True:
            rostime = Time()
            rostime.data = rospy.get_rostime()
            self.webcam_bag.write('rostime', rostime)
            self.webcam_bag.write('webcamImage', image_msg)

    def dji_gps_position_cb(self, gps_msg):

        if self.gps_record == True:
            rostime = Time()
            rostime.data = rospy.get_rostime()
            self.gpsA3_bag.write('rostime', rostime)
            self.gpsA3_bag.write('geoPosition', gps_msg)
            
        if self.telemetry_save:
            self.telemetry_file_writer.writerow({'Time':rospy.get_rostime(), 'Latitude':gps_msg.latitude, 'Longitude':gps_msg.longitude, 'Altitude':gps_msg.altitude, 'Laser_altitude':self.current_laser_altitude})



    ## Services callbacks

    def telemetry_data_service_cb(self, req):
        if req.record == True:
            self.telemetry_save = req.record
            telemetry_file = open(self.telemetry_files_dir + 'telemetry_{0}.csv'.format(self.start_mission_time.strftime("%Y-%m-%d %H:%M:%S")), 'a')
            columns = ['Time', 'Latitude', 'Longitude', 'Altitude', 'Laser_altitude']
            self.telemetry_file_writer = csv.DictWriter(telemetry_file, fieldnames=columns)
        elif req.record == False:
            self.telemetry_save = req.record
            # telemetry_file.close()
        return True

    def gpsA3_bag_service_cb(self, req):
        if req.record == True:
            self.gpsA3_bag = rosbag.Bag(self.mission_bags_dir +'/gpsA3_{0}.bag'.format(self.start_mission_time.strftime("%Y-%m-%d %H:%M:%S")), 'w')
            self.gps_record = req.record
        elif req.record == False:
            self.gps_record = req.record
            self.gpsA3_bag.close()         
        return True

    def webcam_bag_service_cb(self, req):
        if req.record == True:
            time_now = datetime.datetime.now()
            self.webcam_bag = rosbag.Bag(self.mission_bags_dir +'/webcam_{0}.bag'.format(time_now.strftime("%Y-%m-%d %H:%M:%S_")), 'w')
            self.webcam_record = req.record
        elif req.record == False:
            self.webcam_record = req.record
            self.webcam_bag.close()
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
        self.rgb_capture = req.capture
        if req.capture == True:
            rate = req.rate
            self.rgb_thread = threading.Thread(target=self.rgb_camera_capture_thread, args=(rate,))
            self.rgb_thread.start()
            rospy.loginfo ('starting RGB camera capture thread')

        if req.capture == False:
            rospy.loginfo("RGB camera capturing stopped")

        return True

    
    ## Cameras capture Threads

    def rgb_camera_capture_thread(self, rate):
            if self.rgb_capture == True:
                rgb_file = open(self.rgb_images_dir + 'rbg_images_{0}.csv'.format(self.start_mission_time.strftime("%Y-%m-%d %H:%M:%S")), 'a')
                # rgb_file = open(self.rgb_images_dir + 'rbg_images_.csv', 'a')
                columns = ['RGB Image', 'Time']
                self.rgb_file_writer = csv.DictWriter(rgb_file, fieldnames=columns)
                try:
                    gp.check_result(gp.use_python_logging())
                    self.rgb_camera = gp.check_result(gp.gp_camera_new())
                    gp.check_result(gp.gp_camera_init(self.rgb_camera))
                    rospy.loginfo('init RGB Camera')
                    try:
                        while self.rgb_capture == True:
                            # print('capturing')
                            self.rgb_camera_capture_function()
                            # print('captured')
                            rospy.sleep(1/rate)
                    except:
                        rospy.logerr("ERROR. Lost connection with RGB camera")
                    gp.check_result(gp.gp_camera_exit(self.rgb_camera))  
                    rospy.loginfo('exit camera RGB Camera')              
                except:
                    rospy.logerr("ERROR: RGB camera connection failed")
                rgb_file.close()
                

            gp.check_result(gp.gp_camera_exit(self.rgb_camera))  
            rospy.loginfo ('exit RGB camera capture thread')
            sys.exit()
        
    ## Capture functions

    def rgb_camera_capture_function(self):
        time_now = datetime.datetime.now()
        print('Capturing image')
        file_path = gp.check_result(gp.gp_camera_capture(self.rgb_camera, gp.GP_CAPTURE_IMAGE))
        # print('Camera file path: {0}/{1}'.format(file_path.folder, file_path.name))
        filename = 'image_{0}.jpg'.format(time_now.strftime("%Y-%m-%d %H:%M:%S"))
        target = os.path.join(self.rgb_images_dir, filename)
        print('Copying image to', target)
        camera_file = gp.check_result(gp.gp_camera_file_get(self.rgb_camera, file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL))
        gp.check_result(gp.gp_file_save(camera_file, target))
        
        rostime = Time()
        rostime.data = rospy.get_rostime()
        self.rgb_file_writer.writerow({'RGB Image':filename, 'Time':rospy.get_rostime()})

## end PAL Class ##




def main():
    rospy.init_node('PAL')
    pal = PAL()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
