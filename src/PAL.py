#!/usr/bin/env python

import rospy
import rosbag

import socket
import ftplib
import gphoto2 as gp
import threading
import sys, os, os.path
import csv
import time
import datetime

from std_msgs.msg import Float64, Int32, Time
from sensor_msgs.msg import Image, NavSatFix
from inspector_software_uav.srv import RecordBagService, CameraCaptureService, DownloadService, ConnectionService

# Thermal camera IP address & TCP port
WIRIS_IP = '10.0.2.228'
WIRIS_TCP_PORT = 2240
WIRIS_USER = 'wiris'
WIRIS_PASSWORD = ''
class PAL:

    def __init__(self):
        
        # Mission start time
        self.mission_start_time = datetime.datetime.now()

        ## 
        self.webcam_record = False
        self.gps_record = False
        self.telemetry_save = False
        self.current_laser_altitude = 0.0
        self.thermal_camera_connected = False
        
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
        
        if not os.path.exists(self.thermal_images_dir + 'logs/'):
            os.makedirs(self.thermal_images_dir + 'logs/')
        
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
        thermal_camera_connection_srv = rospy.Service('thermal_camera_connection_service', ConnectionService, self.thermal_camera_connection_service_cb)
        thermal_camera_capture_srv = rospy.Service('thermal_camera_capture_service', CameraCaptureService, self.thermal_camera_capture_service_cb)
        thermal_camera_download_srv = rospy.Service('thermal_camera_download_service', DownloadService, self.thermal_camera_download_service_cb)

        ## RGB camera
        rgb_camera_connection_srv = rospy.Service('rgb_camera_connection_service', ConnectionService, self.rgb_camera_connection_service_cb)
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
            telemetry_file = open(self.telemetry_files_dir + 'telemetry_{0}.csv'.format(self.mission_start_time.strftime("%Y-%m-%d %H:%M:%S")), 'a')
            columns = ['Time', 'Latitude', 'Longitude', 'Altitude', 'Laser_altitude']
            self.telemetry_file_writer = csv.DictWriter(telemetry_file, fieldnames=columns)
            rospy.loginfo('PAL: Saving telemetry data')
            return True
        elif req.record == False:
            self.telemetry_save = req.record
            rospy.loginfo('PAL: Stop saving telemetry data')
            # telemetry_file.close()
            return True

    def gpsA3_bag_service_cb(self, req):
        if req.record == True:
            self.gpsA3_bag = rosbag.Bag(self.mission_bags_dir +'/gpsA3_{0}.bag'.format(self.mission_start_time.strftime("%Y-%m-%d %H:%M:%S")), 'w')
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

    ## RGB Camera
    def rgb_camera_connection_service_cb(self, req):
        if not hasattr(self, 'rgb_camera'):
            try:
                gp.check_result(gp.use_python_logging())
                self.rgb_camera = gp.check_result(gp.gp_camera_new())
                gp.check_result(gp.gp_camera_init(self.rgb_camera))
                rospy.loginfo('PAL: init RGB Camera')
                return True
            except:
                rospy.logerr("PAL: RGB camera connection failed")
                return False

    def rgb_camera_capture_service_cb(self, req):
        self.rgb_interval = req.interval
        
        if req.capture == True:
            # if not hasattr(self, 'rgb_file'):
            print ('opening rgb_file')
            self.rgb_file = open(self.rgb_images_dir + 'rbg_images_{0}.csv'.format(self.mission_start_time.strftime("%Y-%m-%d %H:%M:%S")), 'a')
            columns = ['RGB Image', 'Time']
            self.rgb_file_writer = csv.DictWriter(self.rgb_file, fieldnames=columns)                   

            self.rgb_capture = True
            
            if not hasattr(self, 'rgb_camera_thread'):
                self.rgb_camera_thread = threading.Thread(target=self.rgb_camera_capture_thread)
                rospy.loginfo('PAL: created rgb_camera_thread')
                self.rgb_camera_thread.start()
            rospy.loginfo('PAL: starting RGB camera capture thread')

        elif req.capture == False:
            self.rgb_capture = False
            time.sleep(req.interval)
            if hasattr(self, 'rgb_file'):
                print ('closing rgb_file')
                self.rgb_file.close()
            rospy.loginfo("RGB camera capture stopped")
        return True

    def rgb_camera_capture_thread(self):
        while True:
            while self.rgb_capture == True:
                print ('calling rgb camera capture function')
                next_call = time.time() + self.rgb_interval
                self.rgb_camera_capture_function()
                if (next_call - time.time()) > 0:
                    time.sleep(next_call - time.time())
                else:
                    rospy.logwarn('PAL: RGB Capture time exceeded interval in %f seconds', abs((next_call - time.time())))
            time.sleep(0.1)
            
                # gp.check_result(gp.gp_camera_exit(self.rgb_camera))  
                # rospy.loginfo('PAL: exit camera RGB Camera')              
        # rgb_file.close()
            
        # gp.check_result(gp.gp_camera_exit(self.rgb_camera))  
        # rospy.loginfo('PAL: exit RGB camera capture thread')
        # sys.exit()

    def rgb_camera_capture_function(self):
        try:
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
        except:
            rospy.logerr("PAL: Error in connection with RGB camera")

    ## Thermal Camera
    def thermal_camera_connection_service_cb(self, req):
        if not hasattr(self,'s'):
            try:
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                rospy.loginfo('PAL: Created socked connection')
                self.s.connect((WIRIS_IP, WIRIS_TCP_PORT))
                rospy.loginfo('PAL: Socket connection on')
                self.s.send('HIWS')
                connect_resp = self.s.recv(3)
                if connect_resp == 'OK':
                    self.s.send('SPET')
                    rospy.loginfo("WIRIS Camera OK")
                    self.thermal_camera_connected = True
                else:
                    rospy.logerr("PAL: WIRIS Connection Failed")
                return True
            except:
                rospy.logerr("PAL: Can't open sockect connection with WIRIS")
                return False
        else:
            rospy.loginfo('PAL: Thermal camera already connected')
    

    def thermal_camera_capture_service_cb(self, req):
        self.thermal_camera_interval = req.interval

        if self.thermal_camera_connected:
            if req.capture == True:
                rospy.loginfo("WIRIS Camera starting secuence capture")
                self.thermal_camera_capture = True                  
                if not hasattr(self, 'thermal_camera_thread'):
                    self.thermal_camera_thread = threading.Thread(target=self.thermal_camera_capture_thread)
                    self.thermal_camera_thread.start()
            elif req.capture == False:
                self.thermal_camera_capture = False
                rospy.loginfo('PAL: Thermal camera capture stopped')
            return True
        else:
            rospy.logerr('PAL: Thermal camera not connected')


    def thermal_camera_capture_thread(self):
        while True:
            while self.thermal_camera_capture:
                interval = self.thermal_camera_interval
                next_call = time.time() + interval
                self.s.send('CIMG')
                capt_resp = self.s.recv(3)
                if capt_resp == 'OK':
                    rospy.loginfo('PAL: Capturing Thermal Image')
                else:
                    rospy.logerr("PAL: Can't capture Thermal Image")
                if (next_call - time.time()) > 0:
                    time.sleep(next_call - time.time())
                else:
                    rospy.logwarn('PAL: WIRIS Capture time exceeded interval in %f seconds', abs((next_call - time.time())))
            # rospy.loginfo('PAL: exit Thermal camera capture thread')
            time.sleep(0.1)
        # sys.exit()

    ## Thermal images download 
    def thermal_camera_download_service_cb(self, req):
        try:
            self.thermal_images_download()
            return True
        except Exception, e:
            rospy.logerr("PAL: Can't download files from thermal camera or files can't be deleted")
            rospy.logerr(e)
            return False
        # self.thermal_images_download()
        # return True
    
    def thermal_images_download(self):
        ftp = ftplib.FTP(WIRIS_IP)
        ftp.login(WIRIS_USER, WIRIS_PASSWORD)
        directorylist = ftp.nlst()
        rospy.loginfo('PAL: Downloading files from thermal camera')
        print('directory list: ', directorylist)
        for directory in directorylist:
            if directory == 'logs':
                print '\nlogs:'
                filenames = ftp.nlst(directory)
                print filenames
                for filename in filenames:
                    local_filename = self.thermal_images_dir + 'logs/' + filename
                    f = open(local_filename, 'wb')
                    ftp.retrbinary('RETR '+ 'logs/' + filename, f.write)
                    f.close()
                    print ('downloaded: ' + filename)
                    ftp.delete('logs/' + filename)
                    print ('deleted: ' + filename)
            else:
                subdirlist = ftp.nlst(directory)
                print('\ndirectory: ' + directory)
                print('subdirectories: ' + str(subdirlist))
                print('imgs:')
                img_dir = directory + '/img/'
                filenames = ftp.nlst(img_dir)
                for filename in filenames:
                    # local_filename = os.path.join(self.thermal_images_dir, filename)
                    local_filename = self.thermal_images_dir + filename
                    f = open(local_filename, 'wb')
                    ftp.retrbinary('RETR '+ img_dir + filename, f.write)
                    f.close()
                    print ('downloaded: ' + filename)
                    try:
                        ftp.delete(img_dir + filename)
                        print ('deleted: ' + filename)
                    except Exception, e:
                        print(e)
                try:
                    ftp.rmd(img_dir)
                    print('removed: ' + str(img_dir))
                except Exception, e:
                    print(e)
        rospy.loginfo('PAL: Download finished')
        rospy.loginfo('PAL: Deleting all files in WIRIS internal memory')
        for directory in directorylist:
            try:
                ftp.rmd(directory)
                print('removed directory:' + str(directory))
            except Exception, e:
                print(e)
        rospy.loginfo('PAL: Deletion finished')
        ftp.quit()
        

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
