#!/usr/bin/env python

import rospy
import csv
import pandas
import os
from subprocess import call


class PostProcessing:

    def __init__(self):

        self.desktop = os.path.expanduser("~/Desktop")
        # call(["ls"])
        # self.bags_extraction()
        self.images_georef()

    # def bags_extraction(self):
    #     os.system("rosrun ros_csv_extraction extract_all -b " + self.desktop + "/mission_bags/webcam.bag -o " + self.desktop + "/mission_bags/webcam.csv")
    #     os.system("rosrun ros_csv_extraction extract_all -b " + self.desktop + "/mission_bags/gpsA3.bag -o " + self.desktop + "/mission_bags/gpsA3.csv")

    def images_georef(self):

        # images_file = pandas.read_csv(self.desktop + '/RGB_Camera_Images/rgb_images.csv')
        # telemetry_data_file = pandas.read_csv(self.desktop + '/Telemetry_Data/telemetry.csv')

        images_file = pandas.read_csv(self.desktop + '/GeoReferenced_Images/rgb_images_2019-06-13.csv')
        telemetry_data_file = pandas.read_csv(self.desktop + '/GeoReferenced_Images/telemetry_2019-06-13.csv')
        # row = 0
        # while True:
        images_file['Latitude'] = float()
        images_file['Longitude'] = float()
        images_file['Altitude'] = float()
        images_file['Laser_altitude'] = float()
        images_file['Quat.x'] = float()
        images_file['Quat.y'] = float()
        images_file['Quat.z'] = float()
        images_file['Quat.w'] = float()

        print images_file
        print telemetry_data_file
        i = 0
        j= 0
        for index in images_file.index:
            # i = 0
            while True:
                time_diff = abs(images_file['Time'][index] - telemetry_data_file['Time'][i])
                i += 1
                # print time_diff, 'time_diff'
                time_diff_2 = abs(images_file['Time'][index] - telemetry_data_file['Time'][i])
                if time_diff_2 > time_diff:
                    # print 'break'
                    # print time_diff_2, 'time_diff_2'
                    i -= 1
                    images_file['Latitude'][index] = telemetry_data_file['Latitude'][i]
                    images_file['Longitude'][index] = telemetry_data_file['Longitude'][i]
                    images_file['Altitude'][index] = telemetry_data_file['Altitude'][i]
                    images_file['Laser_altitude'][index] = telemetry_data_file['Laser_altitude'][i]
                    images_file['Quat.x'][index] = telemetry_data_file['Quat.x'][i]
                    images_file['Quat.y'][index] = telemetry_data_file['Quat.y'][i]
                    images_file['Quat.z'][index] = telemetry_data_file['Quat.z'][i]
                    images_file['Quat.w'][index] = telemetry_data_file['Quat.w'][i]

                    j += 1
                    # print 'i ', i
                    # print 'j ', j
                    break
            # print 'end while'
        print images_file                
        images_file.to_csv(self.desktop + '/GeoReferenced_Images/georefenenced_rgb_images_2019-06-13.csv')
          

       
def main():
    rospy.init_node('PostProcessing RGB Images')
    post_p = PostProcessing()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
