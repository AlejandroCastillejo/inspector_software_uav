#!/usr/bin/env python

import rospy
from inspector_software_uav.srv import RecordBagService, CameraCaptureService, DownloadService, ConnectionService


## PAL services Clients

def telemetry_data_client(record):
    try:
        rospy.wait_for_service('telemetry_data_service')
        telemetry_data = rospy.ServiceProxy('telemetry_data_service', RecordBagService)
        resp = telemetry_data(record)
        print resp.result
        return resp.result
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' %e)
        return False

def rgb_camera_connection_client():
    try:
        rospy.wait_for_service('rgb_camera_connection_service')
        rgb_camera_connection = rospy.ServiceProxy('rgb_camera_connection_service', ConnectionService)
        resp = rgb_camera_connection()
        return resp.result
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' %e)
        return False

def rgb_camera_capture_client(capture, shooting_dist):
    try:
        rospy.wait_for_service('rgb_camera_capture_service')
        rgb_camera_capture = rospy.ServiceProxy('rgb_camera_capture_service', CameraCaptureService)
        resp = rgb_camera_capture(capture, shooting_dist)
        return resp.result
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' %e)
        return False

def thermal_camera_connection_client():
    try:
        rospy.wait_for_service('thermal_camera_connection_service')
        thermal_camera_connection = rospy.ServiceProxy('thermal_camera_connection_service', ConnectionService)
        resp = thermal_camera_connection()
        return resp.result
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' %e)
        return False

def thermal_camera_capture_client(capture, shooting_dist):
    try:
        rospy.wait_for_service('thermal_camera_capture_service')
        thermal_camera_capture = rospy.ServiceProxy('thermal_camera_capture_service', CameraCaptureService)
        resp = thermal_camera_capture(capture, shooting_dist)
        return resp.result
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' %e)
        return False

def thermal_camera_download_client():
    try:
        rospy.wait_for_service('thermal_camera_download_service')
        thermal_camera_connection = rospy.ServiceProxy('thermal_camera_download_service', DownloadService)
        resp = thermal_camera_connection()
        return resp.result
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' %e)
        return False