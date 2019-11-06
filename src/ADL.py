#!/usr/bin/env python

import json
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import copy
import os.path

import rospy
import smach
import smach_ros
import time
import threading
from std_msgs.msg import Float32, UInt8, String
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, TwistStamped, Vector3Stamped
from sensor_msgs.msg import NavSatFix, BatteryState

from inspector_software_uav.srv import *
from uav_abstraction_layer.srv import *
from uav_abstraction_layer.msg import *

import PAL_clients as pal
import geo

# Global variables
stop_flag = False
auto_download = False
loaded_mission = False
flight_status = UInt8()
current_pos = PointStamped()
current_gps_pos = NavSatFix()
battery_state = BatteryState()
current_vel = Vector3Stamped()
ual_state = UInt8()
last_ual_state = UInt8()
adl_state = String()

## Json files with mission information
mission_status_file = os.path.expanduser("~") + '/catkin_ws/src/inspector_software_uav/json_files/mission_status.json'
mission_waypoints_file = os.path.expanduser("~") + '/catkin_ws/src/inspector_software_uav/json_files/mission_wps.json'
mission_data_file = os.path.expanduser("~") + '/catkin_ws/src/inspector_software_uav/json_files/mission_data.json'

print mission_waypoints_file

## PAL srv Clients ##

# # Define PAL Clients object
# pal = PALClients()

# uav_id = rospy.get_param('~uav_id', 'uav_1')
# acept_radio = rospy.get_param('~acept_radio', 1.2)
# rgb_images_on = rospy.get_param('~rgb_images_on', False)
# thermal_images_on = rospy.get_param('~thermal_images_on', False)
# stop_distance = rospy.get_param('~stop_distance', 10)


####################
## State Machine ##
###################

# define Standby state
class Standby_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_new_mission','to_paused_state','download_files'], input_keys = [], output_keys=[])
        self.start = False
    
    def execute(self, userdata):
        global adl_state
        adl_state = 'Standby'
        # register uav_id in gcs
        rospy.loginfo('waiting for uav_link service...')
        rospy.wait_for_service('/uav_link_service')
        try:
            rospy.loginfo('uav: %s calling uav_link_service', uav_id)
            uav_link_client = rospy.ServiceProxy('/uav_link_service', uavLink)
            resp = uav_link_client(uav_id)
        except rospy.ServiceException, e:
            rospy.loginfo("uav_link_service call failed: %s"%e)

        with open(mission_status_file) as f:
            status = json.load(f)
            paused_mission = status['paused_mission']
        if(paused_mission == 'True'):
            return 'to_paused_state'
        else:
            rospy.loginfo('ADL: UAV Ready. Waiting to receive order')
            # global loaded_mission
            # loaded_mission = False
            self.action = 0

            def mission_service_cb(req):
                # userdata.H_d = req.h_d
                H_d = req.h_d
                flight_angle = req.flight_angle
                orientation = req.orientation
                thermal_camera_shooting_distance = req.thermal_camera_shooting_distance
                rgb_camera_shooting_distance = req.rgb_camera_shooting_distance
                userdata.MissionPath = req.MissionPath.poses
                MissionPath = req.MissionPath.poses
                print "recibido. h_d = {0}".format(req.h_d)
                print "recibido. MissionPath = {0}".format(MissionPath)
                ## add wayPoints to mission_wps.json
                path = []
                path_geo = []
                lat_ref = current_gps_pos.latitude
                lon_ref = current_gps_pos.longitude
                h_ref = current_gps_pos.altitude
                for pos in MissionPath:
                    latitude = pos.pose.position.latitude
                    longitude = pos.pose.position.longitude
                    altitude = pos.pose.position.altitude
                    # wp = {"x": pos.pose.position.x, "y" : pos.pose.position.y, "z" : pos.pose.position.z}
                    wp_geo = {"latitude": latitude, "longitude" : longitude, "altitude" : altitude}
                    path_geo.append(wp_geo)
                    x,y,z = geo.geodetic_to_enu(latitude, longitude, altitude, lat_ref, lon_ref, h_ref)
                    wp = {"x": x, "y" : y, "z" : altitude}
                    path.append(wp)
                
                mission_waypoints_data = {"h_d": H_d, "path_geo": path_geo, "path": path}
                mission_data = {"thermal_camera_shooting_distance": thermal_camera_shooting_distance, \
                    "rgb_camera_shooting_distance": rgb_camera_shooting_distance, \
                    "flight_angle": flight_angle, "orientation": orientation}
                print mission_waypoints_data
                print mission_data
                with open(mission_waypoints_file,'w') as f:
                    json.dump(mission_waypoints_data, f)
                with open(mission_data_file, 'w') as f:
                    json.dump(mission_data, f)
                ##
                rospy.loginfo('ADL: Mission path received. Waiting for starting order')
                global loaded_mission
                loaded_mission = True
                return MissionServiceResponse(True)

            def stby_action_service_cb(req):
                self.action = req.stby_action
                return StbyActionServiceResponse(True)
           
            stby_action_srv = rospy.Service('stby_action_service', StbyActionService, stby_action_service_cb)
            mission_srv = rospy.Service('mission_service', MissionService, mission_service_cb)
            while True:
                if self.action == 1:
                    # if loaded_mission:
                    if True:  ##test
                        #userdata.H_d = self.H_d
                        stby_action_srv.shutdown()
                        mission_srv.shutdown()
                        status ={'paused_mission' : 'False'}
                        with open(mission_status_file, 'w') as f:
                            json.dump(status, f)
                        return 'start_new_mission'
                    else:
                        rospy.loginfo('ADL: No mission available. Please load mission path')
                        self.action = 0
                elif self.action == 2:
                    stby_action_srv.shutdown()
                    mission_srv.shutdown()
                    return 'download_files'
                time.sleep(1)
                

                
## MISSION STATES

# Delay
class Delay_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end_delay','stop_mission'])

    def execute(self,userdata):

        # Opening connection with cameras
        print ('acept_radio', acept_radio)
        print ('rgb_images_on', rgb_images_on)
        print ('thermal_images_on', thermal_images_on)
        
        if rgb_images_on:
            resp_rgb_connection = pal.rgb_camera_connection_client()
            if resp_rgb_connection:
                rospy.loginfo('ADL: RGB Camera connected')
            else:
                rospy.logwarn("ADL: Can't connect with RGB Camera")
        if thermal_images_on:
            resp_thermal_connection = pal.thermal_camera_connection_client()
            if resp_thermal_connection:
                rospy.loginfo('ADL: Thermal Camera connected')
            else:
                rospy.logwarn("ADL: Can't connect with Thermal Camera")

        rospy.loginfo('ADL: Waiting for safe take off')
        rospy.sleep(2)
        return 'end_delay'

# Take Off 
class TakeOff_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['takeOff_finished','stop_mission'], input_keys = [], output_keys=['wp_00'])
        #self.end = False

    def execute(self, userdata):
        global adl_state
        adl_state = 'Taking Off'
        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        H_d = mission['h_d']

        rospy.loginfo('ADL: Mission status: Waiting for takeOff Server')
        rospy.wait_for_service('ual/take_off')
            
        # Start recording telemetry data
        if record_telemetry:
            resp_telemetry = pal.telemetry_data_client(True)
            if resp_telemetry:
                rospy.loginfo('ADL: Recording telemetry data')
            else:
                rospy.logwarn('ADL: Error recording telemetry data')
        
        # Calling take off
        try:
            take_off_client = rospy.ServiceProxy ('ual/take_off', TakeOff)
            resp_take_off = take_off_client(H_d, False)
            if resp_take_off:
                rospy.loginfo('ADL: Mission status: Taking Off')
            else:
                rospy.loginfo('ADL: Take Off Failed')

            while not(ual_state == State.FLYING_AUTO):
                if stop_flag:    # Mission manually stopped or battery low
                    rospy.loginfo('ADL: Mission stopped... Waiting for take Off to be finished')
                    while not(ual_state == State.FLYING_AUTO):
                        rospy.sleep(1)
                    return 'stop_mission' 
                rospy.sleep(0.1)
            rospy.sleep(1)
            rospy.loginfo("Take Off finished")
            userdata.wp_00 = copy.deepcopy(current_pos)  # Define wp_00
            return 'takeOff_finished'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        # rospy.sleep(5)

# Go to wp_01 (First wp latitude and longitude but positioning height)
class GoToWp01_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_WayPoint_01','stop_mission'])

    def execute(self,userdata):
        global adl_state
        adl_state = 'Going to fist WP'
        rospy.loginfo('ADL: Mission status: Positioning... flying on safe height')

        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        target_wp = PoseStamped()
        target_wp.pose.position.x = mission['path'][0]['x']
        target_wp.pose.position.y = mission['path'][0]['y']
        target_wp.pose.position.z = mission['h_d']
      
        goToWaypoint_function(self, target_wp, True, True)
        if stop_flag:    # Mission manually stopped or battery low
            return 'stop_mission'
        rospy.loginfo('at wp 01')
               
        return 'at_WayPoint_01'
                

# Go to wp_1
class GoToWp1_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_WayPoint_1','stop_mission'])

    def execute(self,userdata):
        global adl_state
        adl_state = 'Going to sweep height'
        rospy.loginfo('ADL: Mission status: Possitioning... going to sweep height')
        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        target_wp = PoseStamped()
        target_wp.pose.position.x = mission['path'][0]['x']
        target_wp.pose.position.y = mission['path'][0]['y']
        target_wp.pose.position.z = mission['path'][0]['z']

        goToWaypoint_function(self, target_wp, True, False)
        if stop_flag:    # Mission manually stopped or battery low
            return 'stop_mission'

        ## Starting captures
        with open(mission_data_file) as f:
            mission_data = json.load(f)
            rgb_camera_shooting_distance = mission_data['rgb_camera_shooting_distance']
            thermal_camera_shooting_distance = mission_data['thermal_camera_shooting_distance']

        if rgb_images_on:
            resp_rgb_capture = pal.rgb_camera_capture_client(True, rgb_camera_shooting_distance)
            if resp_rgb_capture:
                rospy.loginfo('ADL: RGB Camera capturing')
            else:
                rospy.logwarn("ADL: Can't connect with RGB Camera")
        if thermal_images_on:
            resp_thermal_capture = pal.thermal_camera_capture_client(True, thermal_camera_shooting_distance)
            if resp_thermal_capture:
                rospy.loginfo('ADL: Thermal Camera capturing')
            else:
                rospy.logwarn("ADL: Can't connect with Thermal Camera")
        ##
        rospy.loginfo('at wp 1')
        
        return 'at_WayPoint_1'

# Sweep
class Sweep_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sweep_finished','stop_mission'], input_keys=[], output_keys=['wayPoints_left'])

    def execute(self,userdata):
        global adl_state
        adl_state = 'Sweep running'
        rospy.loginfo('ADL: Mission status: Sweep')
        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        wayPoints_left_ = copy.deepcopy(mission['path'])

        target_wp = PoseStamped()
        for pos in mission['path']:
            target_wp.pose.position.x = pos['x']
            target_wp.pose.position.y = pos['y']
            target_wp.pose.position.z = pos['z']

            goToWaypoint_function(self, target_wp, True, False)
            if stop_flag:    # Mission manually stopped or battery low
                wp = {"x": current_pos.point.x, "y" : current_pos.point.y, "z" : current_pos.point.z}
                wayPoints_left_.insert(0, wp)
                # goToWaypoint_function(self, current_wp, False, False)
                # rospy.sleep(1)
                return 'stop_mission'
            wayPoints_left_.pop(0)
            userdata.wayPoints_left = wayPoints_left_
        status ={'paused_mission' : 'False'}
        with open(mission_status_file, 'w') as f:
            json.dump(status, f)
        rospy.sleep(1)        
        return 'sweep_finished'
            
# Go to h_d
class GoToHd_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_h_d'])

    def execute(self,userdata):
        global adl_state
        adl_state = 'Going back home'
        ## Stop captures
        if rgb_images_on:
            resp_rgb_capture = pal.rgb_camera_capture_client(False, 1.0)
            if resp_rgb_capture:
                rospy.loginfo('ADL: RGB Camera capturing stopped')
            else:
                rospy.logwarn("ADL: Can't connect with RGB Camera")
        if thermal_images_on:
            resp_thermal_capture = pal.thermal_camera_capture_client(False, 1.0)
            if resp_thermal_capture:
                rospy.loginfo('ADL: Thermal Camera capturing stopped')
            else:
                rospy.logwarn("ADL: Can't connect with Thermal Camera")
    
        ## Stop UAV
        rospy.loginfo("ADL: Stopping UAV")

        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        current_wp = mission['path'][0]
        next_wp = mission['path'][1]
        x_ref = next_wp['x'] - current_wp['x']
        y_ref = next_wp['y'] - current_wp['y']
        r = math.sqrt(x_ref**2 + y_ref**2)
        stop_wp = PoseStamped()
        stop_wp.pose.position.x = current_wp['x'] + x_ref/r * min(stop_distance, r)
        stop_wp.pose.position.y = current_wp['y'] + y_ref/r * min(stop_distance, r)
        stop_wp.pose.position.z = current_pos.point.z
        print('current_wp: ', current_wp)
        print('stop_wp: ', stop_wp)
        goToWaypoint_function(self, stop_wp, False, False)
        rospy.sleep(1)
        ##

        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        H_d = mission['h_d']
        rospy.loginfo('ADL: Mission status: Going back Home')
        target_wp = PoseStamped()
        target_wp.pose.position.x = current_pos.point.x
        target_wp.pose.position.y = current_pos.point.y
        target_wp.pose.position.z = H_d
        goToWaypoint_function(self, target_wp, False, False)
        rospy.sleep(1)
        return 'at_h_d'


# Go to WP_00
class GoToWp00_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_WayPoint_00'], input_keys=['wp_00'])

    def execute(self,userdata):
        global adl_state
        adl_state = 'Going back home'
        ## Downloading thermal images
        if thermal_images_on:
            resp_thermal_images_download = pal.thermal_camera_download_client()
            if resp_thermal_images_download:
                rospy.loginfo('ADL: Downloading thermal images')
            else:
                rospy.logwarn("ADL: Can't download thermal images")
        ##

        rospy.loginfo('ADL: Mission status: Going back Home')
        print 'wp_00', userdata.wp_00
        target_wp = PoseStamped()
        target_wp.pose.position.x = userdata.wp_00.point.x
        target_wp.pose.position.y = userdata.wp_00.point.y
        target_wp.pose.position.z = userdata.wp_00.point.z
        goToWaypoint_function(self, target_wp, False, True)
        rospy.sleep(1)
        return 'at_WayPoint_00'

# Land
class Land_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['landed'])

    def execute(self,userdata):
        global adl_state
        adl_state = 'Landing'

        landing = True
        while landing and not rospy.is_shutdown():
            rospy.wait_for_service('ual/land')
            land_client = rospy.ServiceProxy ('ual/land', Land)
            resp = land_client(False)
            if resp:
                rospy.loginfo('ADL: Mission status: Landing')
                while ual_state == State.LANDING:
                    rospy.sleep(0.1)
                landing = False
            else:
                rospy.loginfo('ADL: Land Failed')
                rospy.sleep(5)
        return 'landed'

        # rospy.wait_for_service('ual/land')
        # land_client = rospy.ServiceProxy ('ual/land', Land)
        # resp = land_client(False)
        # if resp:
            # rospy.loginfo('ADL: Mission status: Landing')
        # else:
            # rospy.loginfo('ADL: Land Failed')
        # while flight_status != 0:
            # rospy.sleep(0.1)
        # return 'landed'


# Landed
class Landed_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['auto_download_on','auto_download_off','mission_paused'])

    def execute(self,userdata):
        global adl_state
        adl_state = 'Landed'
        rospy.loginfo('ADL: Mission status: Landed')
        
        ## Stop recording telemetry data
        if record_telemetry:
            resp_telemetry = pal.telemetry_data_client(False)
            if resp_telemetry:
                rospy.loginfo('ADL: Stop recording telemetry data')
            else:
                rospy.logwarn('ADL: Error at stop recording telemetry data')
        ## 
            
        if auto_download:
            return 'auto_download_on'
        else:
            return 'auto_download_off'


# SAVE CURRENT POSITION
class SaveCurrentPosition_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['current_position_saved'], input_keys=['wayPoints_left','H_d'], output_keys=[])

    def execute(self,userdata):
        rospy.loginfo('ADL: Mission status: Saving Current Position')
        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        H_d = mission['h_d']
        data = {"h_d": H_d, "path": userdata.wayPoints_left}
        print data
        with open(mission_waypoints_file,'w') as f:
            json.dump(data, f)
        status ={'paused_mission' : 'True'}
        with open(mission_status_file, 'w') as f:
            json.dump(status, f)

        print 'current position and wayPoints_left saved'
        return 'current_position_saved'


## PAUSE_STATE

class Pause_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['resume','cancel_mission'])

    def execute(self,userdata):
        global adl_state
        adl_state = 'Paused mission'
        rospy.loginfo('ADL: Mission Status: PAUSED. Waiing to receive resume or cancel order')
        self.action = 0

        def paused_st_action_service_cb(req):
            self.action = req.paused_action
            return PausedStActionServiceResponse(True)

        paused_st_action_srv = rospy.Service('paused_state_action_service', PausedStActionService, paused_st_action_service_cb)

        while True:
            if self.action == 1:
                rospy.loginfo('ADL: Canceling paused mission')
                status ={'paused_mission' : 'False'}
                with open(mission_status_file, 'w') as f:
                    json.dump(status, f)
                paused_st_action_srv.shutdown()
                return 'cancel_mission'
            elif self.action == 2:
                rospy.loginfo('ADL: Resuming paused mission')

                paused_st_action_srv.shutdown()
                return 'resume'
   

## FILES DOWNLOADING STATES

# Wait for connection 
class WaitingForConnection_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['connection_successful'])

    def execute(self,userdata):
        rospy.loginfo('ADL: Waiting for connection...')
        # topic connection ...
        rospy.loginfo('ADL: Connection Successful')

# Download
class FilesDownload_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['download_finished'])

    def execute(self,userdata):
        rospy.loginfo('ADL: Downloading')
        # topic goToWaypoint ...


## STOP MISSION SERVER
def stop_mission_cb(req):
    rospy.loginfo('ADL: Calling Stop Mission')
    global stop_flag
    stop_flag = True
    rospy.sleep(0.2)
    stop_flag = False
    return True

def stop_mission_server():
    stop_srv = rospy.Service('stop_service', StopService, stop_mission_cb)


#Publisher
set_velocity_pub = rospy.Publisher('ual/set_velocity', TwistStamped, queue_size=1)
set_pose_pub = rospy.Publisher('ual/set_pose', PoseStamped, queue_size=1)


##GO TO WAYPOINT FUNCTION

def goToWaypoint_function (self, target_wp, stop_on, heading_on):
    rospy.loginfo('ADL: going to WayPoint: x:{0}, y:{1}, z:{2}'.format(target_wp.pose.position.x, target_wp.pose.position.y, target_wp.pose.position.z))
    rospy.wait_for_service('ual/go_to_waypoint')
    go_to_waypoint_client = rospy.ServiceProxy ('ual/go_to_waypoint', GoToWaypoint)

    if heading_on and math.sqrt( (current_pos.point.x - target_wp.pose.position.x)**2 + (current_pos.point.y - target_wp.pose.position.y)**2) > acept_radio:
        yaw = math.atan2((target_wp.pose.position.y - current_pos.point.y),  (target_wp.pose.position.x - current_pos.point.x))
    else:
        with open(mission_data_file) as f:
            data = json.load(f)
            # orientation = data['flight_angle'] + 90
            orientation = -data['flight_angle']
            if orientation > 180:
                orientation -= 360
            elif orientation < -180:
                orientation +=360
            yaw = math.pi/180*orientation
            
    quat = quaternion_from_euler(0, 0, yaw)

    o_wp = PoseStamped()
    o_wp.pose.position = current_pos.point
    o_wp.pose.orientation.x = quat[0]
    o_wp.pose.orientation.y = quat[1]
    o_wp.pose.orientation.z = quat[2]
    o_wp.pose.orientation.w = quat[3]

    velocity = TwistStamped()
    while abs(yaw - current_yaw) > 0.01 :
        if (yaw - current_yaw) > 0:
            velocity.twist.angular.z = min (0.6, yaw - current_yaw)
            set_velocity_pub.publish(velocity)
        else:
            velocity.twist.angular.z = max (-0.6, yaw - current_yaw)
            set_velocity_pub.publish(velocity)
        # print ('yaw: ', yaw)
        # print ('current_yaw', current_yaw)
        rospy.sleep(0.1)

    o_wp.pose.position = target_wp.pose.position
    go_to_waypoint_client(o_wp, False)
    x_left, y_left, z_left = 1, 1, 1
    while math.sqrt(x_left**2 + y_left**2) > acept_radio or abs(z_left) > 0.5:
        x_left = current_pos.point.x - target_wp.pose.position.x 
        y_left = current_pos.point.y - target_wp.pose.position.y 
        z_left = current_pos.point.z - target_wp.pose.position.z

        if stop_flag and stop_on:   # Mission manually stopped or battery low 
            return
        rospy.sleep(0.1)
    print 'at wp', target_wp.pose.position
    
def paused_st_action_service_cb(req):
        self.action = req.paused_action
        return PausedStActionServiceResponse(True)
def paused_mission_server():
    paused_st_action_srv = rospy.Service('paused_state_action_service', PausedStActionService, paused_st_action_service_cb)


## 
## main
def main():
    rospy.init_node('adl')
    rospy.loginfo('starting ADL...')

    ## ROS params
    global uav_id, autopilot, acept_radio, rgb_images_on, thermal_images_on, stop_distance, record_telemetry
    uav_id = rospy.get_param('~uav_id', 'uav_1')
    autopilot = rospy.get_param('~autopilot', 'dji')
    acept_radio = rospy.get_param('acept_radio', 1.2)
    rgb_images_on = rospy.get_param('~rgb_images_on', False)
    thermal_images_on = rospy.get_param('~thermal_images_on', False)
    record_telemetry = rospy.get_param('~record_telemetry', False)
    stop_distance = rospy.get_param('~stop_distance', 10)

    # uav_id = rospy.get_param('uav_id', 1)

    # acept_radio = rospy.get_param(rospy.search_param('acept_radio'), 1.2)
    # rgb_images_on = rospy.get_param(rospy.search_param('rgb_images_on'), False)
    # thermal_images_on = rospy.get_param(rospy.search_param('thermal_images_on'), False)

    # rgb_images_interval = rospy.get_param('rgb_images_interval', 10.0)
    # thermal_images_interval = rospy.get_param('thermal_images_interval', 10.0)

    print ('uav_id', uav_id)
    print ('acept_radio', acept_radio)
    print ('thermal_images_on', thermal_images_on)
    print ('rgb_images_on', rgb_images_on)
    
    # Define ADL State publisher
    adl_state_pub = rospy.Publisher('adl_state', String, queue_size=10)

    # Define global position publisher
    global_pos_publisher = rospy.Publisher('global_position', NavSatFix, queue_size=10)
    battery_state_publisher = rospy.Publisher('battery_state', BatteryState, queue_size=1)

    # Subscribe to gps position
    def global_pos_cb(pose):
        global current_gps_pos
        current_gps_pos = pose
    
    def battery_cb(msg):
        global battery_state
        battery_state = msg
    
    if autopilot == 'dji':
        global_pos_subscriber = rospy.Subscriber("dji_sdk/gps_position", NavSatFix, global_pos_cb, queue_size=1)
        battery_subscriber = rospy.Subscriber("dji_sdk/battery_state", BatteryState, battery_cb, queue_size=1)
    elif autopilot == 'mavros':
        global_pos_subscriber = rospy.Subscriber("mavros/global_position/global", NavSatFix, global_pos_cb, queue_size=1)
   
    # Subscribe to ual/pose topic
    def local_pos_cb(pose):
        global current_pos
        current_pos.point = pose.pose.position
        rospy.sleep(0.05)
        global current_yaw
        q = pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        current_yaw = euler[2]
    local_pos_subscriber = rospy.Subscriber("ual/pose", PoseStamped, local_pos_cb, queue_size=1)

    # Subscribe to ual/velocity topic
    def velocity_cb(velocity):
        global current_vel
        current_vel = velocity
        rospy.sleep(0.05)
    velocity_subscriber = rospy.Subscriber("ual/velocity", TwistStamped, velocity_cb, queue_size=1)

    # Subscribe to ual/state topic
    def ual_state_cb(state):
        global ual_state
        global last_ual_state
        ual_state = state.state
        if ual_state != last_ual_state:
            rospy.loginfo('ADL: ual state: %s' %(ual_state))
        last_ual_state = ual_state
    ual_state_subscriber = rospy.Subscriber("ual/state", State, ual_state_cb, queue_size=1)
    
    # Subscribe to flight status topic
    def flight_status_cb(status):
        global flight_status
        flight_status = status.data
        rospy.sleep(0.1)
    flight_status_subscriber = rospy.Subscriber("dji_sdk/flight_status", UInt8, flight_status_cb, queue_size=1)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])#'outcome4', 'outcome5'])

    # Thread
    def main_thread():
        while not rospy.is_shutdown():
            adl_state_pub.publish(adl_state)
            global_pos_publisher.publish(current_gps_pos)
            battery_state_publisher.publish(battery_state)
            rospy.sleep(0.5)
    t = threading.Thread(target=main_thread)
    t.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('STANDBY',Standby_State(), 
                                transitions={'start_new_mission':'MISSION','to_paused_state':'PAUSE_STATE','download_files':'FILES_DOWNLOADING'})
        ## Create sub state machine for Mission
        sm_mission = smach.StateMachine(outcomes=['end_without_downloading','download_files','mission_paused'])
        
        # Open stop_mission server
        stop_mission_server()

        # Open the container
        with sm_mission:
            # Add states
            smach.StateMachine.add('DELAY', Delay_State(), transitions={'end_delay':'TAKE_OFF','stop_mission':'LANDED'})
            smach.StateMachine.add('TAKE_OFF', TakeOff_State(), transitions={'takeOff_finished':'GO_TO_WP_01','stop_mission':'LAND'}, remapping = {'wp_00':'wp_00'})
            smach.StateMachine.add('GO_TO_WP_01', GoToWp01_State(), transitions={'at_WayPoint_01':'GO_TO_WP_1','stop_mission':'GO_TO_WP_00'})
            smach.StateMachine.add('GO_TO_WP_1', GoToWp1_State(), transitions={'at_WayPoint_1':'SWEEP','stop_mission':'GO_TO_H_d'})
            smach.StateMachine.add('SWEEP', Sweep_State(), transitions={'sweep_finished':'GO_TO_H_d','stop_mission':'SAVE_CURRENT_POSITION'}, remapping={'wayPoints_left':'wayPoints_left'})
            smach.StateMachine.add('SAVE_CURRENT_POSITION',SaveCurrentPosition_State(),transitions={'current_position_saved':'GO_TO_H_d'}, remapping={'wayPoints_left':'wayPoints_left'})
            smach.StateMachine.add('GO_TO_H_d', GoToHd_State(), transitions={'at_h_d':'GO_TO_WP_00'})
            smach.StateMachine.add('GO_TO_WP_00', GoToWp00_State(), transitions={'at_WayPoint_00':'LAND'}, remapping = {'wp_00':'wp_00'})
            smach.StateMachine.add('LAND', Land_State(), transitions={'landed':'LANDED'})            
            smach.StateMachine.add('LANDED', Landed_State(), transitions={'auto_download_on':'download_files','auto_download_off':'end_without_downloading','mission_paused':'mission_paused'})
            # smach.StateMachine.add('', (), transitions={'':''})

        smach.StateMachine.add('MISSION',sm_mission, 
                                transitions={'download_files':'FILES_DOWNLOADING','end_without_downloading':'STANDBY','mission_paused':'PAUSE_STATE'}, 
                                remapping = {'H_d' : 'H_d', 'wayPoints_left' : 'wayPoints_left'})

        # ## Create sub state machine for Pause Mission
        smach.StateMachine.add('PAUSE_STATE', Pause_State(), transitions={'resume':'MISSION','cancel_mission':'STANDBY'})

        # Create sub state machine for Files Downloading
        sm_files_downloading = smach.StateMachine(outcomes=['download_finished'])

        with sm_files_downloading:
            # Add states
            smach.StateMachine.add('WAITING_FOR_CONNECTION', WaitingForConnection_State(), transitions={'connection_successful':'DOWNLOAD'})
            smach.StateMachine.add('DOWNLOAD', FilesDownload_State(), transitions={'download_finished':'download_finished'})
        
        smach.StateMachine.add('FILES_DOWNLOADING', sm_files_downloading, transitions={'download_finished':'STANDBY'})

        ## loginfo
        rospy.loginfo('ADL running')
      

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # while not rospy.is_shutdown():


    # Wait for ctrl-c to stop the application
    # rospy.spin()
    sis.stop()




if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        pass
