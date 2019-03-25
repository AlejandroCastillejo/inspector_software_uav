#!/usr/bin/env python

import json
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import copy
import os.path

import rospy
# import rosmultimaster_launch
import smach
import smach_ros
import time
from std_msgs.msg import Float32, UInt8
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, TwistStamped

from inspector_software_uav.srv import *
from uav_abstraction_layer.srv import *
from uav_abstraction_layer.msg import *

# Global variables
stop_flag = False
flight_status = UInt8
current_pos = PointStamped()
# current_height = Float32
auto_download = False
loaded_mission = False

current_state = UInt8()

acept_radio = rospy.get_param('acept_radio', 1.2)

## Json files with mission information
mission_status_file = os.path.expanduser("~") + '/catkin_ws/src/inspector_software_uav/src/mission_status.json'
mission_waypoints_file = os.path.expanduser("~") + '/catkin_ws/src/inspector_software_uav/src/mission_wps.json'

print mission_waypoints_file

## PAL srv Clients ##

def telemetry_data_client(record):
    try:
        rospy.wait_for_service('telemetry_data_service')
        telemetry_data = rospy.ServiceProxy('telemetry_data_service', RecordBagService)
        resp = telemetry_data(record)
        return resp.result
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' %e)
        return resp.result

# def rgb_camera_capture_client(capture, interval):
#     try:
#         rospy.wait_for_service()


####################
## State Machine ##
###################

# define Standby state
class Standby_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_new_mission','to_paused_state','download_files'], input_keys = [], output_keys=[])
        self.start = False
    
    def execute(self, userdata):
        with open(mission_status_file) as f:
        # with open('mission_status.json') as f:
            status = json.load(f)
            paused_mission = status['paused_mission']
        if(paused_mission == 'True'):
            return 'to_paused_state'
        else:
            rospy.loginfo('UAV Ready. Waiting to receive order')
            self.stby_flag = False
            # global loaded_mission
            # loaded_mission = False
            self.action = 0

            def mission_service_cb(req):
                # userdata.H_d = req.h_d
                self.H_d = req.h_d
                userdata.MissionPath = req.MissionPath.poses
                MissionPath = req.MissionPath.poses
                print "recibido. h_d = {0}".format(req.h_d)
                print "recibido. MissionPath = {0}".format(MissionPath)
                ## add wayPoints to mission_wps.json
                path = []
                for pos in MissionPath:
                    wp = {"x": pos.pose.position.x, "y" : pos.pose.position.y, "z" : pos.pose.position.z}
                    path.append(wp)
                data = {"h_d": self.H_d, "path": path}
                print data
                with open(mission_waypoints_file,'w') as f:
                    json.dump(data, f)
                ##
                rospy.loginfo('Mission path received. Waiting for starting order')
                global loaded_mission
                loaded_mission = True
                return MissionServiceResponse(True)

            def stby_action_service_cb(req):
                self.action = req.stby_action
                return StbyActionServiceResponse(True)
           
            stby_action_srv = rospy.Service('stby_action_service', StbyActionService, stby_action_service_cb)
            # stby_action_srv = gcs_master.Service('stby_action_service', StbyActionService, stby_action_service_cb)
            mission_srv = rospy.Service('mission_service', MissionService, mission_service_cb)
            # mission_srv = gcs_master.Service('mission_service_0', MissionService, mission_service_cb)
            # while not self.stby_flag:
            while True:
                if self.action == 1:
                    # if loaded_mission:
                    if True:  ##test
                        #userdata.H_d = self.H_d
                        # self.stby_flag = True
                        stby_action_srv.shutdown()
                        mission_srv.shutdown()
                        status ={'paused_mission' : 'False'}
                        with open(mission_status_file, 'w') as f:
                            json.dump(status, f)
                        return 'start_new_mission'
                    else:
                        rospy.loginfo('No mission available. Please load mission path')
                        self.action = 0
                elif self.action == 2:
                    # self.stby_flag = True
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
        rospy.loginfo('Waiting for safe take off')
        rospy.sleep(1)
        return 'end_delay'

# Take Off 
class TakeOff_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['takeOff_finished','stop_mission'], input_keys = [], output_keys=['wp_00'])
        #self.end = False

    def execute(self, userdata):
        # while not stop_flag:
        print "test stop_flag"
        rospy.loginfo('Mission status: Waiting for takeOff Server')
        rospy.wait_for_service('ual/take_off')
        # self.H_d = userdata.H_d
        # takeOff_flag = False
        # while not takeOff_flag and not stop_flag:
        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        H_d = mission['h_d']

        # try:
        #     rospy.wait_for_service('telemetry_data_service')
        #     telemetry_data = rospy.ServiceProxy('telemetry_data_service', RecordBagService)
        #     resp_telemetry_data = telemetry_data(record)
        # except rospy.ServiceException, e:
        #     rospy.logerr('telemetry_data_service call failed: %s' %e)
            
        resp_telemetry = telemetry_data_client(True)
        if resp_telemetry:
            rospy.loginfo('Saving telemetry data')
        else:
            rospy.logwarn('Error saving telemetry data')
        
        # while True:
        try:
            take_off_client = rospy.ServiceProxy ('ual/take_off', TakeOff)
            resp_take_off = take_off_client(H_d, False)
            if resp_take_off:
                rospy.loginfo('Mission status: Taking Off')
            else:
                rospy.loginfo('Take Off Failed')

            while not(current_state == State.FLYING_AUTO):
            # while not(current_state == 4):
                # print("from msg",State.FLYING_AUTO)
                # print(current_state)
                if stop_flag:    # Mission manually stopped or battery low
                    return 'stop_mission' 
                # print 'taking off'
                rospy.sleep(0.1)
            # height_subscriber.unregister()
            rospy.sleep(1)
            # takeOff_flag = True
            rospy.loginfo("Take Off finished")
            # Define wp_00
            userdata.wp_00 = copy.deepcopy(current_pos)
            # print 'wp_00', userdata.wp_00
            return 'takeOff_finished'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        # print "trying to call service in 5s"
        # rospy.sleep(5)

# Go to wp_01 (First wp latitude and longitude but positioning height)
class GoToWp01_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_WayPoint_01','stop_mission'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Positioning... flying on safe height')
        # while True:
        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        target_wp = PoseStamped()
        target_wp.pose.position.x = mission['path'][0]['x']
        target_wp.pose.position.y = mission['path'][0]['y']
        target_wp.pose.position.z = mission['h_d']
        # rospy.wait_for_service('ual/go_to_waypoint')
        # go_to_waypoint_client = rospy.ServiceProxy ('ual/go_to_waypoint', GoToWaypoint)
        # resp = go_to_waypoint_client(target_wp, False)
        # self.current_wp = PointStamped()
        # def local_pos_cb(current_wp):
        #         self.current_wp = current_wp
        #         rospy.sleep(0.1)
        # local_pos_subscriber = rospy.Subscriber("dji_sdk/local_position", PointStamped, local_pos_cb, queue_size = 1)
        # x_left, y_left, z_left = 1, 1, 1
        # while math.sqrt(x_left**2 + y_left**2) > 0.1 and abs(z_left) >0.1:
        #     if stop_flag: 
        #         return 'stop_mission' # Mission manually stopped or battery low
        #     x_left = self.current_wp.point.x - target_wp.pose.position.x
        #     y_left = self.current_wp.point.y - target_wp.pose.position.y
        #     z_left = self.current_wp.point.z - target_wp.pose.position.z
        # # local_pos_subscriber.unregister()
        # return 'at_WayPoint_01'
        goToWaypoint_function(self, target_wp, True, True)
        if stop_flag:    # Mission manually stopped or battery low
            return 'stop_mission'
        print 'at wp 01'
        return 'at_WayPoint_01'
                


# Go to wp_1
class GoToWp1_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_WayPoint_1','stop_mission'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Possitioning... going to sweep height')
        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        target_wp = PoseStamped()
        target_wp.pose.position.x = mission['path'][0]['x']
        target_wp.pose.position.y = mission['path'][0]['y']
        target_wp.pose.position.z = mission['path'][0]['z']
        # print target_wp
        # rospy.wait_for_service('ual/go_to_waypoint')
        # go_to_waypoint_client = rospy.ServiceProxy ('ual/go_to_waypoint', GoToWaypoint)
        # resp = go_to_waypoint_client(target_wp, False)
        # self.current_wp = PointStamped()
        # def local_pos_cb(current_wp):
        #         self.current_wp = current_wp
        #         rospy.sleep(0.1)
        # local_pos_subscriber = rospy.Subscriber("dji_sdk/local_position", PointStamped, local_pos_cb, queue_size = 1)
        # x_left, y_left, z_left = 1, 1, 1
        # while math.sqrt(x_left**2 + y_left**2) > 0.1 and abs(z_left) >0.1:
        #     if stop_flag: 
        #         return 'stop_mission' # Mission manually stopped or battery low
        #     x_left = self.current_wp.point.x - target_wp.pose.position.x
        #     y_left = self.current_wp.point.y - target_wp.pose.position.y
        #     z_left = self.current_wp.point.z - target_wp.pose.position.z
        goToWaypoint_function(self, target_wp, True, False)
        if stop_flag:    # Mission manually stopped or battery low
            return 'stop_mission'
        print 'at wp 1'
        # local_pos_subscriber.unregister()
        return 'at_WayPoint_1'

# Sweep
class Sweep_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sweep_finished','stop_mission'], input_keys=[], output_keys=['wayPoints_left'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Sweep')
        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        wayPoints_left_ = copy.deepcopy(mission['path'])

        target_wp = PoseStamped()
        for pos in mission['path']:
            target_wp.pose.position.x = pos['x']
            target_wp.pose.position.y = pos['y']
            target_wp.pose.position.z = pos['z']

            goToWaypoint_function(self, target_wp, True, True)
            if stop_flag:    # Mission manually stopped or battery low
                wp = {"x": current_pos.point.x, "y" : current_pos.point.y, "z" : current_pos.point.z}
                wayPoints_left_.insert(0, wp)
                return 'stop_mission'
            wayPoints_left_.pop(0)
            userdata.wayPoints_left = wayPoints_left_
        status ={'paused_mission' : 'False'}
        with open(mission_status_file, 'w') as f:
            json.dump(status, f)
        return 'sweep_finished'
            


# Go to h_d
class GoToHd_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_h_d'])

    def execute(self,userdata):
        with open(mission_waypoints_file) as f:
            mission = json.load(f)
        H_d = mission['h_d']
        rospy.loginfo('Mission status: Going back Home')
        target_wp = PoseStamped()
        target_wp.pose.position.x = current_pos.point.x
        target_wp.pose.position.y = current_pos.point.y
        target_wp.pose.position.z = H_d
        goToWaypoint_function(self, target_wp, False, False)
        return 'at_h_d'


# Go to WP_00
class GoToWp00_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_WayPoint_00'], input_keys=['wp_00'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Going back Home')
        print 'wp_00', userdata.wp_00
        target_wp = PoseStamped()
        target_wp.pose.position.x = userdata.wp_00.point.x
        target_wp.pose.position.y = userdata.wp_00.point.y
        target_wp.pose.position.z = userdata.wp_00.point.z
        goToWaypoint_function(self, target_wp, False, True)
        return 'at_WayPoint_00'

# Land
class Land_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['landed'])

    def execute(self,userdata):
        rospy.wait_for_service('ual/land')
        land_client = rospy.ServiceProxy ('ual/land', Land)
        resp = land_client(False)
        if resp:
            rospy.loginfo('Mission status: Landing')
        else:
            rospy.loginfo('Land Failed')
        while flight_status != 0:
            rospy.sleep(0.1)
        return 'landed'


# Landed
class Landed_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['auto_download_on','auto_download_off','mission_paused'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Landed')
        if auto_download:
            return 'auto_download_on'
        else:
            return 'auto_download_off'


# SAVE CURRENT POSITION
class SaveCurrentPosition_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['current_position_saved'], input_keys=['wayPoints_left','H_d'], output_keys=[])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Saving Current Position')
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
        rospy.loginfo('Mission Status: PAUSED. Waiing to receive resume or cancel order')
        self.action = 0

        def paused_st_action_service_cb(req):
            self.action = req.paused_action
            return PausedStActionServiceResponse(True)
        # def paused_mission_server():
        paused_st_action_srv = rospy.Service('paused_state_action_service', PausedStActionService, paused_st_action_service_cb)

        # paused_st_action_srv = gcs_master.Service('paused_state_action_service', PausedStActionService, paused_st_action_service_cb)

        while True:
            if self.action == 1:
                rospy.loginfo('Canceling paused mission')
                status ={'paused_mission' : 'False'}
                with open(mission_status_file, 'w') as f:
                    json.dump(status, f)
                paused_st_action_srv.shutdown()
                return 'cancel_mission'
            elif self.action == 2:
                rospy.loginfo('Resuming paused mission')

                paused_st_action_srv.shutdown()
                return 'resume'
   

## FILES DOWNLOADING STATES

# Wait for connection 
class WaitingForConnection_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['connection_successful'])

    def execute(self,userdata):
        rospy.loginfo('Waiting for connection...')
        # topic connection ...
        rospy.loginfo('Connection Successful')

# Download
class FilesDownload_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['download_finished'])

    def execute(self,userdata):
        rospy.loginfo('Downloading')
        # topic goToWaypoint ...

# # 
# class (smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=[''])

#     def execute(self,userdata):
#         rospy.loginfo('Mission status: Sweep')
#         # topic goToWaypoint ...


## STOP MISSION SERVER
def stop_mission_cb(req):
    rospy.loginfo('Calling Stop Mission')
    global stop_flag
    stop_flag = True
    rospy.sleep(0.2)
    stop_flag = False
    return True

def stop_mission_server():
    stop_srv = rospy.Service('stop_service', StopService, stop_mission_cb)
    # stop_srv = gcs_master.Service('stop_service', StopService, stop_mission_cb)


#Publisher
set_velocity_pub = rospy.Publisher('ual/set_velocity', TwistStamped, queue_size=1)


##GO TO WAYPOINT FUNCTION

def goToWaypoint_function (self, target_wp, stop_on, rotation_on):
    rospy.loginfo('going to WayPoint: x:{0}, y:{1}, z:{2}'.format(target_wp.pose.position.x, target_wp.pose.position.y, target_wp.pose.position.z))
    rospy.wait_for_service('ual/go_to_waypoint')
    go_to_waypoint_client = rospy.ServiceProxy ('ual/go_to_waypoint', GoToWaypoint)

    if rotation_on and math.sqrt( (current_pos.point.x - target_wp.pose.position.x)**2 + (current_pos.point.y - target_wp.pose.position.y)**2) > acept_radio:
        yaw = math.atan2((target_wp.pose.position.y - current_pos.point.y),  (target_wp.pose.position.x - current_pos.point.x))
    else:
        yaw = current_yaw

    print 'yaw', yaw
    print 'current_yaw', current_yaw
    quat = quaternion_from_euler(0, 0, yaw)
    # print 'quaternion', quat

    o_wp = PoseStamped()
    o_wp.pose.position = current_pos.point
    o_wp.pose.orientation.x = quat[0]
    o_wp.pose.orientation.y = quat[1]
    o_wp.pose.orientation.z = quat[2]
    o_wp.pose.orientation.w = quat[3]
    print o_wp
    # go_to_waypoint_client(o_wp, False)
    # rospy.sleep(3)

    velocity = TwistStamped()
    while abs(yaw - current_yaw) > 0.1 :
        if (yaw - current_yaw) > 0:
            velocity.twist.angular.z = min (0.2, yaw - current_yaw)
            set_velocity_pub.publish(velocity)
        else:
            velocity.twist.angular.z = max (-0.2, yaw - current_yaw)
            set_velocity_pub.publish(velocity)

    o_wp.pose.position = target_wp.pose.position
    go_to_waypoint_client(o_wp, False)
    x_left, y_left, z_left = 1, 1, 1
    while math.sqrt(x_left**2 + y_left**2) > acept_radio or abs(z_left) > 0.5:
        # print '\n going to wp', target_wp.pose.position
        # print 'currrent', current_pos.point
        x_left = current_pos.point.x - target_wp.pose.position.x 
        y_left = current_pos.point.y - target_wp.pose.position.y 
        z_left = current_pos.point.z - target_wp.pose.position.z
        if stop_flag and stop_on:   # Mission manually stopped or battery low
        # if stop_mission_server():
            # global stop_flag
            # stop_flag = False
            # return 'stop_mission' 
            return
        # print 'stop_srv', stop_srv
        rospy.sleep(0.1)
    print 'at wp', target_wp.pose.position
    
def paused_st_action_service_cb(req):
        self.action = req.paused_action
        return PausedStActionServiceResponse(True)
def paused_mission_server():
    paused_st_action_srv = rospy.Service('paused_state_action_service', PausedStActionService, paused_st_action_service_cb)

## main
def main():
    rospy.init_node('ADL_state_machine')

     # gcs_master = rosmultimaster.Adaptor (host='192.168.1.2', port=11311, name='gcs_master', anonymous=True)

    # # Subscribe to local possiontion topic
    # def local_pos_cb(_current_pos):
    #     global current_pos
    #     current_pos = _current_pos
    #     rospy.sleep(0.05)
    # local_pos_subscriber = rospy.Subscriber("dji_sdk/local_position", PointStamped, local_pos_cb, queue_size = 1)
   
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


    # Subscribe to ual/state topic
    def ual_state_cb(state):
        global current_state
        current_state = state.state
        # rospy.loginfo('ual state: %s' %(current_state) )
    ual_state_subscriber = rospy.Subscriber("ual/state", State, ual_state_cb, queue_size=1)
    
    # # Subscribe to height topic
    # def height_cb(height):
    #     global current_height
    #     current_height = height.data
    #     rospy.sleep(0.1)
    # height_subscriber = rospy.Subscriber("dji_sdk/height_above_takeoff", Float32, height_cb, queue_size = 1)

    #Subscribe to flight status topic
    def flight_status_cb(status):
        global flight_status
        flight_status = status.data
        rospy.sleep(0.1)
    flight_status_subscriber = rospy.Subscriber("dji_sdk/flight_status", UInt8, flight_status_cb, queue_size=1)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])#'outcome4', 'outcome5'])
    # sm.userdata.wp_00 = PointStamped()


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
        # sm_pause_mission = smach.StateMachine(outcomes=['mission_paused','battery_low'])

        # with sm_pause_mission:

        #     smach.StateMachine.add('SAVE_CURRENT_POSITION',SaveCurrentPosition(),transitions={'current_position_saved':'GO_TO_H_d'})
        #     smach.StateMachine.add('GO_TO_H_d', GoToHd(), transitions={'at_h_d':'GO_TO_WP_00'})
        #     smach.StateMachine.add('GO_TO_WP_00', GoToWp00(), transitions={'at_WayPoint_00':'LAND'})
        #     smach.StateMachine.add('LAND', Land(), transitions={'landed':'LANDED'})
        #     smach.StateMachine.add('LANDED', Landed2(), transitions={'_mission_paused':'mission_paused','_battery_low':'battery_low'})

        # smach.StateMachine.add('MISSION_PAUSED',sm_pause_mission, transitions={'mission_paused':'PAUSE_STATE','battery_low':'BATTERY_CHANGING'})

        # smach.StateMachine.add('BATTERY_CHANGING', BatteryChanging(), transitions={'battery_changed':'PAUSE_STATE'})
        smach.StateMachine.add('PAUSE_STATE', Pause_State(), transitions={'resume':'MISSION','cancel_mission':'STANDBY'})


        # Create sub state machine for Files Downloading
        sm_files_downloading = smach.StateMachine(outcomes=['download_finished'])

        with sm_files_downloading:

            # Add states
            smach.StateMachine.add('WAITING_FOR_CONNECTION', WaitingForConnection_State(), transitions={'connection_successful':'DOWNLOAD'})
            smach.StateMachine.add('DOWNLOAD', FilesDownload_State(), transitions={'download_finished':'download_finished'})
        
        smach.StateMachine.add('FILES_DOWNLOADING', sm_files_downloading, transitions={'download_finished':'STANDBY'})

        

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
