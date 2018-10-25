#!/usr/bin/env python

import json

import rospy
import smach
import smach_ros

from software_uav.srv import *

paused_mission = False
datos ={'paused_mission' : 'False',}
with open('file.json', 'w') as f:
    json.dump(datos, f)

# define Standby state
class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_new_mission','resume_mission','download_files'])
        self.start = False
    
    def execute(self, userdata):
        if(paused_mission):
            return 'resume_mission'
        else:
            rospy.loginfo('UAV Ready. Waiting to receive mission')
            rospy.wait_for_service('mission_service')
            try:
                mission_service = rospy.ServiceProxy('mission_service', MissionService)
                self.resp = mission_service()
                print self.resp.h_d
            except rospy.ServiceException, e:
                print "serv call failed"
            
            #if(self.resp.MissionReceived)
            

        # def callback(req):
        #     print req.h_d
        #     print 'callback'
        #     return MissionReceived(True)

        # s = rospy.Service('mission', MissionService, callback)
        # print s
        # # start = 'start topic'
        # # if self.start:
        # #     if path_ready:
        # #         return 'start_mission'
        # #     else:
        # #         rospy.loginfo('Cannot start mission. Not loaded Path')
        # return 'start_new_mission'
        rospy.loginfo('UAV Ready. Waiting to receive start order')
                
## MISSION STATES

# Delay
class Delay(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end_delay'])

    def execute(self,userdata):
        rospy.loginfo('Waiting for safe take off')
        rospy.sleep(1)
        return 'end_delay'

# Take Off 
class TakeOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['takeOff_finished'])
        #self.end = False

    def execute(self,userdata):
        rospy.loginfo('Mission status: Taking Off')
        # rostopic takeOff ...

        return 'takeOff_finished'


# Go to wp_01
class GoToWp01(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_WayPoint_01'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Possitioning... flying on safe height')
        # topic goToWaypoint ...

# Go to wp_1
class GoToWp1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_WayPoint_1'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Possitioning... going to sweep height')
        # topic goToWaypoint ...

# Sweep
class Sweep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sweep_finished'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Sweep')
        # topic goToWaypoint ...

# Go back home
class GoBackHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_WayPoint_01'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Going back Home')
        # topic goToWaypoint ...

# Land
class Land(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['landed'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Landing')
        # topic goToWaypoint ...
        # if _landed:
        #     rospy.loginfo('Mission status: Landed')
        #     return landed

# Landed
class Landed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['auto_download_on','auto_download_off'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Landed')


## MISSION PAUSED STATES

# Landed after stoping
class Landed2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['_mission_paused','_battery_low'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Landed')
   
# Save Curren Position
class SaveCurrentPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['current_position_saved'])

    def execute(self,userdata):
        rospy.loginfo('Mission status: Saving Current Position')

## Low battery
class BatteryChanging(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['battery_changed'])

    def execute(self,userdata):
        rospy.loginfo('Status: Changing Battery')

## PAUSE_STATE

class PauseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['resume','cancel_mission'])

    def execute(self,userdata):
        rospy.loginfo('Mission Status: PAUSED')

   

## FILES DOWNLOADING STATES

# Wait for connection 
class WaitingForConnection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['connection_successful'])

    def execute(self,userdata):
        rospy.loginfo('Waiting for connection...')
        # topic connection ...
        rospy.loginfo('Connection Successful')

# Download
class FilesDownload(smach.State):
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



# main
def main():
    rospy.init_node('ADL_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])#'outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('STANDBY',Standby(), transitions={'start_new_mission':'MISSION','resume_mission':'PAUSE_STATE','download_files':'FILES_DOWNLOADING'})
        
        ## Create sub state machine for Mission
        sm_mission = smach.StateMachine(outcomes=['end_without_downloading','download_files','stop_mission','low_battery'])

        # Open the container
        with sm_mission:

            # Add states
            smach.StateMachine.add('DELAY', Delay(), transitions={'end_delay':'TAKE_OFF'})
            smach.StateMachine.add('TAKE_OFF', TakeOff(), transitions={'takeOff_finished':'GO_TO_WP_01'})
            smach.StateMachine.add('GO_TO_WP_01', GoToWp01(), transitions={'at_WayPoint_01':'GO_TO_WP_1'})
            smach.StateMachine.add('GO_TO_WP_1', GoToWp1(), transitions={'at_WayPoint_1':'SWEEP'})
            smach.StateMachine.add('SWEEP', Sweep(), transitions={'sweep_finished':'GO_BACK_HOME'})
            smach.StateMachine.add('GO_BACK_HOME', GoBackHome(), transitions={'at_WayPoint_01':'LAND'})
            smach.StateMachine.add('LAND', Land(), transitions={'landed':'LANDED'})
            smach.StateMachine.add('LANDED', Landed(), transitions={'auto_download_on':'download_files','auto_download_off':'end_without_downloading'})

            
            # smach.StateMachine.add('', (), transitions={'':''})

        smach.StateMachine.add('MISSION',sm_mission, transitions={'download_files':'FILES_DOWNLOADING','end_without_downloading':'STANDBY','stop_mission':'MISSION_PAUSED','low_battery':'MISSION_PAUSED'})

        ## Create sub state machine for Pause Mission
        sm_pause_mission = smach.StateMachine(outcomes=['mission_paused','battery_low'])

        with sm_pause_mission:

            smach.StateMachine.add('SAVE_CURRENT_POSITION',SaveCurrentPosition(),transitions={'current_position_saved':'GO_BACK_HOME'})
            smach.StateMachine.add('GO_BACK_HOME', GoBackHome(), transitions={'at_WayPoint_01':'LAND'})
            smach.StateMachine.add('LAND', Land(), transitions={'landed':'LANDED'})
            smach.StateMachine.add('LANDED', Landed2(), transitions={'_mission_paused':'mission_paused','_battery_low':'battery_low'})

        smach.StateMachine.add('MISSION_PAUSED',sm_pause_mission, transitions={'mission_paused':'PAUSE_STATE','battery_low':'BATTERY_CHANGING'})

        smach.StateMachine.add('BATTERY_CHANGING', BatteryChanging(), transitions={'battery_changed':'PAUSE_STATE'})
        smach.StateMachine.add('PAUSE_STATE', PauseState(), transitions={'resume':'MISSION','cancel_mission':'STANDBY'})


        # Create sub state machine for Files Downloading
        sm_files_downloading = smach.StateMachine(outcomes=['download_finished'])

        with sm_files_downloading:

            # Add states
            smach.StateMachine.add('WAITING_FOR_CONNECTION', WaitingForConnection(), transitions={'connection_successful':'DOWNLOAD'})
            smach.StateMachine.add('DOWNLOAD', FilesDownload(), transitions={'download_finished':'download_finished'})
        
        smach.StateMachine.add('FILES_DOWNLOADING', sm_files_downloading, transitions={'download_finished':'STANDBY'})

        

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    # rospy.spin()
    sis.stop()







if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
