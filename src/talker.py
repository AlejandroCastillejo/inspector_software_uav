#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import *

def talker():
    pub = rospy.Publisher('chatter', Bool, queue_size=10)
    rospy.init_node('talker')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        _start = True
        rospy.loginfo(_start)
        pub.publish(_start)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass