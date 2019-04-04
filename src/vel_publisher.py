#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import TwistStamped, Vector3Stamped

def main():
    rospy.init_node('vel_publisher')

    pub = rospy.Publisher('linear_velocity', TwistStamped, queue_size=10)

    def dji_vel_cb(data):
        vel = TwistStamped()
        vel.twist.linear = data.vector
        pub.publish(vel)
        time.sleep(0.01)

    dji_vel_subscriber = rospy.Subscriber("dji_sdk/velocity", Vector3Stamped, dji_vel_cb, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass