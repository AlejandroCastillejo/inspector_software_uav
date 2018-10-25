import rospy
import time
from std_msgs.msg import *

rospy.init_node('subs')




start = time.time()


def callback(Data):

    print Data

    time.sleep(0.5)

    
subs = rospy.Subscriber("chatter", Bool, callback)

while time.time() - start < 10:
    time.sleep(1)

print("fuera")

subs.unregister()


time.sleep(1000)