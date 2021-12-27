#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import UInt8

def flip():
    pub = rospy.Publisher('tello/flip',UInt8, queue_size=1)
    rospy.init_node('flip', anonymous=True)
    time.sleep(1.0)
    pub.publish(0)
    time.sleep(1.0)



if __name__ == '__main__':
    try:
        flip()
    except rospy.ROSInterruptException: pass        