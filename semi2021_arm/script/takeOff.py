#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Empty

def takeoff():
    pub = rospy.Publisher('tello/takeoff', Empty, queue_size=10);
    rospy.init_node('takeoff', anonymous=True)
    pub.publish()
        

if __name__ == '__main__':
    try:
        takeoff()
    except rospy.ROSInterruptException: pass       