#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Empty

def land():
    pub = rospy.Publisher('tello/land', Empty, queue_size=10);
    rospy.init_node('land', anonymous=True)
    pub.publish()

if __name__ == '__main__':
    try:
        land()
    except rospy.ROSInterruptException: pass        