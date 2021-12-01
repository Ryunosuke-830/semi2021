#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from geometry_msgs.msg import Twist

def control():
    pub = rospy.Publisher('tello/cmd_vel', Twist, queue_size=10);
    rospy.init_node('control', anonymous=True)
    """"
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 0.5
        pub.publish(msg)
        r.sleep()
    """
    time.sleep(1.0)
    msg = Twist()
    msg.linear.x = 0.5
    pub.publish(msg)

    time.sleep(1.0)

    msg.linear.x = 0
    pub.publish(msg)



if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException: pass        