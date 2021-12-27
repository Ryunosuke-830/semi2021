#!/usr/bin/env python
# license removed for brevity
from re import I, T
import rospy
from rospy.topics import Publisher
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from jsk_recognition_msgs.msg import PeoplePoseArray
import math
import time

global canvasX
global canvasY
canvasX = 3
canvasY = 1.5

global N
N = 100

global TotalTime
TotalTime = 30.0

global R
R = 0.3

class circle:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.tw =  Twist()

        time.sleep(1.0)
        self.takeoff_pub.publish()
        self.setZero()
        time.sleep(5.0)
        self.pub()

    def setZero(self):
        self.tw.linear.x = 0.0
        self.tw.linear.y = 0.0
        self.tw.linear.z = 0.0
        self.tw.angular.x = 0.0
        self.tw.angular.y = 0.0
        self.tw.angular.z = 0.0
        self.cmd_vel_pub.publish(self.tw)

    def setX(self,x):
        self.tw.linear.x = x

    def setYZ(self, y,z):
        self.tw.linear.y = y
        self.tw.linear.z = z
        self.cmd_vel_pub.publish(self.tw)

    def pub(self):
        v = 0.6
        time.sleep(1.0)
        interval = TotalTime / 8.0
        self.setYZ(-v*0.5, 0)
        time.sleep(interval)
        self.setYZ(0,-v)
        time.sleep(interval)
        self.setYZ(v*0.5, 0)
        time.sleep(interval)
        self.setYZ(0,v)
        time.sleep(interval)
        self.setZero()
        time.sleep(1.0)
        

        self.land_pub.publish()

 
if __name__ == '__main__':
    try:
        rospy.init_node('human_pose_teleop', anonymous=True)
        teleop_node = circle()

    except rospy.ROSInterruptException: pass