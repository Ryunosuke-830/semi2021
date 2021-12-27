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

global r
r = 1.0

class naka:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.tw = Twist()

        self.setZero()
        time.sleep(1.0)
        rospy.Publisher('tello/takeoff',Empty,queue_size=1).publish()
        self.setZero()
        time.sleep(5)
        self.pub()


    def setZero(self):
        self.tw.linear.x = 0.0
        self.tw.linear.y = 0.0
        self.tw.linear.z = 0.0
        self.tw.angular.x = 0.0
        self.tw.angular.y = 0.0
        self.tw.angular.z = 0.0
        self.cmd_vel_pub.publish(self.tw)
        self.cmd_vel_pub.publish(self.tw)
        self.cmd_vel_pub.publish(self.tw)
        time.sleep(1.0)

    def goHighLow(self, l):
        self.tw.linear.z = 0.3
        self.cmd_vel_pub.publish(self.tw)
        time.sleep(l)
        self.setZero()
    
    def goLR(self, l):
        self.tw.linear.y = 0.3
        self.cmd_vel_pub.publish(self.tw)
        time.sleep(l)
        self.setZero()
    
    def setOn(self):
        self.tw.linear.x = 0.05
        self.cmd_vel_pub.publish(self.tw)
        time.sleep(1.0)
        self.setZero()
    
    def setOff(self):
        self.tw.linear.x = -0.05
        self.cmd_vel_pub.publish(self.tw)
        time.sleep(1.0)
        self.setZero()

    def pub(self):
        R = 3*r
        time.sleep(1.0)
        #center
        self.goLR(-R/2.0)
        self.goHighLow(r/2.0)
        #1
        self.setOn()
        self.goHighLow(-r)
        self.setOff()
        self.goHighLow(r)
        #2,3
        self.setOn()
        self.goLR(R)
        self.goHighLow(r)
        self.setOff()
        #4
        self.goLR(-R)
        self.setOn()
        self.goLR(R)
        self.setOff()
        #5
        self.goLR(-R/2.0)
        self.goHighLow(2.0*r)
        self.setOn()
        self.goHighLow(-R)
        self.setOff()

        time.sleep(1.0)
        self.land_pub.publish()


        

 
if __name__ == '__main__':
    try:
        rospy.init_node('human_pose_teleop', anonymous=True)
        teleop_node = naka()

    except rospy.ROSInterruptException: pass