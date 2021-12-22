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
N = 100.0

global TotalTime
TotalTime = 30.0

class circle:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.R = canvasX*0.7/2.0
        self.tw =  Twist()

        time.sleep(1.0)
        rospy.Publisher('tello/takeoff',Empty,queue_size=1).publish()


    def setZero(self):
        self.tw.linear.x = 0.0
        self.tw.linear.y = 0.0
        self.tw.linear.z = 0.0
        self.tw.angular.x = 0.0
        self.tw.angular.y = 0.0
        self.tw.angular.z = 0.0

    def setX(self,x):
        self.tw.linear.x = x

    def setYZ(self, y,z):
        self.tw.linear.y = y
        self.tw.linear.z = z

    def pub(self):
        interval =  TotalTime / N
        theta = math.pi / N
        Ys = []
        Zs = []
        for i in range(N):
            Ys[i] = self.R*math.cos(theta)
            Zs[i] = self.R*math.sin(theta)
        time.sleep(1.0)
        for i in range(N):
            if (i == 0):
                y = Ys[i]
                z = Zs[i] - self.R
            else:
                y = Ys[i] - Ys[i-1]
                z = Zs[i] - Zs[i-1] 
            self.setYZ(y/interval, z/interval)
            self.cmd_vel_pub(self.tw)
            time.sleep(interval)
            self.setZero()
            self.cmd_vel_pub(self.tw)

 
if __name__ == '__main__':
    try:
        rospy.init_node('human_pose_teleop', anonymous=True)
        teleop_node = circle()

    except rospy.ROSInterruptException: pass