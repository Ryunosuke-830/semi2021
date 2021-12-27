#!/usr/bin/env python
# license removed for brevity
from re import I
from rosgraph.xmlrpc import SilenceableXMLRPCRequestHandler
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

# useRightHand
global width
global height
width = 640
height = 480

global baseX
global baseY
global halfX
halfX = width/2.0
baseX = width/4.0
baseY = height/2.0

global N
N = 500


class final:
    def __init__(self):

        self.human_pose_sub = rospy.Subscriber('/edgetpu_human_pose_estimator/output/poses', PeoplePoseArray, self.callback)
        self.chat_pub = rospy.Publisher('chatter', String, queue_size=10);
        self.takeoff_pub = rospy.Publisher('tello/takeoff',Empty,queue_size=1)
        self.land_pub = rospy.Publisher('land', Empty, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('tello/cmd_vel',Twist, queue_size=10)
        self.R = 0.7
        self.totalTime = 10.0
        self.tw = Twist()
        time.sleep(1.0)
        self.setZero()
        self.takeoff_pub.publish()
        self.setZero()
        time.sleep(5.0)
        

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

    def circle(self):
        interval =  self.totalTime / float(N)
        theta = 2*math.pi / N
        self.setZero()
        for i in range(N):
            Ys = -self.R*math.sin(theta*i)*0.6
            Zs = self.R*math.cos(theta*i)*1.2
            self.setYZ(Ys, Zs)
            time.sleep(interval)
            self.setZero()
        self.setZero()
        time.sleep(1.0)


    def callback(self, msg):
        right_wrist = (False, 0)
        right_shoulder = (False, 0)
        right_elbow = (False, 0)
        left_wrist = (False,0)
        left_shoulder = (False,0)
        left_elbow = (False, 0)
        nose = (False, 0)

        if msg.poses:
            poses = msg.poses
            limb_names = poses[0].limb_names
            pose = poses[0].poses
            for i,item in enumerate(limb_names):
                if item == 'right wrist':
                    right_wrist = (True,i)
                elif item == 'right shoulder':
                    right_shoulder = (True,i)
                elif item == 'left wrist':
                    left_wrist = (True,i)
                elif item == 'left shoulder':
                    left_shoulder = (True,i)
                elif item == 'right elbow':
                    right_elbow = (True,i)
                elif item == 'left elbow':
                    left_elbow = (True,i)
                elif item == 'nose':
                    nose = (True,i)

            right_shoulder_pos = None
            right_wrist_pos = None
            left_shoulder_pos = None
            left_wrist_pos = None
            nose_pos = None
            left_elbow_pos = None
            right_elbow_pos = None

            if right_wrist[0]:
                right_wrist_pos = pose[right_wrist[1]].position
            if left_wrist[0]:
                left_wrist_pos = pose[left_wrist[1]].position
            if right_shoulder[0]:
                right_shoulder_pos = pose[right_shoulder[1]].position
            if left_shoulder[0]:
                left_shoulder_pos = pose[left_shoulder[1]].position
            if nose[0]:
                nose_pos = pose[nose[1]].position
            if right_elbow[0]:
                right_elbow_pos = pose[right_elbow[1]].position
            if left_elbow[0]:
                left_elbow_pos = pose[left_elbow[1]].position
            
            if left_elbow_pos.x > left_wrist_pos.x and right_elbow_pos.x < right_wrist_pos.x:
                if left_wrist_pos.y < left_elbow_pos.y and right_wrist_pos.y < right_elbow_pos.y:
                    str = 'OK'
                    rospy.loginfo_throttle(0.5,str)
                    self.circle()
                    
                    
                elif left_wrist_pos.y > left_elbow_pos.y and right_wrist_pos.y > right_elbow_pos.y:
                    str = 'naka'
                    rospy.loginfo_throttle(0.5, str)

            time.sleep(1.0)

if __name__ == '__main__':
    try:
        rospy.init_node('final', anonymous=True)
        teleop_node = final()
        rospy.spin()

    except rospy.ROSInterruptException: pass
