#!/usr/bin/env python
# license removed for brevity
from re import I
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

global baseX
global baseY
baseX = 640/2.0
baseY = 480/2.0

class Position:
    def __init__(self):
        self.x = baseX
        self.y = baseY
        self.z = 0.0


def callback(msg):
    right_wrist = (False, 0)
    right_shoulder = (False, 0)
    left_wrist = (False,0)
    left_shoulder = (False,0)


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
        
        right_shoulder_pos = Position()
        right_wrist_pos = Position()
        left_shoulder_pos = Position()
        left_wrist_pos = Position()

        if right_wrist[0]:
            right_wrist_pos = pose[right_wrist[1]].position
        if left_wrist[0]:
            left_wrist_pos = pose[left_wrist[1]].position
        if right_shoulder[0]:
            right_shoulder_pos = pose[right_shoulder[1]].position
        if left_shoulder[0]:
            left_shoulder_pos = pose[left_shoulder[1]].position
        
        controlX = right_wrist_pos.x - baseX
        controlY = -(right_wrist_pos.y - baseY)
        

        '''
        if right_wrist_pos.y > right_shoulder_pos.y:
            pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
            pub.publish()

        if left_wrist_pos.y > left_shoulder_pos.y:
            pub = rospy.Publisher('tello/cmd_vel', Twist, queue_size=10)
            msg = Twist()
            msg.linear.y = 0.5
            pub.publish()
        '''
        pub1 = rospy.Publisher('tello/cmd_vel', Twist, queue_size=10)
        pub = rospy.Publisher('chatter', String, queue_size=10)

        time.sleep(1.0)
        str1 = "right: x: {}, y: {} ".format(controlX,controlY)
        str2 = "left: wristY={}, shoulderY={}".format(left_wrist_pos.y, left_shoulder_pos.y)
        str = str1+str2
        rospy.loginfo(str)
        pub.publish(str)
        
        '''
        tw = Twist()
        tw.linear.y = 0.0
        tw.linear.z = 0.0
        if(left_wrist_pos.y is not 0.0):
            tw.linear.y = controlX/100.0
            tw.linear.z = controlY/100.0
        
        pub1.publish(tw)
        time.sleep(1.0)
    '''


if __name__ == '__main__':
    try:
        rospy.init_node('controler', anonymous=True)
        rospy.Subscriber('/edgetpu_human_pose_estimator/output/poses', PeoplePoseArray, callback)
        rospy.spin()
        
    except rospy.ROSInterruptException: pass