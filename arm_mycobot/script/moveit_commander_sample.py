#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslib, sys
import moveit_commander
import math
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from jsk_recognition_msgs.msg import PeoplePoseArray


class MoveItPlanningDemo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("moveit_ik_demo")

        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)

        self.arm = moveit_commander.MoveGroupCommander("arm_group")

        self.end_effector_link = self.arm.get_end_effector_link()

        self.reference_frame = "link1"
        self.arm.set_pose_reference_frame(self.reference_frame)

        self.arm.allow_replanning(True)

        self.arm.set_goal_position_tolerance(0.05)
        self.arm.set_goal_orientation_tolerance(0.2)
        # 1 deg = 0.017 rad

        self.human_pose_sub = rospy.Subscriber('/edgetpu_human_pose_estimator/output/poses', PeoplePoseArray, self.callback)
        self.arm.set_named_target("init_pose")
        self.arm.go()
        rospy.sleep(2)

    def calcQuat(self,roll,pitch,yaw):
        deg =[roll,pitch,yaw] #radian
        cos = map(lambda x: math.cos(x/2.0), deg)
        sin = map(lambda x: math.sin(x/2.0), deg)
        q0 = cos[0] * cos[1] * cos[2] + sin[0] * sin[1] * sin[2]
        q1 = sin[0] * cos[1] * cos[2] - cos[0] * sin[1] * sin[2]
        q2 = cos[0] * sin[1] * cos[2] + sin[0] * cos[1] * sin[2]
        q3 = cos[0] * cos[1] * sin[2] - sin[0] * sin[1] * cos[2]
        return [q0, q1, q2, q3]

    def moveingProcess(self, x, y, z):
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        # quat = self.calcQuat(0,0,0)
        q = quaternion_from_euler(-math.pi/2, 0,-math.pi/2)
            #return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose,self.end_effector_link)
        traj = self.arm.plan()
        l = [x,y,z]
        # print(l)
        print(target_pose.pose)
        # print(traj)
        if len(traj.joint_trajectory.points) == 0:
            return
        self.arm.execute(traj)
        rospy.sleep(0.1)

    def shiftProcess(self, x, y, z):
        if (x == y == z == 0):
            return
        now_pose = self.arm.get_current_pose(self.end_effector_link)
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = now_pose.pose.position.x + x
        target_pose.pose.position.y = now_pose.pose.position.y + y
        target_pose.pose.position.z = now_pose.pose.position.z + z
        # quat = self.calcQuat(0,0,0)
        q = quaternion_from_euler(-math.pi/2, 0,-math.pi/2)
            #return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose,self.end_effector_link)
        traj = self.arm.plan()
        l = [x,y,z]
        # print(l)
        print(target_pose.pose)
        # print(traj)
        if len(traj.joint_trajectory.points) == 0:
            return
        self.arm.execute(traj)
        rospy.sleep(0.1)



    def callback(self, msg):
        right_wrist = (False, 0)
        left_wrist = (False, 0)

        if msg.poses:
            poses = msg.poses
            limb_names = poses[0].limb_names
            pose = poses[0].poses
            for i,item in enumerate(limb_names):
                if item == 'right wrist':
                    right_wrist = (True,i)
                elif item == 'left wrist':
                    left_wrist = (True,i)
            right_wrist_pos = None
            left_wrist_pos = None

            if right_wrist[0]:
                right_wrist_pos = pose[right_wrist[1]].position
            if left_wrist[0]:
                left_wrist_pos = pose[left_wrist[1]].position

        centerX = 320
        centerY = 240

        x = 0.0
        y = 0.0
        z = 0.0
        diff = 0.005

        if left_wrist_pos != None:
            x = diff if left_wrist_pos.y > centerY else -1*diff 
        
        if right_wrist_pos != None:
            y = diff if right_wrist_pos.x > centerX else -1*diff
            z = diff if right_wrist_pos.y < centerY else -1*diff

        self.shiftProcess(x,y,z)


        # self.moveingProcess(x, 0, 0.200)

        # self.moveingProcess(0.132,-0.150,0.075)

        return
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.132
        target_pose.pose.position.y = -0.150
        target_pose.pose.position.z = 0.075
        target_pose.pose.orientation.x = 0.026
        target_pose.pose.orientation.y = 1.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.014

        self.arm.set_start_state_to_current_state()

        self.arm.set_pose_target(target_pose, self.end_effector_link)

        traj = self.arm.plan()

        self.arm.execute(traj)
        rospy.sleep(1)

        self.arm.shift_pose_target(1, 0.12, self.end_effector_link)
        self.arm.go()
        rospy.sleep(1)

        self.arm.shift_pose_target(1, 0.1, self.end_effector_link)
        self.arm.go()
        rospy.sleep(1)

if __name__ == "__main__":
    o = MoveItPlanningDemo()
    rospy.spin()