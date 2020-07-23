#
# Copyright (C) 2019  DENSO WAVE INCORPORATED
#
# -*- coding: utf-8 -*-
#
# usage: python ./packing_pose.py
#
#!/usr/bin/env python
import os
import sys
import rospy
import actionlib
import math
import moveit_commander
import rosservice
import message_filters
from geometry_msgs.msg import Pose, Point, Quaternion
from denso_cobotta_gripper.msg import GripperMoveAction, GripperMoveGoal
from denso_cobotta_driver.srv import GetMotorState
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

# from ...view3.udp.udp-comm import udpcomm 
# from udp-comm import udpcomm
import udp_comm
import time
# NOTE: Before start this program, please launch denso_cobotta_bring.launch

joints_name = ["joint_1", "joint_2",
               "joint_3", "joint_4", "joint_5", "joint_6"]

#
# Parallel gripper
#
gripper_parallel_open = 0.015
gripper_parallel_close = 0.0
gripper_parallel_speed = 100.0
gripper_parallel_effort = 20.0

def arm_move(move_group, joint_goal):
    pose_radian = [x / 180.0 * math.pi for x in joint_goal]
    # print (pose_radian)
    move_group.go(pose_radian, wait=True)
    move_group.stop()


def gripper_move(gripper_client, width, speed, effort):
    goal = GripperMoveGoal()
    goal.target_position = width
    goal.speed = speed
    goal.effort = effort
    gripper_client.send_goal(goal)

if __name__ == '__main__':
    rospy.init_node("packing_pose")

    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("arm")
    gripper_client = actionlib.SimpleActionClient('/cobotta/gripper_move',
                                                  GripperMoveAction)

    args = sys.argv
    if (len(args) == 5):
      sendADDRESS = args[1]
      sendPORT    = args[2]
      recvADDRESS = args[3]
      recvPORT    = args[4] 
    else:
      sendPORT = 9180
      recvPORT = 9182
      sendADDRESS = "127.0.1.1"
      # recvADDRESS = "127.0.1.1"
      # recvADDRESS = "192.168.11.60" # this machine's ip address.
      recvADDRESS = "10.42.0.1"

    print("sendADD:", sendADDRESS, ", sendPORT:", sendPORT)
    print("recvADD:", recvADDRESS, ", recvPORT:", recvPORT)
    udp = udp_comm.udpcomm(sendADDRESS, sendPORT, recvADDRESS, recvPORT)
    # joint_packing_recv = [0 for i in range(1, 7)]
    # oldData = joint_packing_recv
    while True:
      old = udp.view3Recv[0]
      udp.receiver()
      if (udp.view3Recv[0] == 0):
        break
      # joint_packing_recv = [udp.view3Recv[i] for i in range(1, 7)]
      if (old != udp.view3Recv[0]):
      # if (oldData != joint_packing_recv):
        print("time = ", udp.view3Recv[0])
        joint_packing_recv = [udp.view3Recv[i] for i in range(1, 7)]
        print("joint_packing_recv = ", joint_packing_recv)
        joints = joint_packing_recv
        gripper_width = gripper_parallel_close
        if (udp.view3Recv[7] == 1):
          gripper_width = gripper_parallel_open
        print("gripper ", gripper_width)
 
        gripper_move(gripper_client, gripper_width,
                     gripper_parallel_speed, gripper_parallel_effort)
        arm_move(move_group, joints)
        oldData = joint_packing_recv
      time.sleep(0.001)
    udp.closer()
    print("Bye...")
