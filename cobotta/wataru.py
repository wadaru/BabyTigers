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
# Poses
#
joints_packing_old = [90, -60, 125, 90, -95, 0]
joints_packing_new = [90, -30, 120, -170, -94, 0]
joints_home = [0, 30, 100, 0, 50, 0]

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


def is_motor_running():
    rospy.wait_for_service('/cobotta/get_motor_state', 3.0)
    try:
        get_motor_state = rospy.ServiceProxy('/cobotta/get_motor_state',
                                             GetMotorState)
        res = get_motor_state()
        return res.state
    except rospy.ServiceException, e:
    # except rospy.SeriveException as e:
        print >> sys.stderr, "  Service call failed: %s" % e
        # print(" Service call failed: ", e, file=sys.stderr)

def is_simulation():
    service_list = rosservice.get_service_list()
    if '/cobotta/get_motor_state' in service_list:
        return False
    return True

def is_button_on(plus_button, minus_button, function_button, joint_state):
    # nothing
    if plus_button == 2:
        print "debug"

def is_button_on_mute(plus_button, minus_button, function_button, joint_state):
    if plus_button == True:
        print "plus: true,  ", 
        # print("plue: true,  ", end = "")
    else:
        print "plus: false, ", 
        # print("plue: false, ", end = "")
    if minus_button == True:
        print "minus: true,  ", 
        # print("minus: true,  ", end = "")
    else:
        print "minus: false, ", 
        # print("minus: false, ", end = "")
    if function_button == True:
        print("function: true,  ")
    else:
        print("function: false, ")
    print joint_state
    # print(joint_state)

if __name__ == '__main__':
    rospy.init_node("packing_pose")
    # rospy.Subscriber('/cobotta/plus_button', Bool, is_plus_button_on)
    sub1 = message_filters.Subscriber('/cobotta/plus_button', Bool)
    sub2 = message_filters.Subscriber('/cobotta/minus_button', Bool)
    sub3 = message_filters.Subscriber('/cobotta/function_button', Bool)
    sub4 = message_filters.Subscriber('/cobotta/joint_states', JointState)
    fps = 1
    delay = 1 / fps * 0.5
    ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3, sub4], 10, delay, allow_headerless = True)
    ts.registerCallback(is_button_on)


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
      recvADDRESS = "192.168.11.60"

    print("sendADD:", sendADDRESS, ", sendPORT:", sendPORT)
    print("recvADD:", recvADDRESS, ", recvPORT:", recvPORT)
    udp = udp_comm.udpcomm(sendADDRESS, sendPORT, recvADDRESS, recvPORT)
    while True:
      old = udp.view3Recv[0]
      udp.receiver()
      if (udp.view3Recv[0] == 0):
        break
      if (old != udp.view3Recv[0]):
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
      time.sleep(0.1)
    udp.closer()
    print("Bye...")
