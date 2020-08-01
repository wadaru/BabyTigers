#!/usr/bin/env python
#
import moveit_commander
import rospy
import sys
import math
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

joints_name = ["joint_1", "joint_2",
               "joint_3", "joint_4", "joint_5", "joint_6"]
jointData = []

def get_realsense_data(data):
    global findX, findY, targetDistance
    findX = data.data[0]
    findY = data.data[1]
    targetDistance = data.data[2]
    # print("findX = ", findX, ", findY = ", findY, ", target = ", targetDistance)

def main():
    global findX, findY, targetDistance
    rospy.init_node("packing_pose")
    
    robot = moveit_commander.RobotCommander()
    print robot.get_group_names()
    # print robot.get_current_state()

    # print sys.argv
    arm = moveit_commander.MoveGroupCommander("arm")
    print("Reference frame: %s" % arm.get_planning_frame())
    print("Reference frame: %s" % arm.get_end_effector_link())

    arm_initial_pose = arm.get_current_pose().pose
    print(arm_initial_pose)
    arm_initial_pose.position.x = 0.15 # 0.0290649014564
    arm_initial_pose.position.y = 0   # 0.0232106640527
    arm_initial_pose.position.z = 0.4 # 0.417222903858
    arm_initial_pose.orientation.x =  0
    arm_initial_pose.orientation.y =  1.0
    arm_initial_pose.orientation.z = 0
    arm_initial_pose.orientation.w =  0
    # print(arm_initial_pose)
    
    arm.set_pose_target(arm_initial_pose)
    arm.go(wait=True)

    target_pose_arm = geometry_msgs.msg.Pose()
    rospy.Subscriber('target', Float32MultiArray, get_realsense_data)
    findX = -1
    findY = -1
    oldX  = -1
    oldY  = -1

    while True:
        while(findX == oldX and findY == oldY):
            rospy.sleep(0.1)
        print("move")
        oldX = findX
        oldY = findY

        arm = moveit_commander.MoveGroupCommander("arm")
        arm_current_pose = arm.get_current_pose().pose
        
        target_pose_arm.position.x = arm_current_pose.position.x
        target_pose_arm.position.y = arm_current_pose.position.y
        target_pose_arm.position.z = arm_current_pose.position.z - (targetDistance - 250) / 1000
        target_pose_arm.orientation.x = arm_current_pose.orientation.x
        target_pose_arm.orientation.y = arm_current_pose.orientation.y
        target_pose_arm.orientation.z = arm_current_pose.orientation.z
        target_pose_arm.orientation.w = arm_current_pose.orientation.w

        arm.set_pose_target(target_pose_arm)
        arm.go(wait=True)

        rospy.sleep(0.1)
    rospy.sleep(3)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
