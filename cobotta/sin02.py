#!/usr/bin/env python
#
import moveit_commander
import rospy
import sys
import math
import geometry_msgs.msg
from sensor_msgs.msg import JointState

joints_name = ["joint_1", "joint_2",
               "joint_3", "joint_4", "joint_5", "joint_6"]
jointData = []

def main():
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
    arm_initial_pose.position.x = 0.1 # 0.0290649014564
    arm_initial_pose.position.y = 0   # 0.0232106640527
    arm_initial_pose.position.z = 0.2 # 0.417222903858
    arm_initial_pose.orientation.x =  0
    arm_initial_pose.orientation.y =  1.0
    arm_initial_pose.orientation.z = 0
    arm_initial_pose.orientation.w =  0
    # print(arm_initial_pose)

    target_pose_arm = geometry_msgs.msg.Pose()

    startDeg = 0
    endDeg = 360
    stepDeg = 21
    counter = 0
    for theta in range(startDeg, endDeg, stepDeg):
        x = math.sin(math.radians(theta)) / 12 + 0.1
        y = math.cos(math.radians(theta)) / 12 + 0.1
        target_pose_arm.position.x = arm_initial_pose.position.x + x
        target_pose_arm.position.y = arm_initial_pose.position.y
        target_pose_arm.position.z = arm_initial_pose.position.z + y
        target_pose_arm.orientation.x = arm_initial_pose.orientation.x
        target_pose_arm.orientation.y = arm_initial_pose.orientation.y
        target_pose_arm.orientation.z = arm_initial_pose.orientation.z
        target_pose_arm.orientation.w = arm_initial_pose.orientation.w

    
        # target_pose_arm.position.x = 0.0290649014564
        # target_pose_arm.position.y = 0.0232106640527
        # target_pose_arm.position.z = 0.417222903858
        # target_pose_arm.orientation.x = -0.663563908793
        # target_pose_arm.orientation.y =  0.287223508357
        # target_pose_arm.orientation.z =  0.65718144155
        # target_pose_arm.orientation.w =  0.212833615947
        print("sin theta = " , x, ", cos theta = ", y)
        arm.set_pose_target(target_pose_arm)

        arm.go()

        while True:
            joint_state = rospy.wait_for_message("/cobotta/joint_states", JointState)
            if (joint_state.name == joints_name):
                print(joint_state.position)
                jointData.append(joint_state.position)
                counter = counter + 6
                break

        rospy.sleep(0)
    print(jointData)
    print(counter)
    rospy.sleep(3)

    # r = rospy.Rate(1.3)
    for j in range(3):
        for i in range((counter / 6) - 1):
            pose_radian = jointData[i]
            print(i,": ", pose_radian)
            arm.go(pose_radian, wait=True)
            # rospy.sleep()
            # r.sleep()
    arm.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
