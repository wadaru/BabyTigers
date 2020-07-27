#!/usr/bin/env python
#
import moveit_commander
import rospy
import sys
import message_filters
import geometry_msgs.msg
from std_msgs.msg import Bool

plus_button = False
minus_button = False
function_button = False

# NOTE: Before start this program, please launch denso_cobotta_bring.launch

joints_name = ["joint_1", "joint_2",
               "joint_3", "joint_4", "joint_5", "joint_6"]


def is_plus_button_on(data):
    global plus_button
    plus_button = data.data

def is_minus_button_on(data):
    global minus_button
    minus_button = data.data

def is_function_button_on(data):
    global function_button
    function_button = data.data

def main():
    global plus_button
    global minus_button
    global function_button
    rospy.init_node("packing_pose")
    rospy.Subscriber('/cobotta/minus_button', Bool, is_plus_button_on)
    rospy.Subscriber('/cobotta/plus_button', Bool, is_minus_button_on)
    rospy.Subscriber('/cobotta/function_button', Bool, is_function_button_on)

    robot = moveit_commander.RobotCommander()
    print robot.get_group_names()
    # print robot.get_current_state()

    # print sys.argv
    arm = moveit_commander.MoveGroupCommander("arm")
    print("Reference frame: %s" % arm.get_planning_frame())
    print("Reference frame: %s" % arm.get_end_effector_link())

    while True:
        if (plus_button == True):
            print("PUSH Plus Button")
        if (minus_button == True):
            print("PUSH Minus Button")
        if (function_button == True):
            print("PUSH Function Button")
            arm   = moveit_commander.MoveGroupCommander("arm")
            print("arm_pose:" , arm.get_current_pose().pose)
            # joint_state = rospy.wait_for_message("/cobotta/joint_states")
            function_button == False
        if (plus_button == True and minus_button == True):
            print("PUSH both plus and minus Button")
            exit()
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
