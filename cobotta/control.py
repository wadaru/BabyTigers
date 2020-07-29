#!/usr/bin/env python
#
import moveit_commander
import rospy
import sys
import message_filters
import geometry_msgs.msg
from std_msgs.msg import Bool
from denso_cobotta_driver.srv import GetMotorState
from denso_cobotta_driver.srv import SetMotorState
from denso_cobotta_driver.srv import SetBrakeState
from sensor_msgs.msg import JointState

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

def set_motor(status):
    rospy.wait_for_service("/cobotta/set_motor_state", 3.0)
    set_motor_state = rospy.ServiceProxy("/cobotta/set_motor_state", SetMotorState)
    m = set_motor_state(status)
    rospy.wait_for_service("/cobotta/set_brake_state", 3.0)
    set_brake_state = rospy.ServiceProxy("/cobotta/set_brake_state", SetBrakeState)
    b = set_brake_state([status, status, status, status, status, status])

def motor_on():
    set_motor(True)

def motor_off():
    set_motor(False)

def main():
    global plus_button
    global minus_button
    global function_button
    rospy.init_node("packing_pose")
    rospy.Subscriber('/cobotta/plus_button', Bool, is_plus_button_on)
    rospy.Subscriber('/cobotta/minus_button', Bool, is_minus_button_on)
    rospy.Subscriber('/cobotta/function_button', Bool, is_function_button_on)

    robot = moveit_commander.RobotCommander()
    print robot.get_group_names()
    # print robot.get_current_state()

    # print sys.argv
    arm = moveit_commander.MoveGroupCommander("arm")
    print("Reference frame: %s" % arm.get_planning_frame())
    print("Reference frame: %s" % arm.get_end_effector_link())

    while True:
        if (plus_button == True and minus_button == True):
            print("PUSH both plus and minus Button")
            set_motor(True)
            exit()
        if (plus_button == True):
            print("PUSH Plus Button")
            motor_on()
            arm   = moveit_commander.MoveGroupCommander("arm")
            print("arm_pose:" , arm.get_current_pose().pose)
        if (minus_button == True):
            print("PUSH Minus Button")
            motor_off()
        if (function_button == True):
            print("PUSH Function Button")
            arm   = moveit_commander.MoveGroupCommander("arm")
            print("arm_pose:" , arm.get_current_pose().pose)
            joint_state = rospy.wait_for_message("/cobotta/joint_states", JointState)
            if (joint_state.name == joints_name):
                print(joint_state.position)
            # function_button == False
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
