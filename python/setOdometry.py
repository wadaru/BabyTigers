#!/usr/bin/env python
#
import rospy
import sys
import math
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

def getOdometry(data):
    global odometry
    odometry = data
    # print("get checkFlag Data:", checkFlag)
def getCheckFlag(data):
    global checkFlag
    checkFlag = data.data

def main():
    rospy.init_node("babyTigers")
    rospy.Subscriber('robotino/odometry', Pose, getOdometry)
    rospy.Subscriber('robotino/checkFlag', Bool, getCheckFlag)
    pub = rospy.Publisher('robotino/setOdometry', Pose, queue_size = 10)
    rate = rospy.Rate(10)
    global checkFlag
    checkFlag = False

    odometry = Pose()
    odometry.position.x = 1000
    odometry.position.y = 0
    odometry.orientation.z = -180

    while not (checkFlag == True or rospy.is_shutdown()):
      # rospy.loginfo(odometry)
      pub.publish(odometry)
      rate.sleep()
    while checkFlag == True:
      rate.sleep()
    
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
