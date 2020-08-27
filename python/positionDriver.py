#!/usr/bin/env python
#
import rospy
import sys
import math
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

def getCheckFlag(data):
    global checkFlag
    checkFlag = data.data
    # print("get checkFlag Data:", checkFlag)

def main():
    rospy.init_node("babyTigers")
    rospy.Subscriber('robotino/checkFlag', Bool, getCheckFlag)
    pub = rospy.Publisher('robotino/positionDriver', Pose, queue_size = 10)
    rate = rospy.Rate(10)
    global checkFlag
    checkFlag = False

    while not rospy.is_shutdown():
      positionDriver = Pose()
      positionDriver.position.x = 1000
      positionDriver.position.y = 0
      positionDriver.orientation.z = 180

      while not (checkFlag == True or rospy.is_shutdown()):
        # rospy.loginfo(positionDriver)
        pub.publish(positionDriver)
        rate.sleep()
      while checkFlag == True:
        rate.sleep()
    
      positionDriver.position.x = 0
      positionDriver.position.y = 0
      positionDriver.orientation.z = 0
      while not (checkFlag == True or rospy.is_shutdown()):
        # rospy.loginfo(positionDriver)
        pub.publish(positionDriver)
        rate.sleep()
      while checkFlag == True:
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
