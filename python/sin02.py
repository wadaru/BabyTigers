#!/usr/bin/env python
#
import rospy
import sys
import math
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool


def main():
    rospy.init_node("babyTigers")
    pub = rospy.Publisher('robotino/velocity', Float32MultiArray, queue_size = 10)
    rate = rospy.Rate(10)

    startDeg = 0
    endDeg = 360
    stepDeg = 1
    counter = 0
    for theta in range(startDeg, endDeg, stepDeg):
        x = math.sin(math.radians(theta)) * 300
        z = math.cos(math.radians(theta)) * 100
        velocity = Float32MultiArray()
        velocity.data = (x, 0, z)
        rospy.loginfo(velocity)
        pub.publish(velocity)
        rate.sleep()
    velocity.data= (0, 0, 0)
    rospy.loginfo(velocity)
    pub.publish(velocity)
    rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
