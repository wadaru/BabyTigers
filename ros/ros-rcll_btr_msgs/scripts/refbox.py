#!/usr/bin/python
import struct
import time
import sys
import rospy
import udpcomm
import rcll_ros_msgs
import rcll_btr_msgs
from rcll_ros_msgs.msg import GameState

def gameState(data):
    
    print(data.state)

    
def getRefBoxInfo():
    rospy.init_node('refbox')
    rospy.Subscriber("rcll/game_state", GameState, gameState)

    rospy.spin()
#
# main
#
if __name__ == '__main__':

  getRefBoxInfo()


