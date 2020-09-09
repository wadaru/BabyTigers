#!/usr/bin/env python
#!/usr/bin/python3
import struct
import time
import sys
import rospy
import udpcomm
from geometry_msgs.msg import Pose
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

#
# ROS for robotino
# 
def setVelocity(data):
    global velocityData, robViewMode
    velocityData = data
    robViewMode = 0
    # print("velocityData:", velocityData.data[0])
def setPosition(data):
    global positionDriver, robViewMode
    positionDriver = data
    robViewMode = 1
    # print("setPosition:", positionDriver.position.x)
def setOdometry(data):
    global odometryData, robViewMode
    odometryData = data
    robViewMode = 2

#
# main
#
if __name__ == '__main__':
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
    recvADDRESS = "127.0.1.1"

  print("sendADD:", sendADDRESS, ", sendPORT:", sendPORT)
  print("recvADD:", recvADDRESS, ", recvPORT:", recvPORT)
  udp = udpcomm.Udpcomm(sendADDRESS, sendPORT, recvADDRESS, recvPORT)

  rospy.init_node('robotino')
  rospy.Subscriber('robotino/velocity', Float32MultiArray, setVelocity)
  rospy.Subscriber('robotino/positionDriver', Pose, setPosition)
  rospy.Subscriber('robotino/setOdometry', Pose, setOdometry)
  # pub01 = rospy.Publisher('odometry', Float32MultiArray, queue_size = 10)
  pub01 = rospy.Publisher('robotino/odometry', Pose, queue_size = 10)
  pub02 = rospy.Publisher('robotino/checkFlag', Bool, queue_size = 10)
  pub03 = rospy.Publisher('robotino/getVelocity', Float32MultiArray, queue_size = 10)
  rate = rospy.Rate(10)

  velocityData = Float32MultiArray()
  velocityData.data = (0, 0, 0)
  positionDriver = Pose()
  positionDriver.position.x = 0
  positionDriver.position.y = 0
  positionDriver.position.z = 0
  positionDriver.orientation = 0
  robViewMode = 0
  oldMode = 0
  checkFlag = False

  # while True:
  while not rospy.is_shutdown():
    udp.receiver()
    # set publish data
    # odometry = Float32MultiArray()
    # odometry.data = (float(udp.view3Recv[1]) / 10, float(udp.view3Recv[2]) / 10, float(udp.view3Recv[3]) / 10)
    getOdometry = Pose()
    getOdometry.position.x = float(udp.view3Recv[1]) / 10
    getOdometry.position.y = float(udp.view3Recv[2]) / 10
    getOdometry.orientation.z = float(udp.view3Recv[3]) / 10
    checkFlag = Bool()
    checkFlag.data =(bool(udp.view3Recv[4]))
    velocity = Float32MultiArray()
    velocity.data = (float(udp.view3Recv[5]) / 10, float(udp.view3Recv[6]) / 10, float(udp.view3Recv[7]) / 10)

    # get robot command
    udp.view3Send[ 4] = robViewMode # mode number
    if (robViewMode == 0): # omniDrive
      udp.view3Send[ 8] = int(velocityData.data[0])
      udp.view3Send[12] = int(velocityData.data[1])
      udp.view3Send[16] = int(velocityData.data[2])
    elif (robViewMode == 1): # positionDrive
      # DriveMode = 3, Turn|Drive|Turn Nonholonomic
      udp.view3Send[ 8] = int(positionDriver.position.x)
      udp.view3Send[12] = int(positionDriver.position.y)
      udp.view3Send[16] = int(positionDriver.orientation.z)
    elif (robViewMode == 2): # setOdometry
      udp.view3Send[ 8] = int(odometryData.position.x)
      udp.view3Send[12] = int(odometryData.position.y)
      udp.view3Send[16] = int(odometryData.orientation.z)
      # if (odometryData == getOdometry):
      if (odometryData.position.x == getOdometry.position.x and
          odometryData.position.y == getOdometry.position.y and
          (odometryData.orientation.z + 180) % 360 == 
          (getOdometry.orientation.z + 180) % 360):
        checkFlag.data = True
        # print("checkFlag: ", checkFlag.data)
      # print("odometryData:", odometryData.orientation.z % 360, " getOdometry:", getOdometry.orientation.z % 360)

    # rospy.loginfo(getOdometry)
    # rospy.loginfo(checkFlag)
    # rospy.loginfo(velocity)
    pub01.publish(getOdometry)
    pub02.publish(checkFlag)
    pub03.publish(velocity)

    if (robViewMode != oldMode):
      print("mode change from ", oldMode, " to ", robViewMode)
      oldMode = robViewMode

    udp.sender()
    # time.sleep(0.1)
    rate.sleep()
    if (checkFlag.data == True):
      robViewMode = 0
      # velocity.data = (0, 0, 0)
      velocityData.data = (0, 0, 0)
      print("Odometry = (", getOdometry.position.x, ",", getOdometry.position.y, ",", getOdometry.orientation.z)

  udp.closer()


