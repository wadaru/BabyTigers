#!/usr/bin/python
import struct
import time
import sys
import rospy
import udpcomm
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Float32, Float32MultiArray, Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse
from nav_msgs.msg import Odometry
import rcll_ros_msgs
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity
#
# ROS for robotino
# 
def sendRobView():
    global checkFlag
    udp.sender()
    checkFlag = 0
    while checkFlag == 0:
        udp.receiver()
        checkFlag = float(udp.view3Recv[1])
        rate.sleep()
    robViewMode = 0
    udp.view3Send[ 4] = robViewMode
    udp.sender()
    rate.sleep()

def setVelocity(data):
    global velocityData, robViewMode
    print "setVelocity"
    resp = SetVelocity()
    velocityData = data
    robViewMode = 1
    udp.view3Send[ 4] = robViewMode # mode number
    udp.view3Send[ 8] = int(velocityData.pose.x)
    udp.view3Send[12] = int(velocityData.pose.y)
    udp.view3Send[16] = int(velocityData.pose.theta)

    print("header:", data.header)
    print("velocity data:", data.pose.x, data.pose.y, data.pose.theta)
    sendRobView()
    resp.ok = True
    # return resp
    return [resp.ok, ""]

def setPosition(data):
    global positionDriver, robViewMode
    resp = SetPosition()
    positionDriver = data
    robViewMode = 2
    udp.view3Send[ 4] = robViewMode # mode number
    udp.view3Send[ 8] = int(positionDriver.pose.x)
    udp.view3Send[12] = int(positionDriver.pose.y)
    udp.view3Send[16] = int(positionDriver.pose.theta)

    print("goToPosition:", positionDriver.pose)
    print(udp.view3Send[ 8])
    sendRobView()
    resp.ok = True
    return [resp.ok, ""]
    # print("setPosition:", positionDriver.position.x)

def setOdometry(data):
    global odometryData, robViewMode
    resp = SetBoolResponse()
    odometryData = data
    robViewMode = 3
    udp.view3Send[ 4] = robViewMode # mode number
    udp.view3Send[ 8] = int(odometryData.position.x)
    udp.view3Send[12] = int(odometryData.position.y)
    udp.view3Send[16] = int(odometryData.orientation.z)

    udp.sendRobView()
    resp.success = True
    return resp

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
  srv01 = rospy.Service('rvw2/setVelocity', SetVelocity, setVelocity)
  srv02 = rospy.Service('rvw2/positionDriver', SetPosition, setPosition)
  srv03 = rospy.Service('rvw2/setOdometry', SetOdometry, setOdometry)
  # pub01 = rospy.Publisher('odometry', Float32MultiArray, queue_size = 10)
  pub01 = rospy.Publisher('robotino/odometry', Odometry, queue_size = 10)
  pub02 = rospy.Publisher('robotino/checkFlag', Bool, queue_size = 10)
  pub03 = rospy.Publisher('robotino/getVelocity', Float32MultiArray, queue_size = 10)
  rate = rospy.Rate(10)

  velocityData = SetVelocity()
  velocityData.pose = [0, 0, 0]
  positionDriver = Pose()
  positionDriver.position.x = 0
  positionDriver.position.y = 0
  positionDriver.position.z = 0
  positionDriver.orientation = 0
  robViewMode = 0
  oldMode = 0
  checkFlag = 0

  udp.view3Send[ 4] = robViewMode
  udp.sender()

  # while True:
  while not rospy.is_shutdown():
    udp.receiver()
    # set publish data
    # odometry = Float32MultiArray()
    # odometry.data = (float(udp.view3Recv[1]) / 10, float(udp.view3Recv[2]) / 10, float(udp.view3Recv[3]) / 10)
    checkFlag = float(udp.view3Recv[1])
    getOdometry = Odometry()
    poseWithCovariance = PoseWithCovariance()
    point = Point()
    quaternion = Quaternion()
    pose = Pose()
    header = Header()
    point.x = float(udp.view3Recv[2]) / 10
    point.y = float(udp.view3Recv[3]) / 10
    point.z = float(udp.view3Recv[4]) / 10
    quaternion.x = 0
    quaternion.y = 0
    quaternion.z = 0
    quaternion.w = 0
    pose.position = point
    pose.orientation = quaternion
    poseWithCovariance.pose = pose
    poseWithCovariance.covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    header.seq = 1
    header.stamp = rospy.Time.now()
    header.frame_id = "odometry"
    getOdometry.header = header
    getOdometry.pose = poseWithCovariance
    velocity = Float32MultiArray()
    velocity.data = (float(udp.view3Recv[5]) / 10, float(udp.view3Recv[6]) / 10, float(udp.view3Recv[7]) / 10)

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
    if (checkFlag == 1):
      robViewMode = 0
      # velocity.data = (0, 0, 0)
      velocityData.pose = [0, 0, 0]

  udp.closer()


