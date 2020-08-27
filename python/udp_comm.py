#!/usr/bin/python3
import struct
import time
import sys
import rospy
from geometry_msgs.msg import Pose
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

class udpcomm():
  def __init__(self, sADDRESS = "127.0.1.1", sPORT = 9180, rADDRESS = "127.0.1.1", rPORT = 9182):

    self.HOST = ''
    self.sendPORT = int(sPORT) # 9180
    self.recvPORT = int(rPORT) # 9182
    self.sendADDRESS = sADDRESS # "127.0.1.1"
    self.recvADDRESS = rADDRESS # "127.0.1.1"

    self.send = socket(AF_INET, SOCK_DGRAM)
    self.view3Send = struct.Struct("!BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
    self.view3Recv = struct.Struct("!BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
    #                           [00][01][02][03][04][05][06][07][08]
    self.view3Send = [00,00,00,00, 21,22,23,24, 31,32,33,34, 41,42,43,44,
                      51,52,53,54, 61,62,63,64, 71,72,73,74, 81,82,83,84]
    self.view3Recv = [00,00,00,00, 00,00,00,00, 00,00,00,00, 00,00,00,00,
                      00,00,00,00, 00,00,00,00, 00,00,00,00, 00,00,00,00]
    self.data = []


    # print("recvADDRESS: ", self.recvADDRESS, ", recvPORT: ", self.recvPORT)
    self.BUFSIZE = 1024
    self.recv = socket(AF_INET, SOCK_DGRAM)
    self.recv.bind((self.recvADDRESS, self.recvPORT))

  def sender(self):
    s = struct.Struct("!BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
    self.data =[00, 36, 00, 00];

    for i in range(4, 9 * 4, 4):
      if (self.view3Send[i - 4] < 0):
        self.view3Send[i - 4] = ((-self.view3Send[i - 4]) ^ 0xffffffff) + 1
      # if (self.view3Send[i - 4] > 255):
      self.view3Send[i - 1] = (self.view3Send[i - 4] & 0xff000000) >> 24
      self.view3Send[i - 2] = (self.view3Send[i - 4] & 0x00ff0000) >> 16
      self.view3Send[i - 3] = (self.view3Send[i - 4] & 0x0000ff00) >>  8
      self.view3Send[i - 4] = (self.view3Send[i - 4] & 0x000000ff) 

    for i in range(4, 9 * 4):
      # print(i)
      self.data.append(self.view3Send[i - 4]);

    self.data[3] = 0;
    checkSum = 0;
    for i in range(36):
      checkSum += self.data[i];
      if (checkSum > 0xff):
        checkSum -= 0x100;
    self.data[3] = (0xff - checkSum);

    # print(data)
    # print(len(data))
    tmpData = []
    for i in range(0, len(self.data), 4):
      tmpData.append(self.data[i + 0])
      tmpData.append(self.data[i + 1])
      tmpData.append(self.data[i + 2])
      tmpData.append(self.data[i + 3])
    self.data = tmpData
    sendData =s.pack(*self.data)
    self.send.sendto(sendData, (self.sendADDRESS, self.sendPORT))
    # print("send:", len(self.data), "(" , self.sendADDRESS, ",", self.sendPORT, ")", end="")
    # for i in range(len(self.data)):
    #   print(format(self.data[i], '02x'), end="")
    # print()

  def receiver(self):
    s = struct.Struct("!B")
    data, addr = self.recv.recvfrom(self.BUFSIZE)

    for i in range(8):
      self.view3Recv[i] = ((int(data[i * 4 + 4 + 0].encode('hex'), 16)      )
                         + (int(data[i * 4 + 4 + 1].encode('hex'), 16) <<  8)
                         + (int(data[i * 4 + 4 + 2].encode('hex'), 16) << 16)
                         + (int(data[i * 4 + 4 + 3].encode('hex'), 16) << 24))
      if (self.view3Recv[i] >= (1 << 31)):
        self.view3Recv[i]  -= (1 <<32)
      # print (self.view3Recv[i])

    # self.view3Send[0] = data[4] + 1;
    self.view3Send[0] = self.view3Recv[0] + 1;
    if (self.view3Send[0] > 255):
      self.view3Send[0] = 0

  def closer(self):
    self.send.close()

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
  udp = udpcomm(sendADDRESS, sendPORT, recvADDRESS, recvPORT)

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

  udp.closer()


