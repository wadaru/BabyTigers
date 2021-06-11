#!/usr/bin/python
import struct
import time
import sys
import rospy
import udpcomm
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, UInt32, String, Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse
from nav_msgs.msg import Odometry
import rcll_ros_msgs
import rcll_btr_msgs
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity
from rcll_ros_msgs.msg import BeaconSignal, ExplorationInfo, \
                              ExplorationSignal, ExplorationZone, GameState, \
                              LightSpec, MachineInfo, Machine, \
                              MachineReportEntry, MachineReportInfo, \
                              OrderInfo, Order, ProductColor, RingInfo, Ring, \
                              Team, Time
from rcll_ros_msgs.srv import SendBeaconSignal, SendMachineReport, \
                              SendPrepareMachine
#
# ROS for robotino
# 

def getResponse(value):
    while float(udp.view3Recv[1]) == value:
        udp.receiver()
        udp.sender()
        rate.sleep()
    return

def sendRobView():
    rovViewMode = udp.view3Send[1]
    
    # set mode = 0 and wait for ack(!=0) from RobView.
    udp.view3Send[1] = 0
    getResponse(0)

    # send command with mode
    udp.view3Send[1] = robViewMode
    getResponse(1)

    # command finished
    udp.view3Send[1] = 0
    getResponse(0)

    return udp.view3Recv[1]

def setVelocity(data):
    global velocityData, robViewMode
    print "setVelocity"
    resp = SetVelocity()
    velocityData = data
    robViewMode = 1
    udp.view3Send[1] = robViewMode # mode number
    udp.view3Send[2] = int(velocityData.pose.x)
    udp.view3Send[3] = int(velocityData.pose.y)
    udp.view3Send[4] = int(velocityData.pose.theta)

    print("header:", data.header)
    print("velocity data:", data.pose.x, data.pose.y, data.pose.theta)
    resp.ok = (sendRobView() == 1)
    # return resp
    return [resp.ok, ""]

def setPosition(data):
    global positionDriver, robViewMode
    resp = SetPosition()
    positionDriver = data
    robViewMode = 2
    udp.view3Send[1] = robViewMode # mode number
    udp.view3Send[2] = int(positionDriver.pose.x)
    udp.view3Send[3] = int(positionDriver.pose.y)
    udp.view3Send[4] = int(positionDriver.pose.theta)

    print("goToPosition:", positionDriver.pose)
    print(udp.view3Send[2])
    resp.ok = (sendRobView() == 1)
    return [resp.ok, ""]
    # print("setPosition:", positionDriver.position.x)

def setOdometry(data):
    global odometryData, robViewMode
    resp = SetBoolResponse()
    odometryData = data
    robViewMode = 3
    udp.view3Send[1] = robViewMode # mode number
    udp.view3Send[2] = int(odometryData.position.x)
    udp.view3Send[3] = int(odometryData.position.y)
    udp.view3Send[4] = int(odometryData.orientation.z)

    resp.ok = (sendRobView() == 1)
    return [resp.ok, ""]

#
# receive information from RefBox
#

def beaconSignal(data):
    global refboxBeaconSignal
    refboxBeaconSignal = data
    # print("BeaconSignal: ", data)

def explorationInfo(data):
    global refboxExplorationInfo
    refboxExplorationInfo = data
    print("ExplorationInfo: ", data)

def gameState(data):
    global refboxTime, refboxGameState
    refboxTime = data.game_time
    refboxGameState = data.state
    refboxGamePhase = data.phase
    refboxPointsMagenta = data.points_magenta
    refboxTeamMagenta = data.team_magenta
    refboxPointsCyan = data.points_cyan
    refboxTeamCyan = data.team_cyan
    # print("GameState: ", data)

def machineInfo(data):
    global refboxMachineInfo
    refboxMachineInfo = data
    print("MachineInfo: ", data)

def machineReportInfo(data):
    global refboxMachineReportInfo
    refboxMachineReportInfo = data
    print("MachineReportInfo: ", data)

def orderInfo(data):
    global refboxOrderInfo
    refboxOrderInfo = data
    print("OrderInfo: ", data)

def ringInfo(data):
    global refboxRingInfo
    refboxRingInfo = data
    print("RingInfo: ", data)

#
# send information to RefBox
#
def sendBeacon():
    beacon = SendBeaconSignal()
    header1 = Header()
    header2 = Header()
    poseStamped = PoseStamped()
    pose = Pose()
    
    point = Point()
    quaternion = Quaternion()
    point.x = float(udp.view3Recv[2]) / 10
    point.y = float(udp.view3Recv[3]) / 10
    point.z = 0 # float(udp.view3Recv[4]) / 10
    quaternion.x = 0
    quaternion.y = 0
    quaternion.z = 0
    quaternion.w = float(udp.view3Recv[4]) / 10
    pose.position = point
    pose.orientation = quaternion
    header1.seq = 1
    header1.stamp = rospy.Time.now()
    header1.frame_id = "BabyTigers"
    header2.seq = 1
    header2.stamp = rospy.Time.now()
    header2.frame_id = "robot1"
    poseStamped.header = header2
    poseStamped.pose = pose
    beacon.header = header1
    beacon.pose = poseStamped

    rospy.wait_for_service('/rcll/send_beacon')
    try:
        refboxSendBeacon = rospy.ServiceProxy('/rcll/send_beacon', SendBeaconSignal)
        resp1 = refboxSendBeacon(beacon.header, beacon.pose)
        print("sendBeacon: ", beacon.header, beacon.pose)
        print("resp: ", resp1)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

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

  # valiables for refbox
  refboxBeaconSignal = BeaconSignal()
  refboxExplorationInfo = ExplorationInfo()
  refboxExplorationSignal = ExplorationSignal()
  refboxExplorationZone = ExplorationZone()
  refboxGameState = Int8()
  refboxGamePhase = Int8()
  refboxPointsMagenta = UInt32()
  refboxTeamMagenta = String()
  refboxPointsCyan = UInt32()
  refboxTeamCyan = String()
  refboxLightSpec = LightSpec()
  refboxMachineInfo = MachineInfo()
  refboxMachine = Machine()
  refboxMachineReportEntry = MachineReportEntry()
  refboxMachineReportInfo = MachineReportInfo()
  refboxOrderInfo = OrderInfo()
  refboxOrder = Order()
  refboxProductColor = ProductColor()
  refboxRingInfo = RingInfo()
  refboxRing = Ring()
  refboxTeam = Team()
  refboxTime = Time()

  rospy.init_node('robotino')
  rospy.Subscriber("rcll/beacon", BeaconSignal, beaconSignal)
  rospy.Subscriber("rcll/exploration_info", ExplorationInfo, explorationInfo)
  rospy.Subscriber("rcll/game_state", GameState, gameState)
  rospy.Subscriber("rcll/machine_info", MachineInfo, machineInfo)
  rospy.Subscriber("rcll/machine_report_info", MachineReportInfo, machineReportInfo)
  rospy.Subscriber("rcll/order_info", OrderInfo, orderInfo)
  rospy.Subscriber("rcll/ring_info", RingInfo, ringInfo)
  
  rate = rospy.Rate(10)

  robViewMode = 0
  oldMode = 0
  checkFlag = 0

  udp.view3Send[1] = robViewMode
  udp.sender()

  # while True:
  while not rospy.is_shutdown():
    udp.receiver()
    sendBeacon()
    udp.sender()
    # time.sleep(0.1)
    rate.sleep()
    # if (checkFlag == 1):
    #   robViewMode = 0
    #   # velocity.data = (0, 0, 0)
    #   velocityData.pose = [0, 0, 0]

  udp.closer()


