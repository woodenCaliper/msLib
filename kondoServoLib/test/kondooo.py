#!/usr/bin/env python
#coding: utf-8

# import rospy
import time, math
import sys

from kondoServoLib import ctrlKondoServo, b3mCtrl


from tttest.srv import jsjs
from tttest.srv import tjjs
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#from kondo_servo_lib import b3m
#from kondo_servo_lib import ics


def assemblyByte(data):
	reData=0
	if type(data) is not list:
		return False
	for i in range(len(data))[::-1]:	#rangeの反転
		reData += data[i]<<(8*i)
	return reData

def unsignedToSigned(bit, byteLength):
	if bit >= 1<<(8*byteLength-1):
		return -( (0b1<<(8*byteLength)) - bit )
	return bit


if __name__ == '__main__':
	# kServo = ctrlKondoServo.ctrlKondoServoClass(_icsPort="/dev/ttyUSB0", _b3mPort="/dev/ttyUSB1")
	# kServo.beginB3m([0])

	B3m = b3mCtrl.B3mClass("/dev/ttyUSB1" ,1500000)
	B3m.begin()
	idList = [0]
	if type(idList) is not list:
		idList = [idList]
	for id in idList:
		B3m.setMode(id,"FREE")
		B3m.setTrajectoryType(id,"EVEN")
		B3m.setMode(id,"POSITION")

	B3m.positionCmd([0], 18000, 1000)
	time.sleep(1.5)
	B3m.positionCmd([0], 00000, 1000)

	# leastMoveTime = 2.5/320

	# loopTime = 5
	# sendHz = 68
	# waitTime = 1.0/sendHz * 1000
	# loopNum = loopTime*sendHz
	# import random

	# rad = random.uniform(-320/180*math.pi,320/180*math.pi)
	# print "drive"
	# kServo.driveB3m(0, rad, 1)



	# print kServo.B3m.getRam(0, "CurrentPosition")

	# kServo.driveB3m(0, 0, 1)
	# time.sleep(2.5)
	# beforePos = 0

	# count=0
	# while True:
	# 	count+=1
	# 	deg = random.uniform(-320,320)
	# 	sumMovePos = abs(deg - beforePos)
	# 	beforePos = deg
	# 	rad = deg/180.0*math.pi

	# 	needMoveTime = leastMoveTime*sumMovePos
	# 	moveTime = random.uniform(needMoveTime, needMoveTime+0.5)
	# 	print count," ",deg," ", rad ," ", moveTime
	# 	kServo.driveB3m(0, rad, moveTime)

	# 	plusSleep = random.uniform(0, 0.1)
	# 	time.sleep(moveTime + plusSleep)

	# while True:
	# 	for k in range(loopNum):
	# 		theta = 2*math.pi/loopNum*k
	# 		sin = math.sin(theta)
	# 		pos = sin*math.pi

	# 		print theta," " ,sin," " ,pos, " ",k, " ",waitTime
	# 		start = time.time()
	# 		kServo.driveB3m(id, pos, waitTime)
	# 		print "funcEnd ", time.time()-start
	# 		# hoge = kServo.B3m.radToPos(pos)
	# 		# kServo.B3m.setRam(id, hoge, "DesiredPosition")
	# 		time.sleep(waitTime/1000)

	# global B3m
	# try:
	# 	B3m = b3mCtrl.B3mClass("/dev/ttyUSB0",1500000)
	# except:
	# 	print "error, port is not found"
	# 	sys.exit

	# B3m.begin()

	# for i in [14,15,16]:
	# 	B3m.setMode(i, "FREE")
	# 	B3m.setTrajectoryType(i,"EVEN")
	# 	B3m.setMode(i,"POSITION")
	# 	B3m.positionCmd(i, 0)

	# rospy.init_node("servo_server")
	# a = rospy.Service("setTrajectoryB3mSrv",tjjs, set_trajectory_srv)
	# print "ready to serve"
	# rospy.spin()
