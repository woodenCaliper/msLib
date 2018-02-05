#!/usr/bin/env python
#coding: utf-8

import b3mCtrl
import icsCtrl
class ctrlKondoServoClass:
	def __init__(self, _icsPort, _b3mPort):
		self.icsPort = _icsPort
		self.b3mPort = _b3mPort
		self.jointNameToIdDict = {}
		self.idToServoTypeDict = {}		#そのIDがB3MかICSかを記録
		self.idToB3mModeDict   = {}		#
		self.idToIcsLastSendPos = {}	#icsに最後に送った位置を記録、後にposを取得するときに、この位置を送信し、現在位置を取得する

	def __jointNameToId(self, jointName):
		if type(jointName) is not list:
			jointName = [jointName]
		idList = []
		for i in range(len(jointName)):
			idList.append(self.jointNameToIdDict[i])

	def beginB3m(self, idList, speed=1500000):
		self.B3m = b3mCtrl.B3mClass(self.b3mPort ,speed)
		self.B3m.begin()
		if type(idList) is not list:
			idList = [idList]
		for id in idList:
			self.idToServoTypeDict[id] = "B3M"
			self.B3m.setMode(id,"FREE")
			self.B3m.setTrajectoryType(id,"EVEN")
			self.B3m.setMode(id,"POSITION")
			self.idToB3mModeDict[id] = "POSITION"

	def beginIcs(self, idList, speed=115200):
		self.Ics = icsCtrl.IcsClass(self.icsPort , speed)
		self.Ics.begin()
		if type(idList) is not list:
			idList = [idList]
		for id in idList:
			self.idToServoTypeDict[id] = "ICS"
			self.__freeIcs(id)


	#サーボのid番号とそれに対応するjoint_name、B3MかICSかを記憶させる
	def learnJointName(self, jointName, id, B3MorICS):
		B3MorICS = B3MorICS.upper()	#大文字化
		if (B3MorICS != "B3M") and (B3MorICS != "ICS"):
			return False
		if (type(jointName) is list) and (type(id) is list):	#複数個設定だったら
			if len(jointName) != len(id):
				return False
			for i in range(len(jointName)):
				success = self.learnJointName(jointName[i], id[i], B3MorICS)
				if success is False:
					return False
		else:	#単数個設定だったら
			self.jointNameToIdDict[jointName] = id
			self.idToServoTypeDict[id] = B3MorICS
		return True

	#角度を直接与えて駆動させる
	def driveJoint(self, servo, angle, time=0):
		if (type(servo) is not list):
			servo = [servo]
		id=[]
		for i in range(len(servo)):
			if self.jointNameToIdDict.has_key(servo[i]):
				id.append( self.jointNameToIdDict[servo[i]] )
			else:
				id.append( servo[i] )
		if (type(angle) is not list):
			angle = [angle]
		if(len(servo) != len(angle)):
			return False

		for i in range(len(id)):
			if self.idToServoTypeDict.has_key(id[i]):
				if self.idToServoTypeDict[id[i]] == "B3M":
					self.__driveB3m(id[i], angle[i], time)
				elif self.idToServoTypeDict[id[i]] == "ICS":
					self.__driveIcs(id[i], angle[i])
		return True

	#sensor_msgs/JointStateを与えて駆動させる
	def driveJointState(self, jointState, sec=0):
		import time
		from sensor_msgs.msg import JointState
		if type(jointState) is not JointState:
			return False

		servoTypeList = []
		idList = []
		for name in jointState.name:
			if self.jointNameToIdDict.has_key(name):
				id = self.jointNameToIdDict[name]
			else:
				id = name
			servoType = self.idToServoTypeDict[id]
			servoTypeList.append(servoType)
			idList.append(id)

		b3mIdList = []
		b3mPosList = []
		icsIdList = []
		icsPosList = []
		for j in range(len(idList)):
			if servoTypeList[j] == "B3M":
				b3mIdList.append(idList[j])
				b3mPosList.append(jointState.position[j])
			elif servoTypeList[j] =="ICS":
				icsIdList.append(idList[j])
				icsPosList.append(jointState.position[j])

		self.__driveB3m(b3mIdList, b3mPosList, sec)
		self.__driveIcs(icsIdList, icsPosList)





	#moveit_msgs/RobotTrajectoryを与えて駆動させる
	def drivePlan(self, robotTrajectory):
		import time
		from moveit_msgs.msg import RobotTrajectory
		from trajectory_msgs.msg import JointTrajectory

		if type(robotTrajectory) is not RobotTrajectory:
			return False
		plan = robotTrajectory.joint_trajectory

		servoTypeList = []
		idList = []
		for name in plan.joint_names:
			if self.jointNameToIdDict.has_key(name):
				id = self.jointNameToIdDict[name]
			else:
				id = name
			servoType = self.idToServoTypeDict[id]
			servoTypeList.append(servoType)
			idList.append(id)

		funcTimeStart = time.time()
		for i in range(len(plan.points)):
			planTimeFromStart = plan.points[i].time_from_start.to_sec()

			b3mIdList = []
			b3mPosList = []
			icsIdList = []
			icsPosList = []
			for j in range(len(idList)):
				if servoTypeList[j] == "B3M":
					b3mIdList.append(idList[j])
					b3mPosList.append(plan.points[i].positions[j])
				elif servoTypeList[j] =="ICS":
					icsIdList.append(idList[j])
					icsPosList.append(plan.points[i].positions[j])

			sleepTime = ( planTimeFromStart - (time.time()-funcTimeStart) )

			if sleepTime>0:
				time.sleep( sleepTime )

			funcTimeFromStart = ( time.time() - funcTimeStart)
			servoTime = int( (planTimeFromStart) - funcTimeFromStart )
			if(servoTime<0):
				servoTime=0
			self.__driveB3m(b3mIdList, b3mPosList, servoTime)
			self.__driveIcs(icsIdList, icsPosList)

	def __driveB3m(self, id, rad, sec=0):
		# import time
		# start=time.time()
		if type(id) is not list:
			id = [id]
		for i in id:
			if self.idToB3mModeDict[i] != "POSITION":
				self.B3m.setMode(i,"POSITION")
		if type(rad) is not list:
			rad = [rad]
		posList = map(lambda x: self.B3m.radToPos(x), rad)
		self.B3m.positionCmd(id, posList, sec)
		# print time.time()-start

	def __driveIcs(self, ids, rad):
		if type(ids) is not list:
			ids = [ids]
		if type(rad) is not list:
			rad = [rad]
		if len(ids)==0 or len(rad)==0:
			return False
		posList = map(lambda x: self.Ics.radToPos(x), rad)
		for id in range(len(ids)):
			self.idToIcsLastSendPos[ids[id]] = posList[id]
			result = self.Ics.setPos(ids[id], posList[id])
			if result is False:
				print "id ", ids[id], " snyc error"

	def freeJoint(self, servo):
		if (type(servo) is not list):
			servo = [servo]
		id=[]
		for i in range(len(servo)):
			if self.jointNameToIdDict.has_key(servo[i]):
				id.append( self.jointNameToIdDict[servo[i]] )
			else:
				id.append( servo[i] )
		for i in range(len(id)):
			if self.idToServoTypeDict.has_key(id[i]):
				if self.idToServoTypeDict[id[i]] == "B3M":
					self.__freeB3m(id[i])
				elif self.idToServoTypeDict[id[i]] == "ICS":
					self.__freeIcs(id[i])
		return True

	def __freeB3m(self, id):
		if type(id) is not list:
			id = [id]
		for i in id:
			self.B3m.setMode(i,"FREE")
			self.idToB3mModeDict[i] = "FREE"

	def __freeIcs(self, id):
		if type(id) is not list:
			id = [id]
		for i in id:
			self.idToIcsLastSendPos[i] = 0
			self.Ics.setFree(i)

	def getPosJoint(self, servo):
		if (type(servo) is not list):
			servo = [servo]
		id=[]
		for i in range(len(servo)):
			if self.jointNameToIdDict.has_key(servo[i]):
				id.append( self.jointNameToIdDict[servo[i]] )
			else:
				id.append( servo[i] )
		rad = []
		for i in range(len(id)):
			if self.idToServoTypeDict.has_key(id[i]):
				if self.idToServoTypeDict[id[i]] == "B3M":
					rad.append( self.__getPosB3m(id[i]) )
				elif self.idToServoTypeDict[id[i]] == "ICS":
					rad.append( self.__getPosIcs(id[i]) )
		return rad

	def __getPosB3m(self, id):
		if type(id) is not list:
			id = [id]
		rad=[]
		for i in id:
			[pos, status] = self.B3m.getRam(i,"CurrentPosition")
			rad.append( self.B3m.posToRad(pos) )
		if len(rad)==1:
			return rad[0]
		else:
			return rad

	def __getPosIcs(self, ids):
		if type(ids) is not list:
			ids = [ids]
		rad=[]
		for id in ids:
			pos = self.Ics.setPos(id, self.idToIcsLastSendPos[id])
			rad.append( self.Ics.posToRad(pos) )
		if len(rad)==1:
			return rad[0]
		else:
			return rad