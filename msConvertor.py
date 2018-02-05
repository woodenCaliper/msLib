#!/usr/bin/env python
#coding: utf-8

from geometry_msgs.msg import Point, Vector3, Pose, PoseStamped, Transform, TransformStamped, Quaternion, QuaternionStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import RobotTrajectory, RobotState

##
# @file hoge.python
# @brief setumei

## @package pyexample
#  Documentation for this module.
#
#  More details.
# @author sugino
# @version 1.1
# @since 2019/02/02


## 型変換の関数
# 対応型Point, Vecotr3, Quaternion, QuaternionStamped
# Pose, PoseStamped, Transform, TransformStamped
# JointState, JointTrajectoryPoint, JointTrajectory, RobotTrajectory
# tuple, list
# @param hoge
def msTypeConvert(data, toType):
	"""Documentation for a function.

	More details.
	"""
	re = None
	fromType = type(data)
	if (fromType is not Point) and (fromType is not Vector3) and (fromType is not Quaternion) and (fromType is not QuaternionStamped):
		if (fromType is not Pose) and (fromType is not PoseStamped) and (fromType is not Transform) and (fromType is not TransformStamped):
			if (fromType is not JointState) and (fromType is not JointTrajectoryPoint) and (fromType is not JointTrajectory) and (fromType is not RobotTrajectory):
				if (fromType is not RobotState):
					if (fromType is not tuple) and (fromType is not list):
						print "cannot comvert from", fromType, "to", toType
						return False
	if toType is fromType:
		return data
	elif toType is list:
		if   fromType is tuple:
			re = list(data)

		elif fromType is Point:
			re = [data.x, data.y, data.z]
		elif fromType is Vector3:
			re = [data.x, data.y, data.z]

		elif fromType is Quaternion:
			re = [data.x, data.y, data.z, data.w]
		elif fromType is QuaternionStamped:
			re = msTypeConvert(data.quaternion, list)

		elif fromType is Pose:
			re = [data.position.x, data.position.y, data.position.z,
					data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
		elif fromType is PoseStamped:
			re = msTypeConvert(data.pose, list)
		elif fromType is Transform:
			re = [data.translation.x, data.translation.y, data.translation.z,
					data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
		elif fromType is TransformStamped:
			re = msTypeConvert(data.transform, list)

		elif fromType is JointState:
			re = []
			for i in range(len(data.position)):
				re.append(data.position[i])
		elif fromType is JointTrajectoryPoint:
			re = []
			for i in range(len(data.positions)):
				re.append( positions[i] )
		elif fromType is JointTrajectory:
			re = []
			time = []
			for i in range(len(data.points)):
				re.append(msTypeConvert( data.points[i].positions, list) )
				time.append( data.points[i].time_from_start.to_sec() )
			return re,time
		elif fromType is RobotTrajectory:
			return msTypeConvert( data.joint_trajectory, list )
		elif fromType is RobotState:
			js = msTypeConvert(data, JointState)
			return msTypeConvert(js, list)


	elif toType is Point:
		re = Point()
		if fromType is list:
			re.x = data[0]
			re.y = data[1]
			re.z = data[2]
		elif (fromType is Vector3):
			list3 = msTypeConvert(data,list)
			re = msTypeConvert(list3, Pose)
		elif (fromType is Pose) or (fromType is PoseStamped) or (fromType is Transform) or (fromType is TransformStamped):
			list7 = msTypeConvert(data, list)
			re = msTypeConvert(list7[:3], Point)

	elif toType is Vector3:
		re = Vector3()
		if fromType is list:
			re.x = data[0]
			re.y = data[1]
			re.z = data[2]
		elif (fromType is Pose):
			list3 = msTypeConvert(data,list)
			re = msTypeConvert(list3, Vector3)
		elif (fromType is Pose) or (fromType is PoseStamped) or (fromType is Transform) or (fromType is TransformStamped):
			list7 = msTypeConvert(data, list)
			re = msTypeConvert(list7[:3], Vector3)

	elif toType is Pose:
		re = Pose()
		if fromType is list:
			re.position.x = data[0]
			re.position.y = data[1]
			re.position.z = data[2]
			re.orientation.x = data[3]
			re.orientation.y = data[4]
			re.orientation.z = data[5]
			re.orientation.w = data[6]
		elif (fromType is PoseStamped) or (fromType is Transform) or (fromType is TransformStamped):
			list7 = msTypeConvert(data, list)
			re = msTypeConvert(list7, Pose)
		elif (fromType is Quaternion) or (fromType is QuaternionStamped):
			list4 = msTypeConvert(data, list)
			re = msTypeConvert([0,0,0]+list4, Pose)
	elif toType is PoseStamped:
		re = PoseStamped()
		if (fromType is list) or (fromType is Pose) or (fromType is Transform) 	or (fromType is Quaternion):
			re.pose = msTypeConvert(data, Pose)
		elif (fromType is TransformStamped) or (fromType is QuaternionStamped):
			re.header = data.header
			re.pose = msTypeConvert(data, Pose)

	elif toType is Transform:
		re = Transform()
		if fromType is list:
			re.translation.x = data[0]
			re.translation.y = data[1]
			re.translation.z = data[2]
			re.rotation.x = data[3]
			re.rotation.y = data[4]
			re.rotation.z = data[5]
			re.rotation.w = data[6]
		elif (fromType is TransformStamped) or (fromType is Pose) or (fromType is PoseStamped):
			list7 = msTypeConvert(data, list)
			re = msTypeConvert(list7, Transform)
		elif (fromType is Quaternion) or (fromType is QuaternionStamped):
			list4 = msTypeConvert(data, list)
			re = msTypeConvert([0,0,0]+list4, Transform)

	elif toType is TransformStamped:
		re = TransformStamped()
		if (fromType is list) or (fromType is Pose) or (fromType is Transform) or (fromType is Quaternion):
			re.transform = msTypeConvert(data, Transform)
		elif (fromType is PoseStamped) or (fromType is QuaternionStamped):
			re.header = data.header
			re.transform = msTypeConvert(data, Transform)

	elif toType is Quaternion:
		re = Quaternion()
		if fromType is list:
			re.x = data[0]
			re.y = data[1]
			re.z = data[2]
			re.w = data[3]
		elif (fromType is Pose) or (fromType is PoseStamped) or (fromType is Transform) or (fromType is TransformStamped):
			list7 = msTypeConvert(data, list)
			re = msTypeConvert(list7[3:7], Quaternion)
		elif fromType is QuaternionStamped:
			re = data.quaternion
	elif toType is QuaternionStamped:
		re = QuaternionStamped()
		if (fromType is list)	or (fromType is Pose) or (fromType is Transform) or (fromType is Quaternion):
			re.quaternion = msTypeConvert(data, Quaternion)
		elif (fromType is PoseStamped) or (fromType is TransformStamped):
			re.header = data.header
			re.quaternion = msTypeConvert(data, Quaternion)

	elif toType is JointState:
		re = JointState()
		if fromType is list:
			re.position = data
		elif fromType is JointTrajectoryPoint:
			re.position = data.positions
		elif fromType is JointTrajectory:
			re.header = data.header
			re.position = data.points[ len(data.points)-1 ].positions
		elif fromType is RobotTrajectory:
			re.header = data.joint_trajectory.header
			re.position = data.joint_trajectory.points[ len(data.joint_trajectory.points)-1 ].positions
		elif fromType is RobotState:
			re = data.joint_state

	elif fromType is JointTrajectoryPoint:
		re = JointTrajectoryPoint()
		if fromType is list:
			re.positions = data
		elif fromType is JointState:
			re.positions = data.position
		elif fromType is JointTrajectory:
			re.positions = data.points[ len(data.points)-1 ].positions
		elif fromType is RobotTrajectory:
			re.positions = data.joint_trajectory.points[ len(data.joint_trajectory.points)-1 ].positions

	elif toType is JointTrajectory:
		re = JointTrajectory()
		if fromType is list:
			for i in range(len(data)):
				re.points.append( msTypeConvert(data[i], JointTrajectoryPoint) )
		elif fromType is JointState:
			re.header = data.header
			re.points.append(JointTrajectoryPoint)
			re.points[0].positions = data.position
		elif fromType is JointTrajectoryPoint:
			re.points.append(JointTrajectoryPoint)
			re.points[0].positions = data.positions
		elif fromType is RobotTrajectory:
			re.header = data.joint_trajectory.header
			re.points = data.joint_trajectory.points

	elif toType is RobotTrajectory:
		re = RobotTrajectory()
		if fromType is JointTrajectory:
			re.joint_trajectory = data
		elif (fromType is list) or (fromType is JointState)	or (fromType is JointTrajectoryPoint):
			re.joint_trajectory = msTypeConvert(data, JointTrajectory)

	elif toType is RobotState:
		re = RobotState()
		if fromType is list:
			js = msTypeConvert(data, JointState)
			re.joint_state = js
		elif fromType is JointState:
			re.joint_state = data


	elif toType == "xyz":
		if fromType is list:
			if len(data)==3:
				re = data
			elif len(data) == 6:
				re = data[:3]
			elif len(data) == 7:
				re = data[:3]
		elif (fromType is Point) or (fromType is Vector3):
			re = [data.x, data.y, data.z]
		elif fromType is Pose:
			re = [data.position.x, data.position.y, data.position.z]
		elif fromType is PoseStamped:
			re = msTypeConvert(data.pose, "xyz")
		elif fromType is Transform:
			re = [data.translation.x, data.translation.y, data.translation.z]
		elif fromType is TransformStamped:
			re = msTypeConvert(data.transform, "xyz")

	elif toType == "rpy":
		if fromType is list:
			if len(data) == 3:
				re = data
			elif len(data) == 4:
				re = msQuaternionToRpy(data[0], data[1], data[2], data[3])
			elif len(data) == 6:
				re = data[3:6]
			elif len(data) == 7:
				re = msQuaternionToRpy(data[3], data[4], data[5], data[6])
		elif fromType is Pose:
			re = msQuaternionToRpy(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
		elif fromType is PoseStamped:
			re = msTypeConvert(data.pose, "rpy")
		elif fromType is Transform:
			re = msQuaternionToRpy(data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w)
		elif fromType is TransformStamped:
			re = msTypeConvert(data.transform, "rpy")
		elif fromType is Quaternion:
			re = msQuaternionToRpy(data.x, data.y, data.z, data.w)
		elif fromType is QuaternionStamped:
			re = msTypeConvert(data.quaternion, "rpy")
	elif toType == "xyzw":
		if fromType is list:
			if   len(data) == 3:
				re = msRpyToQuaternion(data[0], data[1], data[2])
			elif len(data) == 4:
				re = data
			elif len(data) == 6:
				re = msRpyToQuaternion(data[3], data[4], data[5])
			elif len(data) == 7:
				re = data[3:7]
		if fromType is Pose:
			re = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
		elif fromType is PoseStamped:
			re = msTypeConvert(data.pose, "xyzw")
		elif fromType is Transform:
			re = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
		elif fromType is TransformStamped:
			re = msTypeConvert(data.transform, "xyzw")
		elif fromType is Quaternion:
			re = [data.x, data.y, data.z, data.w]
		elif fromType is QuaternionStamped:
			re = msTypeConvert(data.quaternion, "xyzw")

	elif toType == "xyzrpy":
		re = msTypeConvert(data, "xyz") + msTypeConvert(data, "rpy")
	elif toType == "xyzxyzw":
		re = msTypeConvert(data, "xyz") + msTypeConvert(data, "xyzw")
	if re is None:
		return False
	else:
		return re


"""
def convertToList(data):
	fromType = type(data)
	if   fromType is Point:
		re = [data.x, data.y, data.z]
	elif fromType is Vector3:
		re = [data.x, data.y, data.z]

	elif fromType is Quaternion:
		re = [data.x, data.y, data.z, data.w]
	elif fromType is QuaternionStamped:
		re = convertToList(data.quaternion)

	elif fromType is Pose:
		re = convertToList(data.position) + convertToList(data.orientation)
	elif fromType is PoseStamped:
		re = convertToList(data.pose)

	elif fromType is Transform:
		re = convertToList(data.translation) + convertToList(data.rotation)
	elif fromType is TransformStamped:
		re = convertToList(data.transform)

	elif fromType is JointState:
		re = []
		for i in range(len(data.position)):
			re.append(data.position[i])
	elif fromType is JointTrajectoryPoint:
		re = []
		for i in range(len(data.positions)):
			re.append( positions[i] )
	elif fromType is JointTrajectory:
		re = []
		for i in range(len(data.points)):
			re.append(convertToList( data.points[i].positions) )
	elif fromType is RobotTrajectory:
		re = convertToList( data.joint_trajectory )
	return re

def convetToPoint(data):
	fromType = type(data)
	re = Point()
	if fromType is list:
		re.x = data[0]
		re.y = data[1]
		re.z = data[2]
	elif (fromType is Vector3):
		list3 = convertToList(data)
		re = convetToPoint(list3)
	elif (fromType is Pose) or (fromType is PoseStamped) or (fromType is Transform) or (fromType is TransformStamped):
		list7 = convertToList(data)
		re = convetToPoint(list7[:3])
	return re

def convertToVector3(data):
	fromType = type(data)
	re = Vector3()
	if fromType is list:
		re.x = data[0]
		re.y = data[1]
		re.z = data[2]
	elif (fromType is Pose):
		list3 = convertToList(data)
		re = convertToVector3(list3)
	elif (fromType is Pose) or (fromType is PoseStamped) or (fromType is Transform) or (fromType is TransformStamped):
		list7 = convertToList(data)
		re = convertToVector3(list7[:3])
	return re

def convertToPose(data):
	fromType = type(data)
	re = Pose()
	if fromType is list:
		re.position.x = data[0]
		re.position.y = data[1]
		re.position.z = data[2]
		re.orientation.x = data[3]
		re.orientation.y = data[4]
		re.orientation.z = data[5]
		re.orientation.w = data[6]
	elif (fromType is Point) or (fromType is Vector3):
		list3 = convertToList(data)
		re = convertToPose(list3+[0,0,0,0])
	elif (fromType is Quaternion) or (fromType is QuaternionStamped):
		list4 = convertToList(data)
		re = convertToPose([0,0,0]+list4)
	elif (fromType is PoseStamped) or (fromType is Transform) or (fromType is TransformStamped):
		list7 = convertToList(data)
		re = convertToPose(list7)
	return re

def convertToPoseStamped(data):
	fromType = type(data)
	re = PoseStamped()
	if (fromType is list) or (fromType is Pose) or (fromType is Transform) 	or (fromType is Quaternion):
		re.pose = convertToPose(data)
	elif (fromType is TransformStamped) or (fromType is QuaternionStamped):
		re.header = data.header
		re.pose = convertToPose(data)
	return re

def convertToTransform(data):
	fromType = type(data)
	re = Transform()
	if fromType is list:
		re.translation.x = data[0]
		re.translation.y = data[1]
		re.translation.z = data[2]
		re.rotation.x = data[3]
		re.rotation.y = data[4]
		re.rotation.z = data[5]
		re.rotation.w = data[6]
	elif (fromType is Point) or (fromType is Vector3):
		list3 = convertToList(data)
		re = convertToTransform(list3+[0,0,0,0])
	elif (fromType is Quaternion) or (fromType is QuaternionStamped):
		list4 = convertToList(data)
		re = convertToTransform([0,0,0]+list4)
	elif (fromType is TransformStamped) or (fromType is Pose) or (fromType is PoseStamped):
		list7 = convertToList(data)
		re = convertToTransform(list7)
	return re

def convertToTransformStamped(data):
	fromType = type(data)
	re = TransformStamped()
	if (fromType is list) or (fromType is Pose) or (fromType is Transform) or (fromType is Quaternion):
		re.transform = convertToTransform(data)
	elif (fromType is PoseStamped) or (fromType is QuaternionStamped):
		re.header = data.header
		re.transform = convertToTransform(data)
	return re

def convertToQuaternion(data):
	fromType = type(data)
	re = Quaternion()
	if fromType is list:
		re.x = data[0]
		re.y = data[1]
		re.z = data[2]
		re.w = data[3]
	elif (fromType is Pose) or (fromType is PoseStamped) or (fromType is Transform) or (fromType is TransformStamped):
		list7 = convertToList(data)
		re = convertToQuaternion(list7[3:7])
	elif fromType is QuaternionStamped:
		re = data.quaternion
	return re

def convertToQuaternionStamped(data):
	fromType = type(data)
	re = QuaternionStamped()
	if (fromType is list)	or (fromType is Pose) or (fromType is Transform) or (fromType is Quaternion):
		re.quaternion = convertToQuaternion(data)
	elif (fromType is PoseStamped) or (fromType is TransformStamped):
		re.header = data.header
		re.quaternion = convertToQuaternion(data)
	return re

def convertToJointState(data):
	fromType = type(data)
	re = JointState()
	if fromType is list:
		re.position = data
	elif fromType is JointTrajectoryPoint:
		re.position = data.positions
	elif fromType is JointTrajectory:
		re.header = data.header
		re.position = data.points[ len(data.points)-1 ].positions
	elif fromType is RobotTrajectory:
		re.header = data.joint_trajectory.header
		re.position = data.joint_trajectory.points[ len(data.joint_trajectory.points)-1 ].positions

def convertToJointTrajectoryPoint(data):
	fromType = type(data)
	re = JointTrajectoryPoint()
	if fromType is list:
		re.positions = data
	elif fromType is JointState:
		re.positions = data.position
	elif fromType is JointTrajectory:
		re.positions = data.points[ len(data.points)-1 ].positions
	elif fromType is RobotTrajectory:
		re.positions = data.joint_trajectory.points[ len(data.joint_trajectory.points)-1 ].positions


def convertToJointTrajectory(data):
	fromType = type(data)
	re = JointTrajectory()
	if fromType is list:
		for i in range(len(data)):
			re.points.append( msTypeConvert(data[i], JointTrajectoryPoint) )
	elif fromType is JointState:
		re.header = data.header
		re.points.append(JointTrajectoryPoint)
		re.points[0].positions = data.position
	elif fromType is JointTrajectoryPoint:
		re.points.append(JointTrajectoryPoint)
		re.points[0].positions = data.positions
	elif fromType is RobotTrajectory:
		re.header = data.joint_trajectory.header
		re.points = data.joint_trajectory.points

def convertToRobotTrajectory(data):
	fromType = type(data)
	re = RobotTrajectory()
	if fromType is JointTrajectory:
		re.joint_trajectory = data
	elif (fromType is list) or (fromType is JointState)	or (fromType is JointTrajectoryPoint):
		re.joint_trajectory = convertToJointTrajectory(data)

def convertToXyz(data):
	fromType = type(data)
	if fromType is list:
		if len(data) == 6:
			re = data[:3]
		elif len(data) == 7:
			re = data[:3]
	elif fromType is Pose:
		re = [data.position.x, data.position.y, data.position.z]
	elif fromType is PoseStamped:
		re = msTypeConvert(data.pose, "xyz")
	elif fromType is Transform:
		re = [data.translation.x, data.translation.y, data.translation.z]
	elif fromType is TransformStamped:
		re = msTypeConvert(data.transform, "xyz")
	return re

def convertToRpy(data):
	fromType = type(data)
	if fromType is list:
		if len(data) == 3:
			re = data
		elif len(data) == 4:
			re = msQuaternionToRpy(data[0], data[1], data[2], data[3])
		elif len(data) == 6:
			re = data[3:6]
		elif len(data) == 7:
			re = msQuaternionToRpy(data[3], data[4], data[5], data[6])
	elif fromType is Pose:
		re = msQuaternionToRpy(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
	elif fromType is PoseStamped:
		re = msTypeConvert(data.pose, "rpy")
	elif fromType is Transform:
		re = msQuaternionToRpy(data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w)
	elif fromType is TransformStamped:
		re = msTypeConvert(data.transform, "rpy")
	elif fromType is Quaternion:
		re = msQuaternionToRpy(data.x, data.y, data.z, data.w)
	elif fromType is QuaternionStamped:
		re = msTypeConvert(data.quaternion, "rpy")

	elif toType == "xyzw":
		if fromType is list:
			if   len(data) == 3:
				re = msRpyToQuaternion(data[0], data[1], data[2])
			elif len(data) == 4:
				re = data
			elif len(data) == 6:
				re = msRpyToQuaternion(data[3], data[4], data[5])
			elif len(data) == 7:
				re = data[3:7]
		if fromType is Pose:
			re = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
		elif fromType is PoseStamped:
			re = msTypeConvert(data.pose, "xyzw")
		elif fromType is Transform:
			re = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
		elif fromType is TransformStamped:
			re = msTypeConvert(data.transform, "xyzw")
		elif fromType is Quaternion:
			re = [data.x, data.y, data.z, data.w]
		elif fromType is QuaternionStamped:
			re = msTypeConvert(data.quaternion, "xyzw")
	elif toType == "xyzrpy":
		re = msTypeConvert(data, "xyz") + msTypeConvert(data, "rpy")
	elif toType == "xyzxyzw":
		re = msTypeConvert(data, "xyz") + msTypeConvert(data, "xyzw")
"""

def msRpyToQuaternion(r,p,y):
	import tf
	return list( tf.transformations.quaternion_from_euler(r,p,y) )

def msQuaternionToRpy(x,y,z,w):
	import tf
	from geometry_msgs.msg import Quaternion
	if type(x) is Quaternion:
		w = x.w
		z = x.z
		y = x.y
		x = x.x
	return list( tf.transformations.euler_from_quaternion( (x,y,z,w) ) )

