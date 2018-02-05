#!/usr/bin/env python
#coding: utf-8
#[x,y,z,w]

from tf import transformations

def makeQuat2(axis, theta):
	qN = transformations.quaternion_about_axis(theta, axis)	#return type=numpy.array
	return qN.tolist()

def product2(leftQuat, rightQuat):
	qN = transformations.quaternion_multiply(leftQuat, rightQuat)
	return qN.tolist()

#https://qiita.com/kenjihiranabe/items/945232fbde58fab45681
def makeQuat(axis, theta):	#回転量と回転軸からQuaternionを生成
	import math
	size = (axis[0]**2 + axis[1]**2 + axis[2]**2)**(1.0/2.0)
	axis = map(lambda x: x/size, axis)	#正規化
	vx= axis[0] * math.sin(theta/2.0)
	vy= axis[1] * math.sin(theta/2.0)
	vz= axis[2] * math.sin(theta/2.0)
	vw= math.cos(theta/2.0)
	return [vx,vy,vz,vw]

def conjugate(quat):	#共役
	return [-quat[0], -quat[1], -quat[2], quat[3]]

def product(leftQuat, rightQuat):	#(外)積、回転の合成
	import numpy
	leftReal = leftQuat[3]
	rightReal = rightQuat[3]
	leftImaginN = numpy.array(leftQuat[:3])
	rightImaginN = numpy.array(rightQuat[:3])
	resultImaginN = numpy.cross(leftImaginN, rightImaginN) + leftReal*rightImaginN + rightReal*leftImaginN
	resultImagin = resultImaginN.tolist()
	resultReal = [leftReal*rightReal - numpy.dot(leftImaginN, rightImaginN)]
	return resultImagin + resultReal

#q * r * qc
def rotateVector(vector3, quaternion):	#点の回転
	conQuat = conjugate(quaternion)
	vector4 = vector3 + [0]
	hoge = product(quaternion, vector4)
	rotatedVector4 = product(hoge, conQuat)
	return rotatedVector4[:3]
