#!/usr/bin/env python
#coding: utf-8

def crossProduct(fromVector, toVector):	#外積
	import numpy
	fromVectorN = numpy.array(fromVector)
	toVectorN = numpy.array(toVector)
	crossN = numpy.cross(fromVectorN, toVectorN)
	return crossN.tolist()

def dotProduct(vectorA, vectorB):	#内積
	import numpy
	vectorAN = numpy.array(vectorA)
	vectorBN = numpy.array(vectorB)
	dot = numpy.dot(vectorAN, vectorBN)
	return float(dot)

def getTheta(vectorA, vectorB):	#２つのベクトルの角度差を取得
	import math
	cosTheta = dotProduct(vectorA, vectorB) / ( vectorSize(vectorA) * vectorSize(vectorB) )
	if 1.0<=cosTheta:
		return 0.0
	if cosTheta<=-1.0:
		return math.pi
	return math.acos(cosTheta)

def vectorSize(vector):	#ベクトルの大きさ
	square = map(lambda x:x**2, vector)
	return (sum(square))**(1.0/2.0)

#正規化
def normalization(vector):
	size = vectorSize(vector)
	return map(lambda x:x/size, vector)

#toVectorがベクトル空間、fromVectorが移したいベクトル本体
def linearMapping(fromVector, toVector):
	hoge = dotProduct(fromVector, toVector) / (vectorSize(toVector)**2)
	return map(lambda x:x*hoge, toVector)

#平面にベクトルを写像する
def mappingVectorToPlane(fromVector, toPlaneVector):
	verticalVector = crossProduct(toPlaneVector, fromVector)
	if verticalVector == [0,0,0]:	#fromVectorとtoPlaneVectorが同一直線上
		return [0,0,0]
	baseVector = crossProduct(verticalVector, toPlaneVector)
	return linearMapping(fromVector, baseVector)



# objectPosを基点とした円（点の集合）を返す
# 回転軸はrotationVectorで表す。
# 返り値の順番はstartingReferencePointの座標をrotationVectorの平面に写した角度から始まる
# objectPosを基点としたstartingReferencePoint
def getRingPosition(objectPos, rotationVector, radius, startingReferencePoint=[1,0,0], angleResolution=0.1):
	import msQuat
	import math
	startBase = mappingVectorToPlane(startingReferencePoint, rotationVector)
	startBaseSize = vectorSize(startBase)
	startBase = map(lambda x:x*radius/startBaseSize, startBase)
	theta=0
	thetaList=[]
	while(theta < 2*math.pi):
		thetaList.append(theta)
		theta += angleResolution
	resultList = []
	for i in range(len(thetaList)):
		quat = msQuat.makeQuat(rotationVector, thetaList[i])
		vec = msQuat.rotateVector(startBase, quat)
		vec = [ vec[0]+objectPos[0], vec[1]+objectPos[1], vec[2]+objectPos[2] ]
		resultList.append(vec)
	return resultList


def isNear(originalVector, nearVector, plusRange=0.00001, minusRange=0.00001):
	for i in range(len(originalVector)):
		if (originalVector[i] < nearVector[i]-minusRange ) or (nearVector[i]+plusRange < originalVector[i] ):
			return False
	return True

#平均、分散、標準偏差の逐次計算(オンライン計算)を行う
class onlineStandardDeviations():
	def __init__(self):
		self.average = 0.0
		self.mn = 0
		self.dataNum = 0.0

	def input(self, data):
		beforeAve = self.average
		self.average = (data - self.average)/(self.dataNum+1) + self.average
		self.mn = (data - self.average)*(data-beforeAve) + self.mn
		self.dataNum += 1.0

	def getAverage(self):
		return self.average
	def getVariance(self):
		if(self.dataNum<=1):
			return 0
		return self.mn/(self.dataNum-1)
	def getStandardDeviation(self):
		return self.getVariance()**(1.0/2.0)
	def getDataNum(self):
		return self.dataNum
	def getResult(self):
		return {"average":self.getAverage(), "variance":self.getVariance(), "standardDivision":self.getStandardDeviation()}