#!/usr/bin/env python
#coding: utf-8


__authorr__ = "sugino"
__version__ = "1.1"
__date__ = "2018/12/21"

import tf2_ros, tf, rospy

##
# rosのtfモジュールを自分用にわかりやすくラッパーしたもの<br>
# と言っても、名前をまとめてlistenerとbroadcasterをまとめたもの
# @author sugino
# @version 1.1
# @since 2018/12/21
class tfGetter:
	## tfとtf2のクラスを初期化<br>
	## 0.5sのsleepを含む
	def __init__(self):
		self.__tfBuffer	= tf2_ros.Buffer()
		self.__listener1 = tf.TransformListener()
		self.__listener2 = tf2_ros.TransformListener(self.__tfBuffer)#tf.transformerを継承して、tfをsubscribeする機能を付与したもの
		self.__broadcaster = tf2_ros.TransformBroadcaster()
		self.__staticBroadcaster = tf2_ros.StaticTransformBroadcaster()
		rospy.sleep(0.5)

	def get(self):
		self.__tfBuffer.can_transform


	## 値のtfをbroadcastする
	# @param transformStamped = tfに上げるTransformStamped
	def broadcastTf(self, transformStamped):
		if transformStamped.header.stamp == rospy.Time(0):
			transformStamped.header.stamp = rospy.Time.now()
		try:
			# print "br tf\t", transformStamped
			self.__broadcaster.sendTransform(transformStamped)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print "error"

	## 値が永続するtfをbroadcastする
	# @param transformStamped = tfに上げるTransformStamped
	def broadcastStaticTf(self, transformStamped):
		transformStamped.header.stamp = rospy.Time.now()
		try:
			self.__staticBroadcaster.sendTransform(transformStamped)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print "error"

	## fromTfNameからみたtoTfNameの状態を得る	<br>
	## timeoutは0.05s
	# @param fromTfName(String) = 基準座標となるTfの名前
	# @param toTfName(String) = 取得対象となるTfの名前
	# @return 成功 = TransformStamped
	# @return 失敗orTimeout = False
	def lookTransform(self, fromTfName, toTfName):
		timeout = rospy.Duration(0.05)
		if toTfName.find("/")==0:
			toTfName=toTfName[1:]
		if fromTfName.find("/")==0:
			fromTfName=fromTfName[1:]
		# if self.__tfBuffer.can_transform(fromTfName, toTfName, rospy.Time(0), timeout):	#最新時間を取得するので、frameがあるかを調べる程度
		# 	return self.__tfBuffer.lookup_transform(fromTfName, toTfName, rospy.Time(0))
		# else:
		# 	print "get tf error"
		# 	return False
		try:
			#最新のものを取得、最大0.05秒までtry
			trans = self.__tfBuffer.lookup_transform(fromTfName, toTfName, rospy.Time(0), timeout)
			return trans
		except:
			print "get tf error"
			return False

	## fromTfNameの基準座標系から見た、poseStamp.header.frame_idの座標系で表されるposeStamp.poseの姿勢を取得する
	# @param fromTfName(String) =　取得したい値の基準座標系のtf
	# @param poseStamp(PoseStamped) = 取得したい座標系の情報を含んだ姿勢<br>
	#   poseStamp.header.stampが空なら、現時刻を取得
	# @return 成功 = TransformStamped
	# @return 失敗 = False
	def getPoseFromTf(self, fromTfName, poseStamp):
		if poseStamp.header.stamp == rospy.Time(0):
			poseStamp.header.stamp = rospy.Time.now()
		try:
			return self.__listener1.transformPose(fromTfName, poseStamp)
		except:
			print "get tf error"
			return False
		# return self.__tfBuffer.transform(poseStamp, fromTfName)
