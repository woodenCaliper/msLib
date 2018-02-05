#!/usr/bin/env python
#coding: utf-8
import rospy, tf
import copy, math, time
import numpy as np

import msQuat, msMath, msPython, msTfGetter
from msConvertor import *

from sensor_msgs.msg import JointState
from moveit_msgs.msg import *
import sys, moveit_commander, moveit_msgs
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory

##
# rosのmove_groupクラスを継承して、いくつか関数を追加したもの<br>
# @author sugino
# @version 1.1
# @since 2018/12/22
class msMoveGroup(moveit_commander.MoveGroupCommander):
	##
	# @param groupName Moveitで登録したグループ名
	def __init__(self, groupName):
		self.__pickNumber = 0
		self.__placeNumber = 0
		self.__tfg = msTfGetter.tfGetter()
		moveit_commander.MoveGroupCommander.__init__(self, groupName)	#オリジナルのMoveGroupCommanderのコンストラクタを実行
		print groupName, "内の有効なジョイント ↓ \n", self.get_active_joints()
		if self.has_end_effector_link():
			print "エンドエフェクタ ↓ \n", self.get_end_effector_link()
		self.set_planning_time(1)
		self.allow_replanning(True)
		# self.set_goal_tolerance(0.000001)
		# print self.get_goal_tolerance()
		# self.set_goal_position_tolerance()
		# self.set_goal_orientation_tolerance()

	## 軌道を生成する<br>
	# 優先順位: random > poseName > joint > pose > position > posture <br>
	# 例)msPlan(joint=[0,0,0,0,0,0])
	# @param position(list_of_xyz_position,Point,Vector3) = 位置
	# @param posture(list_of_xyzw,list_of_rpy,Quaternion) = 姿勢
	# @param pose(list_of_xyzxyzw,list_of_xyzrpy,Pose,PoseStamped) = 位置+姿勢
	# @param joint(list_of_joints,JointState) = 関節角度
	# @param poseName(String) = 登録した関節角度の名前
	# @param random(bool) = 目標値をランダムに設定するか
	# @param startJoint(list_of_joints,JointState,RobotState) = 計画を作る初期の関節の状態
	# @return 成功=軌道(RobotTrajectory)
	# @return 失敗=False(bool)
	def msPlan(self, **kwargs):
		funcKeywards = ["position", "posture", "pose", "joint", "poseName", "random", "startJoint"]
		for key in kwargs.keys():
			if not (key in funcKeywards):
				msPython.printColor("warning: msPlan is not support //"+str(key)+"// keyward", "yellow")
		position	= kwargs.get("position")
		posture		= kwargs.get("posture")
		pose		= kwargs.get("pose")
		joint		= kwargs.get("joint")
		poseName	= kwargs.get("poseName")
		startJoint	= kwargs.get("startJoint")
		random		= kwargs.get("random", False)

		if startJoint is None:	#set start state
			self.set_start_state_to_current_state()
		else:
			rs = msTypeConvert(startJoint, RobotState)
			if rs.joint_state.name == []:
				rs.joint_state.name = self.get_active_joints()
			self.set_start_state(rs)

		if random is True:
			self.set_random_target()
		elif poseName is not None:
			self.set_named_target(poseName)
		elif joint is not None:
			joint = msTypeConvert(joint, list)
			self.set_joint_value_target(joint)
		else:
			if (position is not None) and (posture is not None):
				position = msTypeConvert(position, "xyz")
				posture  = msTypeConvert(posture,  "xyzw")
				pose = position + posture
			if pose is not None:
				pose = msTypeConvert(pose, "xyzxyzw")
				self.set_pose_target( pose )
			elif position is not None:
				print position
				position = msTypeConvert(position, "xyz")
				self.set_position_target(position)
			elif posture is not None:
				posture  = msTypeConvert(posture,  "xyzw")
				self.set_orientation_target(posture)


		plan = self.plan()	#計画するとこ、最重要
		self.set_start_state_to_current_state()
		if not len(plan.joint_trajectory.points)==0:	#sceneがいい感じに生成されtえたら
			return plan
		else:
			print "error: cannot calculate trajectory"
			return False

	## 軌跡の計画と実行を行う。計画(plan)を渡すと実行のみを行う<br>
	# 優先順位: plan > random > poseName > joint > pose > position > posture <br>
	# 例)msGo(joint=[0,0,0,0,0,0])
	# @param position(list_of_xyz_position,Point,Vector3) = 位置
	# @param posture(list_of_xyzw,list_of_rpy,Quaternion) = 姿勢
	# @param pose(list_of_xyzxyzw,list_of_xyzrpy,Pose,PoseStamped) = 位置+姿勢
	# @param joint(list_of_joints,JointState) = 関節角度
	# @param poseName(String) = 登録した関節角度の名前
	# @param random(bool) = 目標値をランダムに設定するか
	# @param startJoint(list_of_joints,JointState,RobotState) = 計画を作る初期の関節の状態
	# @param plan(RobotTrajectory) = 事前に立てた計画
	# @param isWaitDisplayRviz(bool) = Rvizの描画完了まで待つかどうか
	# @return 成否(bool)
	def msGo(self, **kwargs):
		funcKeywardsForPlan = ["position", "posture", "pose", "joint", "poseName", "random", "startJoint"]
		funcKeywardsForGo = ["plan", "isWaitDisplayRviz"]
		funcKeywards = funcKeywardsForPlan + funcKeywardsForGo
		msPython.checkTrueKey(kwargs, funcKeywards)

		plan = kwargs.get("plan")
		isWaitDisplayRviz = kwargs.get("isWaitDisplayRviz", True)
		if plan is None:
			for k in funcKeywardsForGo:	#kwargsからfuncKeywardsForGoのkeyを削除
				if kwargs.has_key(k):
					kwargs.pop(k)
			plan = self.msPlan(**kwargs)
			if plan is False:
				return False

		success = self.execute(plan, isWaitDisplayRviz)
		self.clear_pose_targets()
		self.set_start_state_to_current_state()
		if success:
			return True
		else:
			print "error: excute plan is faild"
			return False


	## 最終地点までの軌跡を生成してからでなく、１ステップ先ずつ関節角度を生成するから、行き詰まりやすい
	#
	def msPlanWay(self, wayPoint, eef_step=0.01, jump_threshold=0.0, avoid_collisions=True, moveSuccessRatio=0.5):
		#wayPointは、２次元list or Poseのlist
		if type(wayPoint[0]) is Pose:
			wayPoint = [msTypeConvert(p, "xyzxyzw") for p in waypoints]
		(ser_path, fraction) = self._g.compute_cartesian_path(wayPoint, eef_step, jump_threshold, avoid_collisions)
		plan = RobotTrajectory()
		plan.deserialize(ser_path)
		print "fraction ",fraction
		if fraction <= moveSuccessRatio:	#fractionは生成した軌跡の成功している割合、1.0=100%生成できている
			return False
		return plan

	## referenceFrameを基準座標としたvectorListの方向にエンドエフェクタを動かす軌跡を生成する
	# @param vectorList(list_of_xyz) 移動するベクトル(方向と量)
	# @param referenceFrame(String) vectorListの基準座標
	# @return 軌跡(RobotTrajectory)
	def msPlanShift(self, vectorList, referenceFrame=None):
		if referenceFrame is None:
			referenceFrame = self.get_pose_reference_frame()	#base_link

		currentTf = self.__tfg.lookTransform(referenceFrame, self.get_end_effector_link())
		current =  msTypeConvert(currentTf, "xyzxyzw")

		if len(vectorList)==3:
			vectorList = vectorList+[0,0,0,0]
		goal = [0,0,0, 0,0,0,0]
		for i in range(7):
			goal[i] += current[i] + vectorList[i]

		startPs = msTypeConvert(current, PoseStamped)
		startPs.header.frame_id = referenceFrame
		goalPs = msTypeConvert(goal, PoseStamped)
		goalPs.header.frame_id = referenceFrame

		startTf = self.__tfg.getPoseFromTf( self.get_pose_reference_frame(), startPs)	#base_linkからみた位置
		goalTf = self.__tfg.getPoseFromTf( self.get_pose_reference_frame(), goalPs)	#base_linkからみた位置

		start = msTypeConvert(startTf, "xyzxyzw")
		goal = msTypeConvert(goalTf, "xyzxyzw")
		wayPoint = [start, goal]

		plan = self.msPlanWay(wayPoint)
		return plan

	##referenceFrameからみたendEffectorのPoseを取得 <br>
	#get_current_poseでは、planning_frameからの座標を取得するため
	def msGetCurrentPose(self):
		result = self.__tfg.lookTransform( self.get_pose_reference_frame(), self.get_end_effector_link())
		if result is not False:
			return msTypeConvert(result, PoseStamped)
		return False



	## アームを何かの方向に向けたいときの姿勢を決定する関数
	# @param goalPosition(list_of_xyz) = アーム先端を向けたい位置
	# @param armPosition(list_of_xyz) = アームの最終位置、どの位置からオブジェクトの方向を向かせたいか
	# @param tipDirectionVecotr3(list_of_xyz) = エンドエフェクタ座標系での、アーム先端方向。デフォはｚ軸方向
	# @param openVector(list_of_xyz) = base_link座標系での、アームの手の平？を向けたい方向。デフォはz軸方向
	# @return 姿勢(list_of_xyzw)
	def msPointToPosition(self, goalPosition, armPosition, tipDirectionVector3=[0,0,1], baseOpenVector=[0,0,1]):
		armOpenVector = [0,1,0]
		#>>>>>相対位置より、tipをgoalPosに向けるための処理
		armToGoalVector = [goalPosition[0]-armPosition[0], goalPosition[1]-armPosition[1], goalPosition[2]-armPosition[2]]
		if armToGoalVector == [0,0,0]:	#位置が完全一致の場合
			pointToGoalQuat = [0,0,0,1]
		else:
			theta = msMath.getTheta(tipDirectionVector3, armToGoalVector)	#2つのベクトルの角度差
			axis = msMath.crossProduct(tipDirectionVector3, armToGoalVector)	#アーム姿勢を回転させるための軸
			if msMath.isNear(axis, [0.0, 0.0, 0.0]):	#theta=0 or πの時
				axis = [1,0,0]
			#アーム姿勢を回転させるためのクオータニオン
			pointToGoalQuat = msQuat.makeQuat(axis, theta)	#tipからgoalに向けるためのQuat
		#<<<<<相対位置より、tipをgoalPosに向けるための処理

		#>>>>>baseOpenVectorをグリッパの上に向ける処理
		mappedOpenVector = msMath.mappingVectorToPlane(baseOpenVector, armToGoalVector)	#armのzの平面にopenVectorを写像
		armXVector = msQuat.rotateVector(armOpenVector, pointToGoalQuat)
		if mappedOpenVector == [0,0,0]:
			return [0,0,0,1]
		theta2 = msMath.getTheta(armXVector, mappedOpenVector)	#写像されたopenVectorと、z軸を向けた後のx軸との角度差
		axis2 = msMath.crossProduct(armXVector, mappedOpenVector)
		if msMath.isNear(axis2, [0.0, 0.0, 0.0]):	#theta2=0orπの時
			axis2 =  msQuat.rotateVector(tipDirectionVector3, pointToGoalQuat)
		pointToBaseOpenQuat = msQuat.makeQuat(axis2, theta2)	#baseOpenVectorに向けるQuat
		#<<<<<baseOpenVectorをグリッパの上に向ける処理
		return msQuat.product(pointToBaseOpenQuat, pointToGoalQuat)	#Quatの合成


	##不具合あり
	# @param grippingJointAngle = 掴んているときのgripperの角度
	# @param openningJointAngle = 開いている時のgripperの角度
	# @param objectPoint = 把持対象物の位置(把持するときに合わせる位置)
	# @param gripPosture = 把持時のエンドエフェクタの姿勢
	# @param gripPosition = エンドエフェクタから見た、掴んだ時にobjectが来る位置
	# @param toGripMoveVector = 掴みに行く動作の方向と大きさ、エンドエフェクタ基準
	# @param toRetreatMoveVector = 掴んだ後に少し移動する方向と大きさ、エンドエフェクタ基準
	def msPick(self, objectId,
					 grippingJointAngle,		openningJointAngle,
					 objectPoint=None,			gripPosture=None,
					 gripPosition=[0,0,-0.02],
					 toGripMoveVector=[0, 0, 0.15],	toRetreatMoveVector=[0, 0.15, 0],
					 tryNum=2):
		toGripMoveDistance = msMath.vectorSize(toGripMoveVector)
		toRetreatMoveDistance = msMath.vectorSize(toRetreatMoveVector)

		g = moveit_msgs.msg.Grasp()
		g.allowed_touch_objects = objectId
		g.pre_grasp_posture  = self.makeGripperPosture(openningJointAngle)
		g.pre_grasp_approach = self.makeGripperTranslation(toGripMoveDistance*0.5, toGripMoveDistance, toGripMoveVector)	#掴みに行く方向、エンドエフェクタ基準
		g.grasp_posture      = self.makeGripperPosture(grippingJointAngle)
		g.post_grasp_retreat = self.makeGripperTranslation(toRetreatMoveDistance*0.5, toRetreatMoveDistance, toRetreatMoveVector)	#逃げ、エンドエフェクタ基準
		g.post_place_retreat = self.makeGripperTranslation(0.1, 0.15, [0,0,1])
		g.max_contact_force  = 0

		if objectPoint is None:
			scene =  moveit_commander.PlanningSceneInterface()
			objectPointFromWorld = scene.get_object_poses([objectId])[objectId]

			ps = PoseStamped()
			ps.header.stamp = rospy.Time.now()
			ps.header.frame_id = "world"
			ps.pose = objectPointFromWorld

			objectTf = self.__tfg.getPoseFromTf("base_link", ps)
			objectPoint = msTypeConvert(objectTf, Pose)

		else:
			print objectPoint
			objectPoint = msTypeConvert(objectPoint, Point)#list等からPoint型に変換

		if (gripPosture=="ring"):
			gripPosture = []
			ringPoints = []
			postureNum = 60
			for i in range(postureNum):	#Ring状に点を生成
				theta = 2*math.pi * (float(i)/postureNum)
				ringPoints.append([ math.cos(theta), math.sin(theta), 0 ])
			for i in ringPoints:	#点から中心へ向く姿勢を生成
				postureTypeList = self.msPointToPosition([0,0,0], i, baseOpenVector=[0,0,1])
				gripPosture.append( msTypeConvert(postureTypeList, Quaternion) )
		elif (gripPosture=="sphere") or (gripPosture is None):		#球状26方向からの姿勢を生成
			gripPosture = []
			points = []
			for i in [-1,0,1]:
				for j in [-1,0,1]:
					for k in [-1,0,1]:
						points.append([i,j,k])
			points.remove([0,0,0])
			abc = 0
			for i in points:
				abc+=1
				postureTypeList = self.msPointToPosition([0,0,0], i)
				gripPosture.append( msTypeConvert(postureTypeList, Quaternion) )
		else:	#姿勢のリストの場合
			if type(gripPosture) is not list:
				gripPosture = [gripPosture]
			for i in range(len(gripPosture)):
				gripPosture[i] = msTypeConvert(gripPosture[i], Quaternion)

		startTime = time.time()

		for j in range(tryNum):
			for i in range(len(gripPosture)):
				objectPointList = msTypeConvert(objectPoint, list)
				gripPostureList = msTypeConvert(gripPosture[i], "xyzw")

				objTf = msTypeConvert(objectPointList+gripPostureList, TransformStamped)

				objTf.header.stamp = rospy.Time.now()
				objTf.header.frame_id = "base_link"
				objTf.child_frame_id = "objPos"
				self.__tfg.broadcastTf(objTf)	#生成してた位置と姿勢をtfとしてPub

				ps = PoseStamped()
				ps.header.frame_id = "objPos"
				gripPosition2  = map(lambda x: -x, gripPosition)	#位置反転

				ps.pose.position = msTypeConvert(gripPosition2, Point)
				ps.pose.orientation = msTypeConvert([0,0,0,1], Quaternion)
				time.sleep(0.05)
				ps.header.stamp = rospy.Time.now()

				overObjTf = self.__tfg.getPoseFromTf("base_link", ps)	#さっきPubしたtfからgripPointを加味したobjectPointを得る
				overObjPoint = msTypeConvert(overObjTf, Point)

				g.id = "toGrip_" + objectId +"_"+ str(i)+"-"+str(j)
				g.grasp_pose.header.stamp = rospy.get_rostime()
				g.grasp_pose.header.frame_id = "/base_link"
				g.grasp_pose.pose.position = overObjPoint
				g.grasp_pose.pose.orientation = gripPosture[i]
				g.grasp_quality = 1.0
				result = self.pick(objectId, g)
				# result=-1
				print result,  j, i
				if result is MoveItErrorCodes.SUCCESS:
					return True
				time.sleep(0.01)
		return False


	## 不具合あり
	def msPlace(self,	objectId, openningJointAngle, placePosition,
						toPlaceMoveVector=[0,-1,0], toRetreatMoveVector=[0,0,1]):
		h = moveit_msgs.msg.PlaceLocation()
		h.id = "placing_id"
		h.post_place_posture = self.makeGripperPosture(openningJointAngle)	#離すとき

		h.place_pose = placePosition

		#置く前に、一旦手前まで移動してから、place動作に移るから、その
		#手前から、place_poseに行くまでの動作
		#手前の位置から動く方向が[0,-1,0]
		h.pre_place_approach = self.makeGripperTranslation(0.01, 0.15, toPlaceMoveVector )
		h.post_place_retreat = self.makeGripperTranslation(0.01, 0.15, toRetreatMoveVector )	#置いたあとに逃げるベクトル
		h.allowed_touch_objects =  objectId
		print h

		timeout=10
		startTime = time.time()
		while(time.time()-startTime<timeout):
			result = rightArmGroup.place(objectId, h)
			if result is MoveItErrorCodes.SUCCESS:
				break


	def makeGripperPosture(self, jointAngle):
		t = JointTrajectory()
		t.header.stamp = rospy.get_rostime()
		t.header.frame_id = "base_link"
		t.joint_names = ['right_gripper_left_joint', "right_gripper_right_joint"]
		tp = JointTrajectoryPoint()
		tp.positions = jointAngle
		tp.effort = [0.02,0.02]
		tp.time_from_start = rospy.Duration(0.0)
		t.points.append(tp)
		return t

	def makeGripperTranslation(self, min_dist, desired, vector):
		g = moveit_msgs.msg.GripperTranslation()
		g.direction.header.stamp = rospy.get_rostime()
		g.direction.header.frame_id = "base_link"

		g.direction.vector.x = vector[0]
		g.direction.vector.y = vector[1]
		g.direction.vector.z = vector[2]

		# g.direction.header.frame_id = 'right_dummy_tip_link'
		g.direction.header.frame_id = self.get_end_effector_link()

		g.min_distance = min_dist
		g.desired_distance = desired
		return g

	##
	# @param position 目標(中心)位置
	# @param option
	# "RING" z軸周りに円状30箇所の姿勢を生成 <br>
	# "SPHERE" 球状26箇所の姿勢を生成 <br>
	# "UPPER_HEMISPHERE" z軸に垂直な平面で割った半球状17箇所の姿勢を生成
	# @return 位置+姿勢のリスト(list_of_Pose)
	def makeArmPoses(self, position, option="RING"):
		position = msTypeConvert(position, Point)
		postures = []
		if (option=="RING"):
			ringPoints = []
			postureNum = 30
			for i in range(postureNum):	#Ring状に点を生成
				theta = 2*math.pi * (float(i)/postureNum)
				ringPoints.append([ math.cos(theta), math.sin(theta), 0 ])
			for i in ringPoints:	#点から中心へ向く姿勢を生成
				postureTypeList = self.msPointToPosition([0,0,0], i)
				posture = msTypeConvert(postureTypeList, Quaternion)
				postures.append( posture )
		elif (option=="SPHERE"):		#球状26方向からの姿勢を生成
			spherePoints = []
			for x in [-1,0,1]:
				for y in [-1,0,1]:
					for z in [-1,0,1]:
						if not x==y==z==0:
							spherePoints.append([x,y,z])
			for i in spherePoints:
				postureTypeList = self.msPointToPosition([0,0,0], i)
				posture = msTypeConvert(postureTypeList, Quaternion)
				postures.append( posture )
		elif option=="UPPER_HEMISPHERE":	#上半球状17方向からの姿勢を生成
			hemispherePoints = []
			for x in [-1,0,1]:
				for y in [-1,0,1]:
					for z in [0,1]:
						if not x==y==z==0:
							spherePoints.append([x,y,z])
			for i in hemispherePoints:
				postureTypeList = self.msPointToPosition([0,0,0], i)
				posture = msTypeConvert(postureTypeList, Quaternion)
				postures.append( posture )
		else:
			msPython.printColor("error: option value error", "red")

		poseList = []
		for i in range(len(postures)):
			pose = Pose()
			pose.position = position
			pose.orientation = postures[i]
			poseList.append( pose )
		return poseList

	## func setumei
	# @param a print to terminal
	def hoge(a=[1,2,3]):
		print a