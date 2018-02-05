#!/usr/bin/env python
# -*- coding: utf-8 -*-

import msPython
import math, time


def flatten(L):	#多次元を１次元に直す
	if isinstance(L, list):
		if L == []:
			return []
		else:
			return flatten(L[0]) + flatten(L[1:])
	else:
		return [L]


class B3mClass(object):
	MAX_ID=255
	MIN_ID=0
	MAX_POS = 32000
	MIN_POS = -32000
	MEMORY_MAP = {
		"ID":(0x00,"byte",0,255),
		"Baudrate":(0x01,"ulong",115200,3000000),
		"PositionMinLimit":(0x05,"short",-32000,32000),
		"PositionMaxLimit":(0x07,"short",-32000,32000),
		"PositionCenterOffset":(0x09,"short",-18000,18000),
		"MCUTempLimit":(0x0B,"short",-32768,32768),
		"MCUTempPowerLimit":(0x0D,"short",0,100),
		"MotorTempLimit":(0x0E,"short",-32768,32768),
		"MotorTempPowerLimit":(0x10,"byte",0,100),
		"CurrentLimit":(0x11,"ushort",0,65535),
		"CurrentPowerLimit":(0x13,"byte",0,100),
		"LockDetectTime":(0x14,"byte",0,255),
		"LockDetectOutputRate":(0x15,"byte",0,100),
		"LockDetectTimePowerLimit":(0x16,"byte",0,100),
		"InputVoltageMin":(0x17,"ushort",0,65535),
		"InputVoltageMax":(0x19,"ushort",0,65535),
		"TorqueLimit":(0x1B,"byte",0,100),
		"DeadBandWidth":(0x1C,"ushort",0,65535),
		"MotorCWRatio":(0x22,"byte",0,100),
		"MotorCCWRatio":(0x23,"byte",0,100),
		"ServoOption":(0x27,"byte",0,255),
		"ServoMode":(0x28,"ushort",0,255),
		"TorqueON":(0x28,"byte",0,255),
		"RunMode":(0x29,"byte",-32000,32000),
		"DesiredPosition":(0x2A,"short",-32000,32000),
		"CurrentPosition":(0x2C,"short",None,None),
		"PreviousPosition":(0x2E,"short",None,None),
		"DesiredVelosity":(0x30,"short",-32768,32768),
		"CurrentVelosity":(0x32,"short",None,None),
		"PreviousVelosity":(0x34,"short",None,None),
		"DesiredTime":(0x36,"ushort",0,65565),
		"RunningTime":(0x38,"ushort",None,None),
		"WorkingTime":(0x3A,"ushort",None,None),
		"DesiredTorque":(0x3C,"short",-32768,32768),
		"SystemClock":(0x3E,"ulong",None,None),
		"SamplingTime":(0x42,"ushort",None,None),
		"MCUTemperature":(0x44,"short",None,None),
		"MotorTemperature":(0x46,"short",None,None),
		"Current":(0x48,"short",None,None),
		"InputVoltage":(0x4A,"ushort",None,None),
		"PwmDuty":(0x4C,"ushort",None,None),
		"PwmFrequency":(0x4E,"ushort",0,65565),
		"EncoderValue":(0x50,"ushort",None,None),
		"EncoderCount":(0x52,"long",-(2**31),(2**31)-1),
		"HallICState":(0x56,"byte",None,None),
		"ControlRow":(0x5C,"ushort",None,None),
		"GainPresetNo":(0x5C,"byte",0,2),
		"ControlType":(0x5D,"byte",None,None),
		"Kp0":(0x5E,"ulong",0,(2**31)-1),
		"Kd0":(0x62,"ulong",0,(2**31)-1),
		"Ki0":(0x66,"ulong",0,(2**31)-1),
		"StaticFriction0":(0x6A,"ushort",0,(2**16)-1),
		"DynamicFriction0":(0x6C,"ushort",0,(2**16)-1),
		"Kp1":(0x6E,"ulong",0,(2**31)-1),
		"Kd1":(0x72,"ulong",0,(2**31)-1),
		"Ki1":(0x76,"ulong",0,(2**31)-1),
		"StaticFriction1":(0x7A,"ushort",0,(2**16)-1),
		"DynamicFriction1":(0x7C,"ushort",0,(2**16)-1),
		"Kp2":(0x7E,"ulong",0,(2**31)-1),
		"Kd2":(0x82,"ulong",0,(2**31)-1),
		"Ki2":(0x86,"ulong",0,(2**31)-1),
		"StaticFriction2":(0x8A,"ushort",0,(2**16)-1),
		"DynamicFriction2":(0x8C,"ushort",0,(2**16)-1),
		"StatusError":(0x9D,"byte",None,None),
		"Status":(0x9E,"ulong",None,None),
		"StatusSystem":(0x9E,"byte",None,None),
		"StatusMotor":(0x9F,"byte",None,None),
		"StatusUart":(0xA0,"byte",None,None),
		"StatusCommand":(0xA1,"byte",None,None),
		"ModelNumber":(0xA2,"ulong",None,None),
		"ModelNumberVoltageClass":(0xA2,"char",None,None),
		"ModelNumberVersion":(0xA3,"byte",None,None),
		"ModelNumberTorque":(0xA4,"byte",None,None),
		"ModelNumberCase":(0xA5,"byte",None,None),
		"ModelType":(0xA6,"ulong",None,None),
		"ModelTypeMotor":(0xA8,"char",None,None),
		"ModelTypeDevice":(0xA9,"char",None,None),
		"FwVersion":(0xAA,"ulong",None,None),
		"FwVersionBuild":(0xAA,"byte",None,None),
		"FwVersionRevision":(0xAB,"byte",None,None),
		"FwVersionMinor":(0xAC,"byte",None,None),
		"FwVersionMajor":(0xAD,"byte",None,None)

		#"EncoderOffsetCenter":(0xAE,"short"),
		#"EncoderOffset":(0xB0,"short")
	}

	def __init__(self, _port=None, _baudrate=1500000, _timeout=0.05):
		self.port = _port
		self.baudrate = _baudrate
		self.timeout = _timeout
		self.lastSnycEndTime=time.time()
		self.receiveLenPlan = []	#1回データやり取りでの受信byte数のリスト
		# self.txtFile = open("b3mSnycTimeRand.txt", "w")
		# self.hoge=0

	def __del__(self):
		self.b3mSerial.close()

	@staticmethod
	def degToPos(deg):
		pos = int(deg*(B3mClass.MAX_POS-B3mClass.MIN_POS)/640.0)
		if (B3mClass.__checkRange(pos, 32000, -32000) is False):
			print "deg range error"
			return False
		return pos
	@staticmethod
	def posToDeg(pos):
		if (B3mClass.__checkRange(pos, 32000, -32000) is False):
			print "pos range error"
			return False
		return (pos-7500)*640.0/(B3mClass.MAX_POS-B3mClass.MIN_POS)
	@staticmethod
	def radToPos(rad):
		pos = (B3mClass.MAX_POS - B3mClass.MIN_POS)/(640.0/180.0)*(rad/math.pi)
		if (B3mClass.__checkRange(pos, 32000, -32000) is False):
			print "rad range error"
			return False
		return int(pos)
	@staticmethod
	def posToRad(pos):
		if (B3mClass.__checkRange(pos, 32000, -32000) is False):
			print "pos range error"
			return False
		return pos*(640.0/180.0)*math.pi/(B3mClass.MAX_POS-B3mClass.MIN_POS)
	@staticmethod
	def __assemblyByte(data):
		reData=0
		if type(data) is not list:
			return Fals__disassemblyBytee
		for i in range(len(data))[::-1]:	#rangeの反転
			reData += data[i]<<(8*i)
		return reData
	@staticmethod
	def __disassemblyByte(data, byteLength):	#リトルエンディアン
		reData = []
		for i in range(byteLength):
			reData.append( data&0xFF )
			data = data>>8
		return reData
	@staticmethod
	def __unsignedToSigned(bit, byteLength):
		if bit >= 1<<(8*byteLength-1):
			return -( (0b1<<(8*byteLength)) - bit )
		return bit
	@staticmethod
	def __signedToUnsigned(num, byteLength):
		if (num<0):
			return (1<<(8*byteLength)) + num
		return num
	@staticmethod
	def __checkRange(val, max, min):	#valが以上以下に収まっているか
		val = flatten(val)
		for i in range(len(val)):
			if (val[i]<min) or (max<val[i]):
				# print "range error"
				return False
		return True
	@staticmethod
	def __checkOption(option):
		return option in {"ERROR", "SYSTEM", "MOTOR", "UART", "COMMAND", "CLEAR"}
	@staticmethod
	def __optionToCmd(option):
		if option == "ERROR":
			return 0b000
		elif option=="SYSTEM":
			return 0b001
		elif option == "MOTOR":
			return 0b010
		elif option == "UART":
			return 0b011
		elif option == "COMMAND":
			return 0b100
		elif option == "CLEAR":
			return 0b10000000
		elif option == "NORMAL":
			return 0b00

	def begin(self, _port=None, _baudrate=None, _timeout=None):
		if (self.port == None) and (_port == None):
			return False
		elif _port != None:
			self.port = _port
		if _baudrate != None:
			self.baudrate=_baudrate
		if _timeout != None:
			self.timeout = _timeout

		self.gardTime = 2.0/self.baudrate + 220*(10**-6)	#2バイト分のデータ＋220us以上
		import serial
		self.b3mSerial = serial.Serial(self.port, self.baudrate,  bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=self.timeout)

	# def __synchronize(self, txBuf, rxLen, waitReceive=True):
	# 	txBuf = [len(txBuf)+2] + txBuf
	# 	txBuf += [sum(txBuf)&0xFF]
	# 	passageTime = time.time()-self.lastSnycEndTime	#前回の送信からの経過時間
	# 	if (self.gardTime > passageTime):
	# 		time.sleep( self.gardTime-passageTime )

	# 	if waitReceive:
	# 		self.moveSerialReceiveToRxBuffer()	#pySerialのrxバッファを移動
	# 		self.b3mSerial.reset_input_buffer()	#残存rxバッファを消す
	# 		self.b3mSerial.write(bytearray(txBuf))
	# 		self.b3mSerial.flush()	#全送信するまで待機

	# 		# start = time.time()
	# 		rxBuf=self.b3mSerial.read(rxLen)
	# 		self.lastSnycEndTime = time.time()	#受信完了時間
	# 		rxBuf = map(lambda x:ord(x), rxBuf)


	# 		if len(rxBuf) == rxLen:
	# 			# snycTime = self.lastSnycEndTime - start
	# 			# print "rxTime",snycTime
	# 			# self.txtFile.write(str(snycTime)+"\n")
	# 			return rxBuf
	# 		else:
	# 			msPython.printColor("b3m rx signal error", "red")
	# 			return False
	# 	else:
	# 		print "__unsynchronize"
	# 		return True


	def __synchronize(self, txBuf, rxLen, waitReceive=True):
		txBuf = [len(txBuf)+2] + txBuf
		txBuf += [sum(txBuf)&0xFF]
		passageTime = time.time()-self.lastSnycEndTime	#前回の送信からの経過時間
		if (self.gardTime > passageTime):
			time.sleep( self.gardTime-passageTime )

		if waitReceive:
			# self.moveSerialReceiveToRxBuffer()	#pySerialのrxバッファを移動
			# self.b3mSerial.reset_input_buffer()	#残存rxバッファを消す
			self.b3mSerial.flushInput()
			self.b3mSerial.write(bytearray(txBuf))
			self.b3mSerial.flush()	#全送信するまで待機
			# start = time.time()

			rxBuf=self.b3mSerial.read(rxLen)
			self.lastSnycEndTime = time.time()	#受信完了時間
			rxBuf = map(lambda x:ord(x), rxBuf)

			if len(rxBuf) == rxLen:
				# snycTime = self.lastSnycEndTime - start
				# print "rxTime",snycTime
				# self.txtFile.write(str(snycTime)+"\n")
				return rxBuf
			else:
				msPython.printColor("b3m rx signal error", "red")
				return False
		else:
			print "__unsynchronize"
			return True


	#pySerialの残存バッファをすべてrxBufferに移す
	def moveSerialReceiveToRxBuffer(self):
		for i in range(len(self.receiveLenPlan)):
			rxBuf = self.b3mSerial.read( self.receiveLenPlan.pop(0) )	#先頭から出す
			rxBuf = map(lambda x:ord(x), rxBuf)
			self.rxBuffer.append(rxBuf)
		# self.b3mSerial.reset_input_buffer()	#残存rxバッファを消す
		self.b3mSerial.flushInput()	#残存rxバッファを消す

		# while(1):
		# 	if len(self.receiveLenPlan)>=1:
		# 		if self.b3mSerial.inWaiting() >= self.receiveLenPlan[0]:
		# 			rxLen = self.receiveLenPlan[0]
		# 			rxBuf = self.b3mSerial.read( self.receiveLenPlan.pop(0) )	#先頭から出す
		# 			rxBuf = map(lambda x:ord(x), rxBuf)
		# 			self.rxBuffer.append(rxBuf)
		# 		else:
		# 			break
		# 	else:
		# 		break

	def readReceiveBuf(self, communicateNum):
		self.moveSerialReceiveToRxBuffer()	#pySerialの残存バッファをすべてrxBufferに移す
		if len(self.rxBuffer) < communicateNum:
			return False
		re = []
		for i in range(communicateNum):
			re.append(rxBuf.pop(0))
		return re

		# if len(self.receiveLenPlan)==0:
		# 	return False
		# if self.b3mSerial.inWaiting() < self.receiveLenPlan[0]:
		# 	return False
		# rxBuf = self.b3mSerial.read( self.receiveLenPlan.pop(0) )
		# rxBuf = map(lambda x:ord(x), rxBuf)
		# return rxBuf

	#シングル/マルチ
	def loadCmd(self, id, option="ERROR"):	#ROMをRAMに書き込み
		if type(id) is not list:
			id = [id]
		if B3mClass.__checkRange(id, 255, 0) is False:
			print "id range error"
			return False, False
		txBuf = [0x01, B3mClass.__optionToCmd(option)] + id
		if len(id)==1:	#シングルモード
			rxBuf = self.__synchronize(txBuf, 5)
			if (rxBuf is False):
				return False, False
			reStatus = rxBuf[2]
			return True, reStatus
		else:	#マルチモード
			rxBuf = self.__synchronize(txBuf, 0)
			if (rxBuf is False):
				return False, False
			return True, True

	#シングル/マルチ
	def saveCmd(self, id, option="ERROR"):	#RAMをROMに書き込み
		if type(id) is not list:
			id = [id]
		if B3mClass.__checkRange(id, B3mClass.MAX_ID, B3mClass.MIN_ID) is False:
			print "id range error"
			return False, False
		txBuf = [0x02, B3mClass.__optionToCmd(option)] + id
		if len(id)==1:	#シングルモード
			rxBuf = self.__synchronize(txBuf, 5)
			if (rxBuf is False):
				return False, False
			reStatus = rxBuf[2]
			return True, reStatus
		else:	#マルチモード
			rxBuf = self.__synchronize(txBuf, 0)
			if (rxBuf is False):
				return False, False
			return True, True

	#シングル
	def readCmd(self, id, address, length, option="ERROR"):	#RAMの読み出し
		if type(id) is not list:
			id = [id]
		if B3mClass.__checkRange(id, B3mClass.MAX_ID, B3mClass.MIN_ID) is False:
			print "id range error"
			return False,False
		if B3mClass.__checkRange(address, 0xFF, 0x00) is False:
			print "address range error"
			return False,False
		if B3mClass.__checkRange(length, 0xFA, 0x01) is False:
			print "length range error"
			return False,False
		txBuf = [0x03, B3mClass.__optionToCmd(option)] + id + [address, length]
		rxBuf = self.__synchronize(txBuf, length+5)
		if (rxBuf is False):
			print "rx error "
			return False,False
		reRam = rxBuf[4:-1]
		reStatus = rxBuf[2]
		return reRam, reStatus

	#シングル/マルチ
	#data次元数 = idの次元数+1		id=[1,2,3]	data=[[10,20], [100,200], [1000,2000]]
	def writeCmd(self, id, address, data, option="ERROR"):	#RAMへの書き込み
		if type(id) is not list:
			id = [id]
		if type(data) is not list:
			data=[data]
		if B3mClass.__checkRange(id, B3mClass.MAX_ID, B3mClass.MIN_ID) is False:
			print "id range error"
			return False, False
		if B3mClass.__checkRange(address, 0xFF, 0x00) is False:
			print "address range error"
			return False, False
		if B3mClass.__checkRange(data, 0xFF, 0x00) is False:
			print "data range error"
			return False, False
		if len(id)==1:	#シングル
			txBuf = [0x04, B3mClass.__optionToCmd(option), id[0]] + flatten(data) + [address, 0x01]
			rxBuf = self.__synchronize(txBuf, 5)
			if (rxBuf is False):
				return False, False
			reStatus = rxBuf[2]
			return True, reStatus
		else:	#マルチ
			if (len(id)!=len(data)):
				return False, False
			id_data = []
			for i in range(len(id)):
				id_data += [id[i]] + data[i]
			txBuf = [0x04] + [ B3mClass.__optionToCmd(option)] + id_data + [address] + [len(id)]
			rxBuf = self.__synchronize(txBuf, 0)
			if (rxBuf is False):
				return False, False
			return True, True

	#シングル/マルチ 返値なし
	def resetCmd(self, id, time=0, option="ERROR"):
		if type(id) is not list:
			id = [id]
		if B3mClass.__checkRange(id, B3mClass.MAX_ID, B3mClass.MIN_ID) is False:
			print "id range error"
			return False
		if B3mClass.__checkRange(time, 25500, 0):
			print "time range error"
			return False
		txBuf = [0x05, B3mClass.__optionToCmd(option)] + id + [time&0xFF, time>>8]
		rxBuf = self.__synchronize(txBuf, 0)
		if (rxBuf is False):
			return False
		return True

	def positionCmd(self, id, pos, time=0, option="ERROR"):
		time = int(time*1000)	#s → ms
		if type(id) is not list:
			id = [id]
		if type(pos) is not list:
			pos = [pos]
		if B3mClass.__checkRange(id, B3mClass.MAX_ID, B3mClass.MIN_ID) is False:
			print "id range error"
			return False, False
		if B3mClass.__checkRange(pos, 32000, -32000) is False:
			print "position range error"
			return False, False
		if B3mClass.__checkRange(time, 655335, 0) is False:
			print "time range error"
			return False, False
		if (len(id) != len(pos)):
			print "id and pos length error"
			return False, False
		if B3mClass.__checkOption(option) is False:
			return False, False

		id_pos=[]
		for i in range(len(id)):
			id_pos += [id[i]]
			pos[i] = B3mClass.__signedToUnsigned(pos[i], 2)
			id_pos +=  B3mClass.__disassemblyByte(pos[i], 2)
		txBuf = [0x06, B3mClass.__optionToCmd(option)] + id_pos + B3mClass.__disassemblyByte(int(time),2)
		if len(id)==1:	#シングルモード
			rxBuf = self.__synchronize(txBuf, 7)
			# rxBuf = self.__unsynchronize(txBuf, 7)
			if (rxBuf is False):
				return False, False
			reStatus = rxBuf[2]
			rePos = B3mClass.__assemblyByte(rxBuf[4:6])
			rePos = B3mClass.__unsignedToSigned(rePos, 2)
			return rePos, reStatus
		else:	#マルチモード
			rxBuf = self.__synchronize(txBuf, 0)
			if (rxBuf is False):
				return False, False
			return True, True

#非コマンド>>>
	def setMode(self, id, mode, option="ERROR"):
		address = 0x28
		if not mode in {"POSITION", "SPEED", "VELOCITY", "CURRENT", "TORQUE", "FEEDFORWARD", "NORMAL", "FREE", "HOLD"}:
			return False
		if mode == "NORMAL":
			data = 0b0000
		elif mode == "FREE":
			data = 0b0010
		elif mode == "HOLD":
			data = 0b0011
		else:
			re = self.setMode(id, "HOLD")
			if re is False:
				return False
			if mode == "POSITION":	#以下は"NORMAL"も同時に書き込んでいる
				data = 0b0000
			elif mode == "SPEED" or  mode == "VELOCITY":
				data = 0b0100
			elif mode == "CURRENT" or mode == "TORQUE":
				data = 0b1000
			elif mode == "FEEDFORWARD":
				data = 0b1100

		if type(id) is not list:	#シングルモード
			rxBuf = self.writeCmd(id, address, data, option)
		elif len(id)==1:					#シングルモード
			rxBuf = self.writeCmd(id[0], address, data[0], option)
		else:								#マルチモード
			rxBuf = self.writeCmd([id]*len(id), address, [[data]*len(id)], option)
		return rxBuf

	def setTrajectoryType(self, id, _type, option="ERROR"):
		address = 0x29
		if _type == "NORMAL":
			data = 0
		elif _type == "EVEN":
			data = 1
		elif _type == "THIRDPOLY":
			data = 3
		elif _type == "FORTHPOLY":
			data = 4
		elif _type == "FIFTHPOLY":
			data = 5
		else:
			return False
		if type(id) is not list:	#シングルモード
			rxBuf = self.writeCmd(id, address, data, option)
		elif len(id)==1:					#シングルモード
			rxBuf = self.writeCmd(id[0], address, data[0], option)
		else:								#マルチモード
			rxBuf = self.writeCmd([id]*len(id), address, [[data]*len(id)], option)
		return rxBuf	#Status

	def setRam(self, id, data, property, option="ERROR"):
		if B3mClass.__checkRange(data, self.MEMORY_MAP[property][3] , self.MEMORY_MAP[property][2]) is False:
			print "data range error"
			return False, False
		address = self.MEMORY_MAP[property][0]
		if self.MEMORY_MAP[property][1] == "char":
			data = B3mClass.__signedToUnsigned(data, 1)		#符号付き→符号なしに変換
			txData = B3mClass.__disassemblyByte(data, 1)	#数値を1byteづつのリストに変換
		elif self.MEMORY_MAP[property][1] == "byte":
			txData = B3mClass.__disassemblyByte(data, 1)
		elif self.MEMORY_MAP[property][1] == "short":
			data = B3mClass.__signedToUnsigned(data, 2)
			txData = B3mClass.__disassemblyByte(data, 2)
		elif self.MEMORY_MAP[property][1] == "ushort":
			txData = B3mClass.__disassemblyByte(data, 2)
		elif self.MEMORY_MAP[property][1] == "long":
			data = B3mClass.__signedToUnsigned(data, 4)
			txData = B3mClass.__disassemblyByte(data, 4)
		elif self.MEMORY_MAP[property][1] == "ulong":
			txData = B3mClass.__disassemblyByte(data, 4)
		(Success, reStatus) = self.writeCmd(id, address, txData, option)
		return (Success, reStatus)

	def getRam(self, id, property, option="ERROR"):
		address = self.MEMORY_MAP[property][0]
		if self.MEMORY_MAP[property][1] == "char":
			(rxRam, reStatus) = self.readCmd(id, address, 1, option)
			reData = B3mClass.__assemblyByte( rxRam )	#char（長さ１）では、要素取り出すだけ
			reData = B3mClass.__unsignedToSigned(reData, 1)
		elif self.MEMORY_MAP[property][1] == "byte":
			(rxRam, reStatus) = self.readCmd(id, address, 1, option)
			reData = rxRam[0]
		elif self.MEMORY_MAP[property][1] == "short":
			(rxRam, reStatus) = self.readCmd(id, address, 2, option)
			reData = B3mClass.__assemblyByte( rxRam )
			reData = B3mClass.__unsignedToSigned(reData, 2)
		elif self.MEMORY_MAP[property][1] == "ushort":
			(rxRam, reStatus) = self.readCmd(id, address, 2, option)
			reData = B3mClass.__assemblyByte( rxRam )
		elif self.MEMORY_MAP[property][1] == "long":
			(rxRam, reStatus) = self.readCmd(id, address, 4, option)	#書き込み
			reData = B3mClass.__assemblyByte( rxRam )						#返り値(1byteづつのリスト)を結合して1つの値に変換
			reData = B3mClass.__unsignedToSigned(reData, 4)					#符号なし→符号付きに変換
		elif self.MEMORY_MAP[property][1] == "ulong":
			(rxRam, reStatus) = self.readCmd(id, address, 4, option)
			reData = B3mClass.__assemblyByte( rxRam )
		return reData, reStatus

	def setNewId(self, currentId, newId):
		re1 = self.loadCmd(currentId)
		re2 = self.setRam(currentId, newId, "ID")
		re3 = self.saveCmd(newId)
		return ( re1[0]==True and re2[0]==True and re3[0]==True )







# aaa=B3mClass("/dev/ttyUSB0", 1500000)
# aaa.begin()

# print "free"
# print aaa.setMode(0,"FREE")

# print "nomal mode"
# print aaa.setMode(0,"NORMAL")

# print "pos"
# print aaa.positionCmd(0,0)
# import time
# time.sleep(2)
# print aaa.positionCmd(0,20000)
# time.sleep(2)
"""
#終了時間制御をする例
print aaa.setTrajectoryType(0,"EVEN")
print aaa.setMode(0,"POSITION")
print aaa.positionCmd(0,32000, 10000)
"""
"""
#トルクを設定する例
print aaa.setMode(0,"TORQUE")
print aaa.setRam(0,100,"DesiredTorque")
"""