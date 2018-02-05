#!/usr/bin/env python
#coding: utf-8

def printColor(str, color):
	"""heeelp
	this is printColor function
	"""
	color = color.lower()
	changeColor = { "clear":"\033[0m",
	"-":"\033[29m",
	"black":"\033[30m",
	"red":"\033[31m",
	"green":"\033[32m",
	"yellow":"\033[33m",
	"blue":"\033[34m",
	"purple":"\033[35m",
	"cyan":"\033[36m",
	"white":"\033[37m"}
	if color in changeColor:
		print changeColor[color] + str + changeColor["clear"]

def checkTrueKey(keyDict, trueKeyList):
	falseKeyList = []
	for key in keyDict.keys():
		if not (key in trueKeyList):
			falseKeyList.append(key)
			printColor("warning: msPlan is not support //"+str(key)+"// keyward", "yellow")
	return falseKeyList