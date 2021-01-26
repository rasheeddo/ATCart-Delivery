#!/usr/bin/env python3

import socket
import pickle
# import maestro
import time
from datetime import datetime
import os

def dateTime2String(stamp):
	timestamp_str = stamp.strftime("%Y-%m-%d_%H:%M:%S")

	return timestamp_str


################################### PORT and SOCKET #############################################

DOOR_PORT = 6666
door_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
door_sock.bind(("0.0.0.0", DOOR_PORT))
door_sock.setblocking(0)


################################### Loop #############################################
try:

	# ttyACM_str = "/dev/ttyACM"
	# got_dev = False
	# dev_num = 0
	# while not got_dev:
	# 	dev_str = ttyACM_str + str(dev_num)

	# 	if os.path.exists(dev_str):
	# 		got_dev = True
	# 	else:
	# 		dev_num += 1

	# print("Using %s for maestro controler" %(dev_str))

	# servo = maestro.Controller(ttyStr=dev_str)

	# ## set to middle
	# servo.setTarget(0,6000)
	# time.sleep(3)
	print("Start listening for door-control message")

	while True:

		try:
			data, addr = door_sock.recvfrom(1024)
			# print("data len", len(data))
			data = pickle.loads(data)
		except socket.error:
			pass
		else:
			if data == "OPEN":
				# servo.setTarget(0,8000)
				timestamp = datetime.now()
				timestamp_str = dateTime2String(timestamp)
				print("OPEN at %s" %(timestamp_str))
			elif data == "CLOSE":
				# servo.setTarget(0,4000)
				timestamp = datetime.now()
				timestamp_str = dateTime2String(timestamp)
				print("CLOSE at %s" %(timestamp_str))
			else:
				print("Error got strange data as   %s" %(data))

		time.sleep(0.001)

except Exception as e:
	print("ERROR as %s" %(e))
	# servo.close()