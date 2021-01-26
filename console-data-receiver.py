#!/usr/bin/env python3

import numpy as np
from numpy import pi
import socket
import struct
import pickle
import time
import os
import json
import argparse

############################### Arguments parser #############################################

parser = argparse.ArgumentParser(description='console-data-receiver')
parser.add_argument('--port',
					help="This is a second port generated by 01_socat.sh")

args = parser.parse_args()
port = args.port


if port is None:
	print("Error: please specify second port of socat generated")
	quit()

################################### PORT and SOCKET #############################################

CUBEPILOT_PORT = 5555
cube_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

DOOR_PORT = 6666
door_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ROBOTARM_PORT = 7777
# arm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


################################################################################################
def read_socat(term):
	read = term.readline().decode()
	return read


#BUTTONS:
# 0 A   		AUTO
# 1 B   		HOLD
# 2 X   		MANUAL
# 3 Y   		ARMDISARM
# 4 LB  		left 45
# 5 RB  		right 45 
# 6 LT  		left 90
# 7 RT  		right 90
# 16 Logicool   turn 180

# 8 Back        
# 9 Start
# 10 Analog left BtnnotRecvData
# 11 Analog right Btn

#AXES
# Axis1 left stick up down
# Axis2 right stick left right


SOCAT_PORT = port

####  This is a data packet that we will get from browser (as one single json)####
## { "ID" : "Logicool Gamepad F310 (STANDARD GAMEPAD Vendor: 046d Product: c21d)", 
## "TIMESTAMP" : 203376.22, "INDEX" : 0, "MAPPING" : "standard", 
## "AXES" : { "#00" : 0, "#01" : 0, "#02" : 0, "#03" : 0 }, 
## "BUTTONS" : { "#00" : 0, "#01" : 0, "#02" : 0, "#03" : 0, "#04" : 0, "#05" : 0, "#06" : 0, "#07" : 0, "#08" : 0, "#09" : 0, "#10" : 0, "#11" : 0, "#12" : 0, "#13" : 0, "#14" : 0, "#15" : 0, "#16" : 0 }, 
## "TOTAL_AXES" : 4, "TOTAL_BUTTONS" : 17, 
## "MODE": "HOLD", "TURN_DIR" : "NONE", "FORWARD" : 0, "LEFT" : 0, "RIGHT" : 0, "ARMED" : 0}

## The gamepad behavior on avatar/test4 is, the data will keep sending when we are pressing for buttons and joystick
## for joysticks, this is convenient for us
## for buttons, we need to make a lock not to enter condition many times  

cube_pilot_data = {
					'MODE' : 'HOLD',
					'TURN_DIR' : None,
					'FORWARD' : 0,
					'LEFT' : 0,
					'RIGHT' : 0,
					'ARMED' : 0,
					'STR_VAL': 0.0,
					'THR_VAL' : 0.0,
}

door_data = "CLOSE"
prev_door_data = door_data

# robot_arm_data = {
# 					'ARM_X' : 350,
# 					'ARM_Z' : 0,
# }

# avatar_face_data = {
# 					'faceToggle' : False,
# 					'FACE' : 'sleep'
# }

arm_state = 0
prev_arm_state = 0
screen_prev_arm_state = 0
button_prev_arm_state = 0
# faceToggle = False
# FACE = 'sleep'
# prev_FACE = 'sleep'
## This unlock flag will help each condition to execute only one time
## even the user keep pressing the buttons
unlock = True

# pre_ARM_X = 350
# pre_ARM_Z = 0

while True:

	with open(SOCAT_PORT, "rb", buffering=0) as term:
	# with serial.Serial(PORT, timeout=0.1) as term:
	
		startTime = time.time()

		try:
			str_buffer = read_socat(term)
			print(str_buffer)
			dec = json.loads(str_buffer)

			#### Gamepad is plugged ####
			if len(str_buffer) > 400:
				### Mode Changing ###
				if (dec["BUTTONS"]["#02"]) == 1:
					if unlock:
						print("MANUAL")
						cube_pilot_data['MODE'] = "MANUAL"
						unlock = False
				elif (dec["BUTTONS"]["#01"]) == 1:
					if unlock:
						print("HOLD")
						cube_pilot_data['MODE'] = "HOLD"
						unlock = False
				elif (dec["BUTTONS"]["#00"]) == 1:
					if unlock:
						print("AUTO")
						cube_pilot_data['MODE'] = "AUTO"
						unlock = False
				
				### Arm / Disarm ###
				elif dec["BUTTONS"]["#03"] == 1 and screen_prev_arm_state == 1:
					if unlock:
						print("ARMDISARM")
						if prev_arm_state == 1:
							print("here")
							cube_pilot_data['ARMED'] = 0
							prev_arm_state = 0
						else:
							cube_pilot_data['ARMED'] = 1
							prev_arm_state = 1
						unlock = False
				
				### Direction Control ###
				elif (dec["BUTTONS"]["#04"] == 1):
					if unlock:
						print("TURNLEFT45")
						cube_pilot_data['MODE'] = "GUIDED"
						cube_pilot_data['TURN_DIR'] = "LEFT45" 
						unlock = False

				elif (dec["BUTTONS"]["#06"] == 1):
					if unlock:
						print("TURNLEFT90")
						cube_pilot_data['MODE'] = "GUIDED"
						cube_pilot_data['TURN_DIR'] = "LEFT90"
						unlock = False

				elif (dec["BUTTONS"]["#05"] == 1):
					if unlock:
						print("TURNRIGHT45")
						cube_pilot_data['MODE'] = "GUIDED"
						cube_pilot_data['TURN_DIR'] = "RIGHT45"
						unlock = False

				elif (dec["BUTTONS"]["#07"] == 1):
					if unlock:
						print("TURNRIGHT90")
						cube_pilot_data['MODE'] = "GUIDED"
						cube_pilot_data['TURN_DIR'] = "RIGHT90"
						unlock = False

				elif (dec["BUTTONS"]["#16"] == 1):
					if unlock:
						print("TURN180")
						cube_pilot_data['MODE'] = "GUIDED"
						cube_pilot_data['TURN_DIR'] = "U180"
						unlock = False


				else:
					unlock = True

				### Joystick ###
				STR_val = dec["AXES"]["#02"]
				THR_val = (-1)*dec["AXES"]["#01"]

				if cube_pilot_data['MODE'] == "MANUAL":
					cube_pilot_data['STR_VAL'] = STR_val
					cube_pilot_data['THR_VAL'] = THR_val
				else:
					cube_pilot_data['STR_VAL'] = 0.0
					cube_pilot_data['THR_VAL'] = 0.0


				# if dec["DOOR"] == 1:
				# 	door_data = "OPEN"
				# else:
				# 	door_data = "CLOSE"

				# ### Arm / Disarm ###
				# if dec['ARMED'] != button_prev_arm_state:
				# 	if dec['ARMED'] == 1:
				# 		print("WITHGAMEPAD by console button : ARM")
				# 		cube_pilot_data['ARMED'] = 1
				# 	else:
				# 		print("WITHGAMEPAD by console button  : DISARM")
				# 		cube_pilot_data['ARMED'] = 0

				# 	screen_prev_arm_state = dec['ARMED']

				# button_prev_arm_state = dec['ARMED']

		
				### Reset ###
				cube_pilot_data['FORWARD'] = 0
				cube_pilot_data['LEFT'] = 0
				cube_pilot_data['RIGHT'] = 0

			
			#### Without gamepad / only buttons on browser ####
			else:

				if (dec["MODE"] == "MANUAL"):
					print("WITHOUT_GAMEPAD : MANUAL")
					cube_pilot_data['MODE'] = "MANUAL"

				elif (dec["MODE"] == "AUTO"):
					print("WITHOUT_GAMEPAD : AUTO")
					cube_pilot_data['MODE'] = "AUTO"

				elif (dec["MODE"] == "HOLD"):
					print("WITHOUT_GAMEPAD : HOLD")
					cube_pilot_data['MODE'] = "HOLD"
				else:
					cube_pilot_data['MODE'] = None


				### Arm / Disarm ###
				if dec['ARMED'] == 1:
					print("WITHOUT_GAMEPAD : ARM")
					cube_pilot_data['ARMED'] = 1
					# avatar_face_data['FACE'] = 'blink'
					# FACE = 'blink'
				else:
					print("WITHOUT_GAMEPAD : DISARM")
					cube_pilot_data['ARMED'] = 0
					# avatar_face_data['FACE'] = 'sleep'
					# FACE = 'sleep'

				screen_prev_arm_state = dec['ARMED']
				prev_arm_state = dec['ARMED']
				# prev_FACE = FACE

				if dec["TURN_DIR"] == "TURNLEFT45":
					print("WITHOUT_GAMEPAD : TURNLEFT45")
					cube_pilot_data['MODE'] = "GUIDED"
					cube_pilot_data['TURN_DIR'] = "LEFT45"

				elif dec["TURN_DIR"] == "TURNLEFT90":
					print("WITHOUT_GAMEPAD : TURNLEFT90")
					cube_pilot_data['MODE'] = "GUIDED"
					cube_pilot_data['TURN_DIR'] = "LEFT90"

				elif dec["TURN_DIR"] == "TURNRIGHT45":
					print("WITHOUT_GAMEPAD : TURNRIGHT45")
					cube_pilot_data['MODE'] = "GUIDED"
					cube_pilot_data['TURN_DIR'] = "RIGHT45"

				elif dec["TURN_DIR"] == "TURNRIGHT90":
					print("WITHOUT_GAMEPAD : TURNRIGHT90")
					cube_pilot_data['MODE'] = "GUIDED"
					cube_pilot_data['TURN_DIR'] = "RIGHT90"

				elif dec["TURN_DIR"] == "TURN180":
					print("WITHOUT_GAMEPAD : TURN180")
					cube_pilot_data['MODE'] = "GUIDED"
					cube_pilot_data['TURN_DIR'] = "U180"
				else:
					cube_pilot_data['TURN_DIR'] = None


				### Go straight ###
				if dec["FORWARD"] != 0:	
					print("GOFORWARD")
					cube_pilot_data['MODE'] = "GUIDED"
					cube_pilot_data['FORWARD'] = int(dec["FORWARD"])
				else:
					cube_pilot_data['FORWARD'] = 0


				if dec["LEFT"] != 0:
					print("GOLEFT")
					cube_pilot_data['MODE'] = "GUIDED"
					cube_pilot_data['LEFT'] = int(dec["LEFT"])
				else:
					cube_pilot_data['LEFT'] = 0


				if dec["RIGHT"] != 0:
					print("GORIGHT")
					cube_pilot_data['MODE'] = "GUIDED"
					cube_pilot_data['RIGHT'] = int(dec["RIGHT"])
				else:
					cube_pilot_data['RIGHT'] = 0

				if dec["DOOR"] == 1:
					door_data = "OPEN"
				else:
					door_data = "CLOSE"




				# robot_arm_data["ARM_X"] = dec["ARM_X"]
				# robot_arm_data["ARM_Z"] = dec["ARM_Z"]

			cube_pilot_packets = pickle.dumps(cube_pilot_data)
			cube_sock.sendto(cube_pilot_packets,("127.0.0.1",CUBEPILOT_PORT))

			## reset ##
			cube_pilot_data['TURN_DIR'] = None
			if cube_pilot_data['MODE'] == 'GUIDED' or cube_pilot_data['MODE'] == 'AUTO':
				cube_pilot_data['MODE'] = None

			## send when data got changed only, for door control
			if prev_door_data != door_data:
				door_packets = pickle.dumps(door_data)
				door_sock.sendto(door_packets,("127.0.0.1",DOOR_PORT))

			# ## send when data got changed only, for robot arm
			# if (robot_arm_data["ARM_X"] != pre_ARM_X or robot_arm_data["ARM_Z"] != pre_ARM_Z):
			# 	robot_arm_packets = pickle.dumps(robot_arm_data)
			# 	arm_sock.sendto(robot_arm_packets,("127.0.0.1",ROBOTARM_PORT))

			# pre_ARM_X = robot_arm_data["ARM_X"]
			# pre_ARM_Z = robot_arm_data["ARM_Z"]

			prev_door_data = door_data

		except KeyboardInterrupt:
			quit()
		except Exception as e:
			print("From console data receiver loop")
			print(e)
			print(str_buffer)
			print("Failed to parse")
			pass

		# period = time.time() - startTime
		# print(period)
