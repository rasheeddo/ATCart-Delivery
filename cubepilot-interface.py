#!/usr/bin/env python3

import numpy as np
from numpy import pi
import socket
import struct
import pickle
import time
import os
import json
os.environ["MAVLINK20"] = "2"
from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil

################################### PORT and SOCKET #############################################

CUBEPILOT_PORT = 5555
cube_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cube_sock.bind(("0.0.0.0", CUBEPILOT_PORT))
cube_sock.setblocking(0)

WP_PORT = 7777
wp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
wp_sock.bind(("0.0.0.0", WP_PORT))
wp_sock.setblocking(0)

GPS_PUB_PORT = 8888
gps_pub_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

LIDAR_PORT = 3101
lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
lidar_sock.bind(("0.0.0.0", LIDAR_PORT))
lidar_sock.setblocking(0)

###################################### Global Params ##############################################
vehicle = None
is_vehicle_connected = False

global rover_status
global current_mode

cur_lat = 0.0
cur_lon = 0.0
cur_yaw = 0.0
gps_status = 0

rover_status ={
				"lat" : cur_lat, 
				"lng" : cur_lon,
				"yaw" : cur_yaw, 
				"status" : gps_status}

##### Obstacle distance params #####
# Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
# See here: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
min_distance = 20 	#120  # In cm, this is the distance from lidar to cart's nose plus a litte bit for safety
max_distance = 1000	#1500  # In cm, I found the lidar never give the value over than 15meters
distances_array_length = 72
angle_offset = -60.0	#-90.0	# deg, the first index of distances is -90.0deg
increment_f  = 1.6		#2.5		# deg, increment angle of our array
distances = np.ones((distances_array_length,), dtype=np.uint16) * (max_distance + 1)

################################### Functions #############################################
def vehicle_connect():
	global vehicle, is_vehicle_connected

	if vehicle == None:
		try:
			# if sim.startswith('sitl'):
			# print("Connecting to Ardupilot on SITL...")
			# vehicle = connect('udp:127.0.0.1:14551', wait_ready=True)
			# else:
			print("Connecting to Ardupilot....")
			vehicle = connect('/dev/usb_uart', wait_ready=True, baud=921600)
			
		except Exception as e:
			print(e)
			print("Device might not be found...check the udevrules")
			quit()
			time.sleep(1)

	if vehicle == None:
		is_vehicle_connected = False
		return False
	else:
		is_vehicle_connected = True
		return True

def turn(deg):
	if is_vehicle_connected == True:
		msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
			0b0000101111111111, # type_mask (only speeds enabled)
			0, 0, 0, # x, y, z positions (not used)
			0, 0, 0, # x, y, z velocity in m/s
			0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
			deg*pi/180.0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

		vehicle.send_mavlink(msg)
		# vehicle.flush()
	else:
		print("INFO: Vehicle not connected.")

def goForward(meter):
	if is_vehicle_connected == True:
		msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
			0b0000111111111000, # type_mask (only speeds enabled)
			meter, 0, 0, # x, y, z positions (not used)
			0, 0, 0, # x, y, z velocity in m/s
			0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
			0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
			
		vehicle.send_mavlink(msg)
		# vehicle.flush()
	else:
		print("INFO: Vehicle not connected.")

def goLeft(meter):
	if is_vehicle_connected == True:
		msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
			0b0000111111111000, # type_mask (only speeds enabled)
			0, -meter, 0, # x, y, z positions (not used)
			0, 0, 0, # x, y, z velocity in m/s
			0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
			0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
			
		vehicle.send_mavlink(msg)
		# vehicle.flush()
	else:
		print("INFO: Vehicle not connected.")

def goRight(meter):
	if is_vehicle_connected == True:
		msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
			0b0000111111111000, # type_mask (only speeds enabled)
			0, meter, 0, # x, y, z positions (not used)
			0, 0, 0, # x, y, z velocity in m/s
			0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
			0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
			
		vehicle.send_mavlink(msg)
		# vehicle.flush()
	else:
		print("INFO: Vehicle not connected.")


# Callback to print the location in global frame
def location_callback(self, attr_name, value):
	# global cur_lat, cur_lon
	cur_lat = value.global_frame.lat
	cur_lon = value.global_frame.lon
	# print("cur_lat: %.7f  cur_lon: %.7f" %(cur_lat, cur_lon))
	rover_status['lat'] = cur_lat
	rover_status['lng'] = cur_lon
	
	# print(value.global_frame)

def attitude_callback(self, attr_name, value):
	# global cur_yaw
	cur_yaw = value.yaw
	# print("cur_yaw: %.6f" %cur_yaw)
	rover_status['yaw'] = cur_yaw
	## range is -pi to pi, 0 is north

def gps_callback(self, attr_name, value):
	# global gps_status
	gps_status = value.fix_type
	# print("gps_status: %d" %gps_status)
	rover_status['status'] = gps_status
	# 3 = 3DFix
	# 4 = 3DGPS
	# 5 = rtkFloat
	# 6 = rtkFixed
	## range is -pi to pi, 0 is north

def readmission(aFileName):
	global cmds
	"""
	Load a mission from a file into a list. The mission definition is in the Waypoint file
	format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

	This function is used by upload_mission().
	"""
	print("\nReading mission from file: %s" % aFileName)
	# cmds = vehicle.commands
	missionlist=[]
	with open(aFileName) as f:
		for i, line in enumerate(f):
			if i==0:
				if not line.startswith('QGC WPL 110'):
					raise Exception('File is not supported WP version')
			else:
				linearray=line.split('\t')
				ln_index=int(linearray[0])
				ln_currentwp=int(linearray[1])
				ln_frame=int(linearray[2])
				ln_command=int(linearray[3])
				ln_param1=float(linearray[4])
				ln_param2=float(linearray[5])
				ln_param3=float(linearray[6])
				ln_param4=float(linearray[7])
				ln_param5=float(linearray[8])
				ln_param6=float(linearray[9])
				ln_param7=float(linearray[10])
				ln_autocontinue=int(linearray[11].strip())
				cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
				missionlist.append(cmd)
	return missionlist

def upload_mission_from_file():
	global cmds
	"""
	Upload a mission from a file. 
	"""
	path = os.getcwd()
	file_path = os.path.join(path, "MISSION.txt")
	if os.path.exists(file_path):
		#Read mission from file
		missionlist = readmission(file_path)
		# print("missionlist", missionlist)
		print("\nUpload mission from a file: %s" % file_path)
		#Clear existing mission from vehicle
		print('Clear old mission...')
		# cmds = vehicle.commands
		cmds.clear()
		#Add new mission to vehicle
		for command in missionlist:
			# print(command)
			cmds.add(command)
		
		vehicle.commands.upload()
		print('MISSION Uploaded')
		return True
	else:
		print("ERROR missing MISSION.txt file")
		return False

def change_flight_mode(_mode):

	global current_mode

	if _mode != None:
		# if _mode == "AUTO":
		# 	success = upload_mission_from_file()
		# 	if success:
		# 		print("Success")
		# 		vehicle.mode = "AUTO"
		# 	else:
		# 		print("Failed")
		# else:
		# 	if current_mode != _mode:
		# 		vehicle.mode = _mode
		# 	current_mode = _mode
		if current_mode != _mode:
			vehicle.mode = _mode
		current_mode = _mode

def send_obstacle_distance_message():
	global current_time_us, distances, min_distance, max_distance, angle_offset, increment_f
	if angle_offset is None or increment_f is None:
		print("Please call set_obstacle_distance_params before continue")
	else:
		# print("Send msg")
		msg = vehicle.message_factory.obstacle_distance_encode(
			current_time_us,    # us Timestamp (UNIX time or time since system boot)
			0,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
			distances,          # distances,    uint16_t[72],   cm
			0,                  # increment,    uint8_t,        deg
			min_distance,	    # min_distance, uint16_t,       cm
			max_distance,       # max_distance, uint16_t,       cm
			increment_f,	    # increment_f,  float,          deg
			angle_offset,       # angle_offset, float,          deg
			12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
		)

		vehicle.send_mavlink(msg)
		vehicle.flush()

############################# Initialized CUBE ##########################
print("INFO: Connecting to vehicle.")
while (not vehicle_connect()):
	pass
print("INFO: Vehicle connected.")

obstacle_distance_msg_hz = 15.0
sched = BackgroundScheduler()
sched.add_job(send_obstacle_distance_message, 'interval', seconds = 1/obstacle_distance_msg_hz)
# send_msg_to_gcs('Sending obstacle distance messages to FCU')
sched.start()

cmds = vehicle.commands

vehicle.mode = "HOLD"
current_mode = "HOLD"
vehicle.armed= False
# prev_mode = 'NONE'

vehicle.add_attribute_listener('location', location_callback)
vehicle.add_attribute_listener('attitude', attitude_callback)
vehicle.add_attribute_listener('gps_0', gps_callback)

#################################### Loop ##################################
prev_arm_state = 0
pwm_mid = 1520
STR_val = 0.0
THR_val = 0.0
got_data_time = time.time()
while True:
	# startTime = time.time()
	current_time_us = int(round(time.time() * 1000000))
	##############################################
	############## Get Console Data ##############
	##############################################
	try:
		data, addr = cube_sock.recvfrom(1024)
		# print("data len", len(data))
		data = pickle.loads(data)
		got_data_time = time.time()
	except socket.error:
		last_got_period = time.time() - got_data_time
		# print("Period {:.4f} No data arrived.. STR {:} THR {:}".format(last_got_period, STR_val, THR_val))
		if (last_got_period > 0.5) and (STR_val != 0 or THR_val != 0):
			print("DANGER...gamepad data is piling... STR was {:} THR was {:}".format(STR_val,THR_val))
			STR_val = 0.0
			THR_val = 0.0
			vehicle.channels.overrides['1'] = pwm_mid
			vehicle.channels.overrides['2'] = pwm_mid
		pass
	else:
		print(data)

		if data['ARMED'] != prev_arm_state:
			if data['ARMED'] == 1:
				vehicle.armed = True
			else:
				vehicle.armed = False

		prev_arm_state = data['ARMED']

		change_flight_mode(data['MODE'])

		if current_mode == "GUIDED":
			if data['TURN_DIR'] is not None:
				if data['TURN_DIR'] == "LEFT45":
					turn(-45)
				elif data['TURN_DIR'] == "LEFT90":
					turn(-90)
				elif data['TURN_DIR'] == "RIGHT45":
					turn(45)
				elif data['TURN_DIR'] == "RIGHT90":
					turn(90)
				elif data['TURN_DIR'] == "U180":
					turn(180)

			if data['FORWARD'] != 0:
				goForward(int(data['FORWARD']))
			elif data['LEFT'] != 0:
				goLeft(int(data['LEFT']))
			elif data['RIGHT'] != 0:
				goRight(int(data['RIGHT']))

		elif current_mode == "MANUAL":
			STR_val = data['STR_VAL']
			THR_val = data['THR_VAL']
			steering_pwm = int(round(STR_val*200 + pwm_mid))
			throttle_pwm = int(round(THR_val*250 + pwm_mid))
			vehicle.channels.overrides['1'] = steering_pwm
			vehicle.channels.overrides['2'] = throttle_pwm

	##############################################
	############### Get WPs Data ###############
	##############################################
	try:
		data, addr = wp_sock.recvfrom(1500)
	except socket.error:
		pass
	else:
		parse_data = pickle.loads(data)
		print(parse_data)
		if parse_data == "GOT_WP":
			upload_mission_from_file()
		else:
			print("Got strange string from wp_sock as %s" %(parse_data))

	##############################################
	############### Get LIDAR Data ###############
	##############################################
	try:
		data, addr = lidar_sock.recvfrom(1500)
	except socket.error:
		# distances = np.ones((distances_array_length,), dtype=np.uint16) * (max_distance + 1)
		pass
	else:
		#print(len(data))
		array_size = len(data)/4
		
		## unpack the whole packet into buff (tuple-type)
		buff = struct.unpack('%df' %(array_size), data)

		## first half of buf is angle and second half is dist
		angle_array = np.asarray(buff[0:int(array_size/2)])
		dist_array = np.asarray(buff[int(array_size/2):int(array_size)])

		## reverse array because 72 distances in OBSTACLE_DISTANCE
		## it starts from left side to right side (clockwise), but lidar scan (counter-clockwise)
		angle_array = angle_array[::-1]
		dist_array = dist_array[::-1]*100.0  # change to cm unit
		
		dist_array = dist_array.astype(np.uint16)	# convert to uint16 for AP

		for i in range(len(dist_array)):
			if dist_array[i] == 0:
				dist_array[i] = 65535 #max_distance + 1 #
			elif dist_array[i] > max_distance:
				dist_array[i] = max_distance + 1
			else:
				pass

		distances = np.copy(dist_array)

		# print(distances)


	##############################################
	############## Send to publisher #############
	##############################################
	if rover_status:
		# print("rover_status", rover_status)
		rover_status_packet = pickle.dumps(rover_status)
		gps_pub_sock.sendto(rover_status_packet,("127.0.0.1", GPS_PUB_PORT))


	## a little bit of delay, help cpu loads
	time.sleep(0.001)