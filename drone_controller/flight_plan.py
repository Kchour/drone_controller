"""
Original Author: unnamed-idea
Modifications: Kchour 

Modified to work with Parrot's GroundSDK FlightPlan:
https://developer.parrot.com/docs/mavlink-flightplan/

QGC WPL <VERSION>
<INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> <PARAM5/X/LONGITUDE> <PARAM6/Y/LATITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>


NOTE: not everything has been implemented. Only takeoff, waypoint, and land have been implemented.
"""

from math import sin,cos,radians,pi
from enum import IntEnum, auto
from dataclasses import dataclass, fields
from io import BytesIO

NAN = "nan"

class Cmd(IntEnum):
	MAV_CMD_NAV_WAYPOINT = 16
	MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
	MAV_CMD_NAV_LAND = 21
	MAV_CMD_NAV_TAKEOFF = 22
	MAV_CMD_NAV_DELAY = 93
	MAV_CMD_CONDITION_DELAY = 112
	MAV_CMD_DO_CHANGE_SPEED = 178
	MAV_CMD_DO_SET_ROI_LOCATION = 195
	MAV_CMD_DO_SET_ROI_NONE = 197
	MAV_CMD_DO_SET_ROI = 201
	MAV_CMD_DO_MOUNT_CONTROL = 205
	MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206
	MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214
	MAV_CMD_IMAGE_START_CAPTURE = 2000
	MAV_CMD_IMAGE_STOP_CAPTURE = 2001
	MAV_CMD_VIDEO_START_CAPTURE = 2500
	MAV_CMD_VIDEO_STOP_CAPTURE = 2501
	MAV_CMD_PANORAMA_CREATE = 2800
	MAV_CMD_SET_VIEW_MODE = 50000
	MAV_CMD_SET_STILL_CAPTURE_MODE = 50001

class MAVViewModeType(IntEnum):
	VIEW_MODE_TYPE_ABSOLUTE = 0
	VIEW_MODE_TYPE_CONTINUE = 1
	VIEW_MODE_TYPE_ROI = 2
	VIEW_MODE_TYPE_TRAVELING = 3

class MAVStillCaptureModeType(IntEnum):
	STILL_CAPTURE_MODE_TYPE_TIMELAPSE = 0
	STILL_CAPTURE_MODE_TYPE_GPS_POSITION = 1

@dataclass
class MavlinkMsg:
	INDEX: int
	CURRENT_WP: int
	COORD_FRAME: int #2: MAV_FRAME_MISSION, 3: MAV_FRAME_GLOBAL_RELATIVE_ALT
	COMMAND: int
	PARAM1: float
	PARAM2: float
	PARAM3: float
	PARAM4: float
	PARAM5: float
	PARAM6: float
	PARAM7: float
	AUTOCONTINUE: int = 1

	def __repr__(self):
		expr = ""
		for f in fields(self):
			expr = "".join((expr, str(getattr(self,f.name)), "\t"))
		expr = "".join((expr,"\n"))
		return expr

class automission(object):
	#docstring for automission
	def __init__(self,vehicle_type):

		super(automission, self).__init__()
		assert vehicle_type=='copter'

		self.mlist=[] #each element of the array represents a command, ie waypoint, with its parameters
		#these two lines are by default, exists every mission planner file
		self.mlist.append('QGC WPL 120\n')
		# self.mlist.append('0	1	0	0	0	0	0	0	0	0	0	1\n')
		
		# increment counter for mavlink index
		self.counter=0

		# camera roi index
		self.roi_counter = -1

	def write(self,name='test.mavlink'): 		
		# saves final command list mlist as txt file. 
		# Missionplanner can direcly open this text document in flight plan / load WP file button
		with open(name, "w") as text_file:
			text_file.write(str(self))        
		print("Wrote mavlink flight plan to file")
	
	def as_text(self):
		return str(self)

	def __repr__(self):
		expr = ""
		for m in self.mlist:
			expr = "".join((expr, str(m)))	
		return expr

	def __len__(self):
		return self.counter + 1

	def get_roi_ind(self):
		return self.roi_counter

	################################################################################
	
	#every parameter list begins with 'INDEX, 0,3,' and ends with ',1'
	
	def waypoint(self,lat,lon,alt,delay=0):	
		# waypoint_id=16
		# self.param_to_mcommand(0,3,waypoint_id,delay,0,0,0,lat,lon,alt,1)
		
		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_NAV_WAYPOINT.value,
					PARAM1=delay, PARAM2=0, PARAM3=0, PARAM4=0, PARAM5=lat, PARAM6=lon, PARAM7=alt)

		self.mlist.append(msg)
		self.counter += 1

	# def takeoff(self,alt, lat=0,lon=0, yaw=0):
	def takeoff(self):
		# takeoff_id=22
		# # Changed CURRENT_WP from 0 to 1
		# self.param_to_mcommand(1,3,takeoff_id,angle,0,0,0,lat,lon,alt,1)
		# # self.param_to_mcommand(0,3,takeoff_id,angle,0,0,0,lat,lon,alt,1)

		# msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=1, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_NAV_TAKEOFF.value,
		# 			PARAM1=0, PARAM2=0, PARAM3=0, PARAM4=0, PARAM5=lat, PARAM6=lon, PARAM7=alt)

		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=1, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_NAV_TAKEOFF.value,
					PARAM1=0, PARAM2=0, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)
		self.mlist.append(msg)
		self.counter += 1

	# def land(self,lat=0,lon=0,alt=0):
	def land(self):
		# landid=21
		# self.param_to_mcommand(0,3,landid,0,0,0,0,lat,lon,alt,1)

		# msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_NAV_LAND.value,
		# 			PARAM1=0, PARAM2=0, PARAM3=0, PARAM4=0, PARAM5=lat, PARAM6=lon, PARAM7=alt)

		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_NAV_LAND.value,
					PARAM1=0, PARAM2=0, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)
		self.mlist.append(msg)
		self.counter += 1

	def do_change_speed(self,speed, speed_type=1):
		# do_change_speed_id=178
		# self.param_to_mcommand(0,3,do_change_speed_id,speed,0,0,0,0,0,0,1)
		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_DO_CHANGE_SPEED.value,
					PARAM1=speed_type, PARAM2=speed, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)

		self.mlist.append(msg)
		self.counter += 1

	# def start_take_picture(self, interval: float, capture_count=0, sequence_number=1):
	def start_take_picture(self, interval: float, capture_count=0, formatting: int=12):
		""" Start image capture sequence
		Params
	
		1: Interval:  Desired elapsed time between two consecutive pictures (s). If 0, Interval and Capture Count are taken from the latest MAV_CMD_SET_STILL_CAPTURE_MODE
		2: Capture count 	  Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE  
		3: formatting: 	  12: rect jpeg, 13: full frame jpeg, 14: full frame jpeg + dng

		all others are ignored

		"""
		# msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_IMAGE_START_CAPTURE.value,
		# 			PARAM1=0, PARAM2=interval, PARAM3=capture_count, PARAM4=sequence_number, PARAM5=0, PARAM6=0, PARAM7=0)

		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_IMAGE_START_CAPTURE.value,
					PARAM1=interval, PARAM2=capture_count, PARAM3=formatting, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)

		self.mlist.append(msg)
		self.counter += 1

	def stop_take_picture(self):
		"""Params
		All empty 	
		
		"""
		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_IMAGE_STOP_CAPTURE.value,
					PARAM1=0, PARAM2=0, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)

		self.mlist.append(msg)
		self.counter += 1

	def delay(self, delay: float):
		"""Delay the next navigation commands. Same as MAV_CMD_CONDITION_DELAY.

		Params
		delay in seconds

		"""
		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=2, COMMAND=Cmd.MAV_CMD_NAV_DELAY.value,
					PARAM1=delay, PARAM2=0, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)

		self.mlist.append(msg)
		self.counter += 1
	
	# def image_capture_mode(self, mode: MAVStillCaptureModeType, interval: float):
	def image_capture_mode(self, mode: MAVStillCaptureModeType, interval: float):
		"""Parrot specific command, set the still capture mode

		# currently trying v1
		1: Capture mode
		2: Interval

		# """
		# msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_SET_STILL_CAPTURE_MODE.value,
		# 			PARAM1=mode, PARAM2=interval, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)

		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_SET_STILL_CAPTURE_MODE.value,
					PARAM1=mode, PARAM2=interval, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)
		self.mlist.append(msg)
		self.counter += 1

	def start_video_capture(self): 
		"""Start video recording

		Seems like everything is ignored

		# """
		# msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_SET_STILL_CAPTURE_MODE.value,
		# 			PARAM1=mode, PARAM2=interval, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)

		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_VIDEO_START_CAPTURE.value,
					PARAM1=0, PARAM2=0, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)
		self.mlist.append(msg)
		self.counter += 1
	
	def stop_video_capture(self):
		"""Stop video recording

		Seems like everything is ignored

		# """
		# msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_SET_STILL_CAPTURE_MODE.value,
		# 			PARAM1=mode, PARAM2=interval, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)

		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_VIDEO_STOP_CAPTURE.value,
					PARAM1=0, PARAM2=0, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)
		self.mlist.append(msg)
		self.counter += 1	

	def set_cam_roi(self, lat, lon, alt):
		"""Set front camera region of interest
		
		params:
		1) mode (default is 3 I believe?). 
		
		5: lat
		6: lon
		7: alt

		all else is ignored

		"""
		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_DO_SET_ROI.value,
					PARAM1=3, PARAM2=0, PARAM3=0, PARAM4=0, PARAM5=lat, PARAM6=lon, PARAM7=alt)
		self.mlist.append(msg)
		self.counter += 1	
		self.roi_counter +=1
	
	def set_view_mode(self, mode:MAVViewModeType, roi_index: int):
		"""Set view mode. parrot specific command

		1) view mode type (must be from MAV_VIEW_MODE_TYPE)
		2) ROI index if view mode type is VIEW_MODE_TYPE_ROI. If ROI does not exist, View Mode is set to Absolute

		all other params ignored

		"""
		msg = MavlinkMsg(INDEX=self.counter, CURRENT_WP=0, COORD_FRAME=3, COMMAND=Cmd.MAV_CMD_SET_VIEW_MODE.value,
					PARAM1=mode, PARAM2=roi_index, PARAM3=0, PARAM4=0, PARAM5=0, PARAM6=0, PARAM7=0)
		self.mlist.append(msg)
		self.counter += 1	
	
	##############################################################################################################
	# Below has not been tested yet

	# def loiter_unlim(self,lat,lon,alt):	
	# 	loiter_unlimid=17
	# 	self.param_to_mcommand(0,3,loiter_unlimid,0,0,0,0,lat,lon,alt,1)
	
	# def do_set_roi(self,lat,lon,alt=0):	
	# 	do_set_roi_id=201
	# 	self.param_to_mcommand(0,3,do_set_roi_id,0,0,0,0,lat,lon,alt,1)

	# def rtl(self):	
	# 	rtl_id=20
	# 	self.param_to_mcommand(0,3,rtl_id,0,0,0,0,0,0,0,1)

	# def spline_waypoint(self,lat,lon,alt,delay=0):
	# 	spline_waypoint_id=82
	# 	self.param_to_mcommand(0,3,spline_waypoint_id,delay,0,0,0,lat,lon,alt,1)

	# def loiter_time(self,time,lat=0,lon=0,alt=0):
	# 	loiter_time_id=19
	# 	self.param_to_mcommand(0,3,loiter_time_id,time,0,0,0,lat,lon,alt,1)
	
	# def loiter_turns(self,turn,direction,lat=0,lon=0,alt=0):
	# 	loiter_turns_id=18
	# 	self.param_to_mcommand(0,3,loiter_turns_id,turn,0,direction,0,lat,lon,alt,1)

	# def loiter_unlim(self,lat=0,lon=0,alt=0):
	# 	loiter_unlim_id=17
	# 	self.param_to_mcommand(0,3,loiter_unlim_id,0,0,0,0,lat,lon,alt,1)

	# def condition_delay(self,time):
	# 	condition_delay_id=112
	# 	self.param_to_mcommand(0,3,condition_delay_id,time,0,0,0,0,0,0,1)

	# def condition_distance(self,dist):
	# 	condition_distance_id=114
	# 	self.param_to_mcommand(0,3,condition_distance_id,dist,0,0,0,0,0,0,1)

	# def condition_yaw(self,deg,rel_abs,dir=0):
	# 	condition_yaw_id=115
	# 	self.param_to_mcommand(0,3,condition_yaw_id,deg,0,dir,rel_abs,0,0,0,1)

	# def do_jump(self,wp_number,repeat):
	# 	do_jump_id=177
	# 	self.param_to_mcommand(0,3,do_jump_id,wp_number,repeat,0,0,0,0,0,1)

	# def do_set_home(self,current=1,lat=0,lon=0):
	# 	do_set_home_id=179
	# 	self.param_to_mcommand(0,3,do_set_home_id,current,0,0,0,lat,lon,0,1)

	# def do_digicam_control(self):
	# 	do_digicam_control_id=203
	# 	self.param_to_mcommand(0,3,do_digicam_control_id,0,0,0,0,0,0,0,1)
		
	####################################################################################





"""
	def meter_to_coord(self,meter,angle,lat,lon):

		m_x=meter*cos(radians(angle))
		m_y=meter*sin(radians(angle))
		R=6378137
		yLat = m_y/R
		xLon = m_x/(R*cos(radians(lat)))
		lat_new= lat + yLat * 180/pi
		lon_new=lon+xLon*180/pi
		return lat_new,lon_new

		"""

if __name__ == "__main__":
	test = MavlinkMsg(0,1,2,3,4,5,6,7,8,9,10)
	print(test)

	my_mission = automission("copter")
	my_mission.takeoff(15)
	my_mission.waypoint(48.879000, 2.366549, 20.0)
	my_mission.waypoint(48.879139, 2.367296, 10.0)
	my_mission.land(48.879139, 2.367296, 0)
	print(my_mission)

	my_mission.write("test.mavlink")
	with open("test.mavlink", 'rb') as data:
		print("opened file")