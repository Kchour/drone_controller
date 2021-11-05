"""Try multithreading command send and state feedback

To download photos/videos goto: http://10.202.0.1/#/

NOTE: the ip address will change depending on whether drone is simulated or real!

"""
import time

from numpy.lib.histograms import histogram
from drone_controller import automission, HiDrone
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State


drone_ip = "10.202.0.1"
drone2_ip = "10.202.1.1"
my_mission = automission("copter")
# Take off command is ignored! wtf
# # Taking a picture
# my_mission.takeoff()
# my_mission.waypoint(48.878922, 2.367782, 1)
# my_mission.image_capture_mode(1,2)
# my_mission.start_take_picture(0, 15, 12)
# my_mission.waypoint(48.878922, 2.367782, 10)
# my_mission.waypoint(48.879000, 2.366549, 20.0)
# my_mission.waypoint(48.879139, 2.367296, 10.0)
# my_mission.stop_take_picture()
# my_mission.land()

# try video recording!
my_mission.takeoff()
my_mission.start_video_capture()
# view modes: 0 fixed, 1 linear, 2 roi

my_mission.set_cam_roi(48.878922, 2.367782, 0)
my_mission.set_view_mode(2,my_mission.get_roi_ind())
my_mission.waypoint(48.878922, 2.367782, 5)

my_mission.set_cam_roi(48.879000, 2.366549, 0)
my_mission.set_view_mode(2,my_mission.get_roi_ind())
my_mission.waypoint(48.879000, 2.366549, 20.0)

my_mission.set_cam_roi(48.879139, 2.367296, 0)
my_mission.set_view_mode(2,my_mission.get_roi_ind())
my_mission.waypoint(48.879139, 2.367296, 10.0)

my_mission.stop_video_capture()
my_mission.land()

# create helper class
hdrone = HiDrone(drone_ip, "Drone_1")
hdrone2 = HiDrone(drone2_ip, "Drone_2")

# setup threads
hdrone.setup()
hdrone2.setup()

# try sending our command
hdrone.send_flightplan(my_mission)
time.sleep(10)
hdrone2.send_flightplan(my_mission)

# main thread loop
start_time = time.time()
i = 0
while time.time() - start_time < 2000:
    print("main thread still alive {}".format(i))
    i += 1
    time.sleep(1)
    if hdrone.drone.get_state(FlyingStateChanged)["state"] is FlyingStateChanged_State.landed:
        # delay sending another flight plan
        print("sending flight plan to first drone")
        hdrone.send_flightplan(my_mission)
    if hdrone2.drone.get_state(FlyingStateChanged)["state"] is FlyingStateChanged_State.landed:
        # delay sending another flight plan
        print("sending flight plan to second drone")
        hdrone2.send_flightplan(my_mission)
