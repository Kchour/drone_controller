"""
Simpler API for interacting with our drones

"""
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy, moveTo
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged,\
                                     PositionChanged, SpeedChanged, AttitudeChanged, AirSpeedChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged, HomeChanged
from olympe.enums.ardrone3.PilotingState import MoveToChanged_Orientation_mode, FlyingStateChanged_State

# Limit the verbosity of log
olympe.log.update_config({"loggers": {"olympe": {"level": "INFO"}}})

from io import BytesIO
import threading
from threading import Lock
import random
import numpy as np
# import utm
from typing import List
import time
from collections import deque
import requests
import os
from os.path import expanduser

from .flight_plan import automission

lock = Lock()



class ThreadHandler(threading.Thread):

    # Keep track of all threads
    instances = {}

    #keep track of hidrone instances
    @classmethod
    def add_instance(cls, name, obj):
        if name not in cls.instances:
            cls.instances[name] = obj

    @classmethod
    def start_and_join(cls):
        """
        Start all threads
        """
        for k,v in cls.instances.items():
            print("starting:", k)
            if not v.is_alive():
                v.start()

        for k,v in cls.instances.items():
            print("joining", k)
            v.join()
        
        # clear all threads or risk raising error, i.e.
        # All threads can only be started once!
        cls.instances = {}

    def __init__(self, func, *args, **kwargs):
        self.func = func
        self.args = args
        self.kwargs = kwargs

        ThreadHandler.add_instance(random.randint(1,900), self)

        # This must be called for threading
        super().__init__()

    def run(self):
        self.func(*self.args, **self.kwargs)

class HiDrone:
    """Allow user to start the above two threads each time this object is created"""
    @staticmethod
    def start_and_join():
        """
        Execute functions
        """
        ThreadHandler.start_and_join()

    def __init__(self, DRONE_IP, name=None):
        self.drone_ip = DRONE_IP
        self.drone = olympe.Drone(DRONE_IP)
        self.drone.connect()

        # wait for gps fix 
        self.drone(GPSFixStateChanged(_policy = 'wait'))

        # # turn on video streaming
        # if video_on:
        #     self.video = VideoStreaming(self.drone)
        #     super().__init__()
        #     super().start()

        if name is None:
            self.name = DRONE_IP
        else:
            self.name = name

        # init thread objects
        home = expanduser("~")
        path = os.path.join(home, ".drone_data_recordings", self.name+".csv")

        self.state_deque = deque()
        self.state_thread = StateThread(self, state_deque=self.state_deque, path=path)

        self.cmd_deque = deque()
        self.cmd_thread = MavlinkThread(self, self.cmd_deque)

    def set_func(self, func, *args, **kwargs):
        """
        Store user defined function to run when invoked with threading.Thread.start()

        """
        # create a new thread
        th = ThreadHandler(func, *args, **kwargs)

    def takeoff(self):
        """
        Await for a successful 'takeoff' and then 'hover'

        Drone will take-off from its grounded position
        """

        print("Takeoff if necessary...")
        if (self.drone.get_state(FlyingStateChanged)["state"] is not
                FlyingStateChanged_State.hovering):
            self.drone(GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")).wait()
            self.drone(
                TakeOff(_no_expect=True)
                & FlyingStateChanged(state="hovering", _policy="wait", _timeout=5)
            ).wait().success()

    def land(self, _async=False):
        """
        Land the drone
        """
        if not _async:
            assert self.drone(Landing()).wait().success()
        else:
            self.drone(Landing())

    def moveby(self, dx:float, dy:float, dz:float, dpsi:float, _async=False):
        """Await for a successful moveby operation if _async is false
        
        Drone needs to have taken-off first!

        params:
            x: wanted displacement in along current (relative) x-axis [meters]
            y: ... y-axis
            z: ... z-axis
            psi: Wanted rotation of heading [rad]

        """
        if not _async:
            self.drone(
                moveBy(dx, dy , dz, dpsi) >> FlyingStateChanged(state="hovering", _timeout=5)
            )
        else:
            assert self.drone(
                moveBy(dx, dy, dz, dpsi)
                >> FlyingStateChanged(state="hovering", _timeout=5)
            ).wait().success()

    def moveto(self, lat:float, lon:float, alt:float, ori: str=None, heading:float=0.0):
        """
        Await for a sucessful moveTo, Flying, and Hover if _async is false
        
        Params:
            lat: Latitude of the location (in degrees) to reach
            lon: Longitude of the location (in degrees) to reach
            alt: Altitude above take off point (in m) to reach
            ori: Orientation mode of the move to 
            heading: Heading (relative to the North in degrees). This value is only used if the orientation mode is HEADING_START or HEADING_DURING

        """     
        if ori is None:
            _ori = MoveToChanged_Orientation_mode.NONE
        elif ori == "to_target":
            _ori = MoveToChanged_Orientation_mode.TO_TARGET
        elif ori == "heading_start":
            _ori = MoveToChanged_Orientation_mode.HEADING_START
        elif ori == "heading_during":
            _ori = MoveToChanged_Orientation_mode.HEADING_DURING

        assert self.drone(
            moveTo(lat, lon, alt, _ori, heading)
            >> FlyingStateChanged(state="flying", _timeout=np.inf) >> FlyingStateChanged(state="hovering", _timeout=np.inf)
            ).wait().success()

    def get_pos(self):
        """
        Get gps information. Will only be available when
        the drone has actually moved once!

        Returns:
            an orderedDict (see below)
        """
        try:
            posDict = self.drone.get_state(PositionChanged)
            lat, lon, alt = posDict["latitude"], posDict["longitude"], posDict["altitude"]

            print("Latitude:", lat) 
            print("Longitude:", lon) 
            print("Altitude:", alt) 
        except:
            posDict = {"latitude": -1, "longitude":-1, "altitude": -1}

        return posDict

    def get_air_speed(self):
        """
        Get drone speed in drone's referential. AirSpeedChanged isn't supported :[

        """
        speedDict = self.drone.get_state(SpeedChanged)

        airspeed = np.linalg.norm([speedDict["speedX"], speedDict["speedY"], speedDict["speedZ"] ])
        print("airspeed: {}".format(airspeed))

        return airspeed

    def get_global_speed(self):
        """
        Get drone speed in NED referential (North-East-Down)
        
        """
        speedDict = self.drone.get_state(SpeedChanged)

        print("speedx:", speedDict["speedX"])
        print("speedy:", speedDict["speedY"])
        print("speedz:", speedDict["speedZ"])

        return speedDict

    def get_attitude(self):

        attDict = self.drone.get_state(AttitudeChanged)

        print("roll:", attDict["roll"])
        print("pitch:", attDict["pitch"])
        print("yaw:", attDict["yaw"])   

        return attDict

    def disconnect(self):
        """
        Disconnect from the drone. Always a good idea when done
        """
        self.drone.disconnect()

    def waypoint_following(self, waypoints):
        """
        This function is blocking!

        """
        print("performing waypoint following")
        for wp in waypoints:
            # self.hidrone.drone(moveTo(wp[0], wp[1], wp[2], MoveToChanged_Orientation_mode.NONE, 0.0))
            # hacky way to do this
            # blah += "moveTo({}, {}, {}, MoveToChanged_Orientation_mode.NONE, 0.0, _no_expect=True) >> FlyingStateChanged(state='flying', _timeout=5,  _no_expect=True) >> FlyingStateChanged(state='hovering', _timeout=5,  _no_expect=True) >> ".format(wp[0], wp[1], wp[2])
            self.moveto(wp[0], wp[1], wp[2])    

    def setup(self):
        """set up threads"""
        # first wait for GPS STATE CHANGED
        # while not self.drone.get_state(GPSFixStateChanged)[]
        while self.drone.get_state(FlyingStateChanged)['state'] is not FlyingStateChanged_State.landed:
            pass
        print("Drone {} @ {} STARTING COMMAND THREAD".format(self.name, self.drone_ip))
        self.cmd_thread.start()
        print("Drone {} @ {} STARTING STATE-RECORDING THREAD".format(self.name, self.drone_ip))
        self.state_thread.start()

    def send_flightplan(self, cmd: automission):
        # only allow one flight plan in queue
        if len(self.cmd_deque) == 0:
            with lock:
                self.cmd_deque.append(cmd)
                print("main thread deque {}".format(len(self.cmd_deque)))

class StateThread(threading.Thread):
    """Thread just for reporting vehicle state"""
    def __init__(self, drone: HiDrone, state_deque, path: str):
        self.drone = drone
        # store returned values into dequeue
        self.state_queue = state_deque
        self.rate = 1

        self.path = self.unique_name(path)
        super().__init__()

    def unique_name(self, path):
        filename, extension = os.path.splitext(path)
        counter = 1

        while os.path.exists(path):
            path = filename + " (" + str(counter) + ")" + extension
            counter += 1

        # create file if it does not exist
        if not os.path.exists(os.path.dirname(path)):
            try:
                os.makedirs(os.path.dirname(path)) 
            except OSError as exc:
                # guard against raise condition
                raise
            
            # write a blank file there
            with open(path, 'w') as f:
                f.write("")

        return path

    def run(self):
        self.prev_time = time.time()
        start = time.time()
     
        with open(self.path, 'a') as file:
            # Add titles
            file.write("time, vgx, vgy, vgz, vb, lat, lon, alt, roll, pitch, yaw\n")
            while True:
                if self.drone.drone.get_state(FlyingStateChanged)["state"] is not FlyingStateChanged_State.landing or \
                    self.drone.drone.get_state(FlyingStateChanged)["state"] is not FlyingStateChanged_State.landed:

                    # make time series

                    # get time
                    # get global frame speed
                    # get body frame speed
                    # get pose (lat/lon/alt)
                    # get attitude
                    vgx, vgy, vgz = self.drone.get_global_speed().values()
                    vb = self.drone.get_air_speed()
                    lat, lon, alt = self.drone.get_pos().values()
                    r, p, y = self.drone.get_attitude().values()

                    # time, vgx, vgy, vgz, vb, lat, lon, alt, roll, pitch, yaw
                    data = "{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(time.time()-start, vgx, vgy, vgz, vb, lat, lon, alt, r, p, y)

                    # write to file
                    file.write(data)

                    # control loop rate
                    time_now = time.time()
                    dt = time_now - self.prev_time
                    if 1/self.rate - dt >0:
                        time.sleep(1/self.rate - dt)

                    self.prev_time = time_now

    
from olympe.messages.common.Mavlink import Start
from olympe.messages.common.MavlinkState import  (
    MavlinkFilePlayingStateChanged, MissionItemExecuted,)
from olympe.enums.common.MavlinkState import MavlinkFilePlayingStateChanged_State

class MavlinkThread(threading.Thread):
    """Thread (queue) for transfering mavlink flightplan to drone"""

    def __init__(self, drone: HiDrone, cmd_deque):
        self.drone = drone
        self.rate = 10
        self.cmd_deque = cmd_deque

        # restAPI related
        self.headers = {
            "Accept": "application/json, text/javascript, text/plain */*; q=0.01",
            "X-Requested-With": "XMLHttpRequest",
            "Content-type": "application/json; charset=UTF-8; application/gzip",
        }
        # must be called for thread to work
        super().__init__()

    def run(self):
        self.prev_time = time.time()
        while True:
            # get cmd
            if len(self.cmd_deque)>0:
                with lock: 
                    cmd_data = self.cmd_deque.popleft()
                    num_of_cmds = len(cmd_data)
                # keep trying to upload mavlink file/string until success
                while self.drone.drone.get_state(FlyingStateChanged)["state"] is not FlyingStateChanged_State.takingoff:
                    print("Sending")
                    resp = requests.put(
                        url=os.path.join("http://", self.drone.drone_ip, "api/v1/upload", "flightplan"),
                        headers=self.headers,
                        data=BytesIO(cmd_data.as_text().encode("utf-8")),
                    )

                    # now execute the flightplan
                    expectation = Start(resp.json(), type="flightPlan")
                    for i in range(num_of_cmds):
                        expectation = expectation >> MissionItemExecuted(idx=i)

                    expectation = expectation >> MavlinkFilePlayingStateChanged(state="stopped")

                    # wait doesn't unblock for some reason
                    # self.drone.drone(expectation).wait()
                    self.drone.drone(expectation)
                    time.sleep(0.25)
                
                # block until mavlinkfileplyaingstatechanged
                while self.drone.drone.get_state(MavlinkFilePlayingStateChanged)['state'] is not MavlinkFilePlayingStateChanged_State.stopped:
                    time.sleep(1)
                print("Done")

            # control loop rate
            time_now = time.time()
            dt = time_now - self.prev_time
            if 1/self.rate - dt >0:
                time.sleep(1/self.rate - dt)

            self.prev_time = time_now

    # def pcmd_moveto(self, lat:float, lon:float, alt):

    #     if not hasattr(self, "pcmd_pid"):
    #         # pitch controller
    #         self.theta_pid = PID(5,1,1)

    #         # roll controller
    #         self.phi_pid(5,1,1)

    #     Xd, Yd, _,_ = utm.from_latlon(lat, lon)

    #     pos = self.get_pos
    #     X, Y, _,_ = utm.from_latlon(pos["latitude"], pos['longitude'])
    #     Z = pos['altitude']

    #     # desired pitch angle [rad]
    #     theta_d = self.theta_pid(Xd - X)

    #     # desired roll angle [rad]
    #     phi_d = self.phi_pid(Yd - Y)


# class PID:

#     def __init__(self, kp, ki, kd):
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd

#         self.error_total = 0
#         self.error_prev = 0

#     def compute(self, error):
#         # accumulate error
#         self.error_total += error

#         # compute control input
#         val = self.kp*error + self.ki * self.error_total + self.kd * (error - self.error_prev)
        
#         # store for previous error
#         self.error_prev = error

class Util:

    @classmethod
    def haversine(cls, coord1, coord2):
        """
        Returns the distance between two coords (lat,lon)
        """
        lat1, lon1 = coord1
        lat2, lon2 = coord2

        # convert from decimal degrees to radians
        phi_1 = np.radians(lat1)
        phi_2 = np.radians(lat2)
        del_phi = np.radians(lat2-lat1)
        del_lambda = np.radians(lon2-lon1)

        a = np.sin(del_phi/2.0)**2 + np.cos(phi_1)*np.cos(phi_2)*np.sin(del_lambda/2.0)**2
        c = 2*np.arctan2(np.sqrt(a), np.sqrt(1-a))
        R = 6371000 # Earths mean radius [m]
        d = R*c  # distance between coord1, coord2 [m]

        return d
    
    @classmethod
    def bearing(cls, coord1, coord2):
        """
        Return the (initial) bearing (or heading from coord1 to coord2) in [rads]

        """
        lat1, lon1 = coord1
        lat2, lon2 = coord2

        # convert from decimal degrees to radians
        phi_1 = np.radians(lat1)
        phi_2 = np.radians(lat2)
        del_phi = np.radians(lat2-lat1)
        del_lambda = np.radians(lon2-lon1)

        a = np.sin(del_lambda)*np.cos(phi_2)
        b = np.cos(phi_1)*np.sin(phi_2)-np.sin(phi_1)*\
            np.cos(phi_2)*np.cos(del_lambda)

        theta = np.arctan2(a, b)

        return theta
