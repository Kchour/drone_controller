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
olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

import threading
import random
import numpy as np
# import utm
from typing import List


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

    @staticmethod
    def start_and_join():
        ThreadHandler.start_and_join()

    def __init__(self, DRONE_IP, name=None):
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
        posDict = self.drone.get_state(PositionChanged)
        lat, lon, alt = posDict["latitude"], posDict["longitude"], posDict["altitude"]

        print("Latitude:", lat) 
        print("Longitude:", lon) 
        print("Altitude:", alt) 

        return posDict

    def get_air_speed(self):
        """
        Get drone speed in drone's referential. AirSpeedChanged isn't supported :[

        """
        speedDict = self.drone.get_state(SpeedChanged)

        airspeed = np.linalg.norm([speedDict["speedX"], speedDict["speedY"], speedDict["speedZ"] ])
        print(airspeed)

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


class WaypointFollower(threading.Thread):
    """ DEPRECATED
    
    To use, simply instantiate and call start() and join()

    """
    def __init__(self, hidrone:HiDrone, waypoints:List):
        super().__init__()
        self.waypoints = waypoints
        self.hidrone = hidrone

    def run(self):
        """
        Do not invoke this function directly

        """
        print("performing waypoint following")
        for wp in self.waypoints:
            # self.hidrone.drone(moveTo(wp[0], wp[1], wp[2], MoveToChanged_Orientation_mode.NONE, 0.0))
            # hacky way to do this
            # blah += "moveTo({}, {}, {}, MoveToChanged_Orientation_mode.NONE, 0.0, _no_expect=True) >> FlyingStateChanged(state='flying', _timeout=5,  _no_expect=True) >> FlyingStateChanged(state='hovering', _timeout=5,  _no_expect=True) >> ".format(wp[0], wp[1], wp[2])
            self.hidrone.moveto(wp[0], wp[1], wp[2])   

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
