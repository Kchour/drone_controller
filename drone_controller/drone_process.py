import multiprocessing as mp
from multiprocessing import Queue
import olympe
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged,\
                                     PositionChanged, SpeedChanged, AttitudeChanged, AirSpeedChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged, HomeChanged
from olympe.enums.ardrone3.PilotingState import MoveToChanged_Orientation_mode, FlyingStateChanged_State

class DroneProcess:
    """Separate drone process to handle user inputs"""
    def 