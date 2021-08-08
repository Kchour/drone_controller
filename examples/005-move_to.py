"""Move the drone around using the moveTo(lat, long, alt, orient_mode, heading) command,
     which  is based off of absolute positioning
     Params:
        - latitude (double)
        - longitude (double)
        - altitude (double): in meters
        - orientation_mode (str): None, To_TARGET, HEADING_START, HEADING_DURING
        - heading (float): relative to North in degrees (only used if orient_mode is 
            HEADING_XXX)    

    The script will wait for a "sucessful" TakeOff and Hovering
    state change before issuing movement commands

    Finally, the '>>' operator is implemented by overriding
    the __rshift__ dunder method

    NOTE: Make sure to reset the simulation (CTRL+R within Gazebo)
          Make sure to check for gps fix
"""
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, moveTo, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.enums.ardrone3.PilotingState import MoveToChanged_Orientation_mode 
from olympe.messages.ardrone3.PilotingState import PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged

DRONE_IP = "10.202.0.1"

if __name__ == "__main__":
    drone = olympe.Drone(DRONE_IP)
    drone.connect()

    # await for a successful "takeOff" and then "Hovering" state
    assert drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    # get home gps location
    # home_pos = drone.get_state(HomeChanged)
    home_pos = drone.get_state(PositionChanged)

    # await for a successful moveBy to meters along relative x, and then hover
    assert drone(
        moveBy(2.5, 0 ,0 ,0)
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    # get away location
    away_pos = drone.get_state(PositionChanged)
    
    # move back to home position using "moveTo"
    # wait for a successful "moveTo" and then "hovering" state
    # assert drone(
    # moveTo(48.978923, 2.368783, 15, MoveToChanged_Orientation_mode.NONE, 0.0)
    # >> FlyingStateChanged(state="flying", _timeout=5)
    # ).wait().success()

    # move to home
    assert drone(
    moveTo(home_pos["latitude"], home_pos["longitude"], home_pos['altitude'], MoveToChanged_Orientation_mode.NONE, 0.0)
    >> FlyingStateChanged(state="flying", _timeout=5) >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    # move to away
    assert drone(
    moveTo(away_pos["latitude"], away_pos["longitude"], away_pos['altitude'], MoveToChanged_Orientation_mode.NONE, 0.0)
    >> FlyingStateChanged(state="flying", _timeout=5) >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    # move to home
    assert drone(
    moveTo(home_pos["latitude"], home_pos["longitude"], home_pos['altitude'], MoveToChanged_Orientation_mode.NONE, 0.0)
    >> FlyingStateChanged(state="flying", _timeout=5) >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    # wait for a successful "landing"
    assert drone(Landing()).wait().success()

    # disconnect from the drone
    drone.disconnect()