"""Demonstrates the get_state() method, which accepts an ArsdkMessage, and
    returns an ordered dictionary. See the API reference for full
    options

    This script will look at MaxTiltChanged, PositionChanged (for gps), SpeedChanged, and
    AttitudeChanged

"""

import olympe
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.PilotingState import PositionChanged, SpeedChanged, AttitudeChanged, FlyingStateChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged, HomeChanged

from olympe.messages.ardrone3.Piloting import TakeOff

DRONE_IP = "10.202.0.1"

if __name__ == "__main__":
    drone = olympe.Drone(DRONE_IP)
    drone.connect()

    # Let's see MaxTiltChanged's data structure (an ordered dictionary)
    print("")
    print(drone.get_state(MaxTiltChanged))
    print("")
    print("Drone MaxTilt = ", drone.get_state(MaxTiltChanged)["current"])
    
    # wait for gps fix 
    drone(GPSFixStateChanged(_policy = 'wait'))

    # print gps home position
    print("GPS position before take-off :", drone.get_state(HomeChanged))   


    # When the drone actually moves (via a low level PCMD),
    # Event SpeedChanged(), AttitudeChanged() and PositionChanged() 
    # (only if gps of the drone has fixed) are triggered.
    # move the drone initially to trigger these events
    # await for a successful "takeOff" and then "Hovering" state
    assert drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    # Now let's print the position (gps) information    
    # latitude, longitude, attitude, _policy, _float_tol
    if drone.check_state(PositionChanged):
        lat = drone.get_state(PositionChanged)["latitude"] 
        lon = drone.get_state(PositionChanged)["longitude"] 
        alt = drone.get_state(PositionChanged)["altitude"] 
    
        print("Latitude:", lon) 
        print("Longitude:", lat) 
        print("Altitude:", alt) 
    
    # now for speed
    # speedX, speedY, speedZ, _policy, _float_tol
    if drone.check_state(SpeedChanged):
        speedx = drone.get_state(SpeedChanged)['speedX']
        speedy = drone.get_state(SpeedChanged)['speedY']
        speedz = drone.get_state(SpeedChanged)['speedZ']

        print("speedx:", speedx)
        print("speedy:", speedy)
        print("speedz:", speedz)

    # now for attitude (radians)
    if drone.check_state(AttitudeChanged):
        roll = drone.get_state(AttitudeChanged)['roll']
        pitch = drone.get_state(AttitudeChanged)['pitch']
        yaw = drone.get_state(AttitudeChanged)['yaw']
        
        print("roll:", roll)
        print("pitch:", pitch)
        print("yaw:", yaw)
    
    drone.disconnect()