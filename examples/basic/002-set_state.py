"""Demonstrates setting a state (MaxTilt) and some feedback
    - .success() (bool)
    - .timedout() (bool)
    i.e. every command message results in either of the two being True!
    
    Two types of olympe:
    - command_message expects 
    - an event_message
   
   Also demonstrates behavior when erraneous commands are sent
    
"""
import olympe
from olympe.messages.ardrone3.PilotingSettings import MaxTilt

DRONE_IP = "10.202.0.1"

if __name__ == "__main__":
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    # send a MaxTilt(10) command message, which expects
    # a MaxTiltChanged(10) event message in return
    maxTiltAction = drone(MaxTilt(10)).wait()
    # if the event message is returned in time, then .success() is True
    # and .timedout() is false
    if maxTiltAction.success():
        print("MaxTilt(10) success")
    elif maxTiltAction.timedout():
        print("MaxTilt(10) timedout")
    else:
        # If ".wait()" is called on the ``maxTiltAction`` this shouldn't happen
        print("MaxTilt(10) is still in progress")
    
    # Now send an erraneous command (MaxTilt needs to be >0)
    maxTiltAction = drone(MaxTilt(0)).wait()
    # The event message will never come because it's impossible to do
    # so .success() is false, while .timedout() will be True
    if maxTiltAction.success():
        print("MaxTilt(0) success")
    elif maxTiltAction.timedout():
        print("MaxTilt(0) timedout")
    else:
        # If ".wait()" is called on the ``maxTiltAction`` this shouldn't happen
        print("MaxTilt(0) is still in progress")
    drone.disconnect()