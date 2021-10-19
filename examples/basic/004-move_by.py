"""Move the drone around using the moveBy(dx, dy, dz, dPsi or heading) command, which
    is based off of relative positioning.

    The script will wait for a "successful" TakeOff and Hovering
    state change before issuing movement commands

    Finally, the '>>' operator is implemented by overriding
    the __rshift__ dunder method

    NOTE: Make sure to reset the simulation (CTRL+R within Gazebo)

"""
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

DRONE_IP = "10.202.0.1"

if __name__ == "__main__":
    drone = olympe.Drone(DRONE_IP)
    drone.connect()

    # Wait for a successful "takeOff" and then "Hovering" state
    assert drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    # wait for a successful "moveBy" and then "Hovering" state
    assert drone(
        moveBy(10, 0, 0, 0)
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    # wait for a successful "landing"
    assert drone(Landing()).wait().success()

    # disconnect from the drone
    drone.disconnect()