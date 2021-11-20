"""This script will connect to the simulated drone and
    send a `TakeOff()` command. Then .wait() ensures the script 
    will block until the drone acknowledges with an event message
    (an internal aspect).


"""
import olympe
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing

# This is the default drone IP address over the virtual Ethernet interface
# DRONE_IP = "10.202.0.1"       # simulated drone
DRONE_IP = "192.168.42.1"       # real drone


def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    assert drone(TakeOff()).wait().success()
    time.sleep(10)
    assert drone(Landing()).wait().success()
    drone.disconnect()


if __name__ == "__main__":
    main()