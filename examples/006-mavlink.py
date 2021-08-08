"""
This script will attempt to send a mavlink file to the drone

example mavlink file from https://developer.parrot.com/docs/mavlink-flightplan/overview.html

"""
import olympe
from olympe.messages.common.Mavlink import Start

# For the mavlink protocol to be active, 
# a wifi interface is needed on the pc running the simulator
# and this wifi interface needs to be active for the simulated 
# drone firmware.

DRONE_IP = "192.168.42.1"

if __name__ == "__main__":
    drone = olympe.Drone(DRONE_IP)
    drone.connect()

    # transfer mavlink file using this:
    # curl -T test.mavlink ftp://10.202.0.1:61

    Start("qgc_mavlink_test.mavlink")

    drone.disconnect