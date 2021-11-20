"""Send a flight plan (olympe independent)
then execute it (using olympe) 

https://forum.developer.parrot.com/t/olympe-mavlink-working-example/14041/2


# for a more complicated example, see `threading_ops`
"""
import olympe
from olympe.messages.common.Mavlink import Start, Stop, Pause
from olympe.messages.common.MavlinkState import (
    MavlinkFilePlayingStateChanged,
    MissionItemExecuted,
)
olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

import time
import requests
import os

from drone_controller import automission

drone_ip = "10.202.0.1"

headers = {
    "Accept": "application/json, text/javascript, text/plain */*; q=0.01",
    "X-Requested-With": "XMLHttpRequest",
    "Content-type": "application/json; charset=UTF-8; application/gzip",
}

drone = olympe.Drone(drone_ip)
drone.connect()

# # Upload mavlink file
# with open("test.mavlink", "rb") as data:
#     resp = requests.put(
#         url=os.path.join("http://", drone_ip, "api/v1/upload", "flightplan"),
#         headers=headers,
#         data=data,
#     )

# try sending string 
my_mission = automission("copter")
# Take off command is ignored!
# try adding 10 second delay
my_mission.takeoff()
# my_mission.delay(5)                          # this doesn't seem to work 
my_mission.waypoint(48.878922, 2.367782, 5)    # this is the starting gps point
my_mission.waypoint(48.879000, 2.366549, 100.0)
my_mission.waypoint(48.879139, 2.367296, 10.0)
my_mission.land()
# Upload mavlink file
resp = requests.put(
    url=os.path.join("http://", drone_ip, "api/v1/upload", "flightplan"),
    headers=headers,
    data=my_mission.as_text(),
)


# # Start flightplan
# expectation = drone(
#     Start(resp.json(), type="flightPlan")
#     >> MissionItemExecuted(idx=0.0)
#     >> MissionItemExecuted(idx=1.0)
#     >> MissionItemExecuted(idx=2.0)
#     >> MissionItemExecuted(idx=3.0)
#     >> MavlinkFilePlayingStateChanged(state="stopped")
# ).wait(_timeout=200)

t1 = time.time()
print("{}: STARTING FLIGHT PLAN".format(time.time()-t1))
# this is equivalent to above, but allows for better programming
# Add .wait() at the end to block until completion
expectation = Start(resp.json(), type="flightPlan")
# # MissionItemExecuted seems optional
# for i in range(5):
#     expectation = expectation >> MissionItemExecuted(idx=i)
# expectation = expectation >> MavlinkFilePlayingStateChanged(state="stopped")
expectation = expectation >> MavlinkFilePlayingStateChanged(state="playing")
drone(expectation).wait()
print("test")
time.sleep(60)

# Try pausing the flight plan after 60 seconds
print("{}: PAUSING FLIGHT PLAN".format(time.time()-t1))
expectation = Stop()
expectation = expectation >> MavlinkFilePlayingStateChanged(state="paused")
drone(expectation)

time.sleep(5)

# After another 5 seconds, unpause flight plan
print("{}: RESUME FLIGHT PLAN".format(time.time()-t1))
expectation = Start(resp.json(), type="flightPlan")
drone(expectation)

time.sleep(5)

# after 5 seconds, try to cancel the flight plan
print("{}: CANCEL FLIGHT PLAN".format(time.time()-t1))
expectation = Stop()
drone(expectation)

# let's just wait for 2 minutes
time.sleep(120)

assert expectation
