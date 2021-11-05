"""Send a flight plan (olympe independent)
then execute it (using olympe) 

https://forum.developer.parrot.com/t/olympe-mavlink-working-example/14041/2


# for a more complicated example, see `threading_ops`
"""

import olympe
from olympe.messages.common.Mavlink import Start
from olympe.messages.common.MavlinkState import (
    MavlinkFilePlayingStateChanged,
    MissionItemExecuted,
)

from drone_controller import automission
import requests
import os

olympe.log.update_config({"loggers": {"olympe": {"level": "INFO"}}})


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
my_mission.takeoff(0)
my_mission.waypoint(48.879000, 2.366549, 100.0)
my_mission.waypoint(48.879139, 2.367296, 10.0)
my_mission.land(48.879139, 2.367296, 0)
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

# this is equivalent to above, but allows for better programming
expectation = Start(resp.json(), type="flightPlan")
for i in range(4):
    expectation = expectation >> MissionItemExecuted(idx=i)
expectation = expectation >> MavlinkFilePlayingStateChanged(state="stopped")
drone(expectation).wait()

assert expectation
