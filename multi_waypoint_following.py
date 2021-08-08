"""
Attempt to do (async) waypoint following assuming a small area
and away from the earth's poles. 

Linear interpolation is used between waypoints
"""

from drone import HiDrone, Util
from video_stream import VideoStreaming


DRONE1_IP = "10.202.0.1"
hidrone1 = HiDrone(DRONE1_IP)
vs1 = VideoStreaming(hidrone1.drone)


DRONE2_IP = "10.202.1.1"
hidrone2 = HiDrone(DRONE2_IP)

wps1 = [(48.878922,2.367782, 1), (48.878932,2.367982, 1), (48.879000,2.367992, 1)]
wps2 = [(48.878992,2.367792, 1), (48.878992,2.367992, 1), (48.879090,2.367992, 1)]

# start video recording on drone 1 
vs1.start()

# both vehicles take-off
hidrone1.set_func(hidrone1.takeoff)
hidrone2.set_func(hidrone2.takeoff)

# Call this to perform prior set funcs. start the thread and join
HiDrone.start_and_join()

# now both vehicles perform waypoint following
hidrone1.set_func(hidrone1.waypoint_following, wps1)
hidrone2.set_func(hidrone2.waypoint_following, wps2)

# start and join threads
HiDrone.start_and_join()

# try to land drones
hidrone1.set_func(hidrone1.land)
hidrone2.set_func(hidrone2.land)

HiDrone.start_and_join()


# stop video recording on drone 1
vs1.stop()

# postprocess
vs1.postprocessing()

print("blah")


