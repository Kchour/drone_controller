"""
This demo script consists of 2 drones. They will take-off, follow some waypoints, and land. 
Additionally, drone1 will use its camera to record.

run this first: `sphinx drones/anafi4k.drone drones/anafi4k.drone::name="blah"::pose="5 0 0.2 0 0 0" worlds/rellis_campus.world`

This script will also make use of the ThreadHandler in the drone module

"""
from drone_controller import HiDrone, Util, VideoStreamShow


# create cv gui instance
vss = VideoStreamShow()

# connect to drone1, add its stream to the gui and record
print("Connecting to drone 1...")
DRONE1_IP = "10.202.0.1"
hidrone1 = HiDrone(DRONE1_IP)
vss.setup_drone(hidrone1)
print("Connected!")

print("Connecting to drone 2...")
DRONE2_IP = "10.202.1.1"
hidrone2 = HiDrone(DRONE2_IP)
vss.setup_drone(hidrone2)
print("Connected!")

# create some sample waypoints [lat, lon, alt[m] ]
wps1 = [(48.878922,2.367782, 1), (48.878932,2.367982, 1), (48.879000,2.367992, 1)]
wps2 = [(48.878992,2.367792, 1), (48.878992,2.367992, 1), (48.879090,2.367992, 1)]

# start video recording streams on both drones
vss.start_stream(hidrone1)
vss.start_stream(hidrone2)

# both vehicles take-off
hidrone1.set_func(hidrone1.takeoff)
hidrone2.set_func(hidrone2.takeoff)

# Call this to perform prior set_funcs at the same time. start the thread and join
HiDrone.start_and_join()

# now perform waypoint following with both vehicles
print("Performing waypoint following")
hidrone1.set_func(hidrone1.waypoint_following, wps1)
hidrone2.set_func(hidrone2.waypoint_following, wps2)

# start and join threads
HiDrone.start_and_join()

# try to land drones
hidrone1.set_func(hidrone1.land)
hidrone2.set_func(hidrone2.land)

HiDrone.start_and_join()

# stop video recording on drone 1&2
vss.stop_stream(hidrone1)
vss.stop_stream(hidrone2)

# postprocess
vss.postprocessing(hidrone1)
vss.postprocessing(hidrone2)

print("blah")

# disconnect drones at the same time
hidrone1.set_func(hidrone1.disconnect)
hidrone2.set_func(hidrone2.disconnect)
HiDrone.start_and_join()


