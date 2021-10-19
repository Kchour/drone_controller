"""
This demo script consists of a single drone. It will take-off, follow some waypoints, and land. 
Additionally, drone1 will use its camera to record.

run this first: `$ sphinx drone_control/drones/xxx.drone::stolen_interface=::simple_front_cam=true drone_control/worlds/rellis_campus.world`


"""
from drone_controller import HiDrone, Util, VideoStreamShow

# create cv gui
vss = VideoStreamShow()

# connect to drone1 and add its stream to the gui
print("Connecting to drone 1...")
DRONE1_IP = "10.202.0.1"
hidrone1 = HiDrone(DRONE1_IP)
vss.setup_drone(hidrone1)
print("Conneced!")

# [lat, lon, altitude[m] ]
wps1 = [(48.878922,2.367782, 1), (48.878932,2.367982, 1), (48.879000,2.367992, 1)] 

# start video recording on drone 1 
vss.start_stream(hidrone1)

# vehicle 1 take-off
hidrone1.takeoff()

# follow waypoints (this is blocking)
hidrone1.waypoint_following(wps1)

# now try landing
hidrone1.land()

# stop video recording on drone 1
vss.stop_stream(hidrone1)

# postprocess
vss.postprocessing(hidrone1)

print("blah")

hidrone1.disconnect()


