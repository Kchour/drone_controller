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

# # # turn on video streaming
# # vs = VideoStreaming(hidrone.drone)

# # # start recording
# # hidrone.video.start()

# hidrone1.takeoff()
# hidrone2.takeoff()

# # create waypoint following threads
# wpf1 = WaypointFollower(hidrone1, wps1)
# wpf2 = WaypointFollower(hidrone2, wps2)

# # start wp following threads
# wpf1.start()
# wpf2.start()

# # join to terminate threads the end
# wpf1.join()
# wpf2.join()


# # stop recording
# hidrone.video.stop()

# # postprocess recorded video
# hidrone.video.postprocessing()

print("blah")

# print(hidrone.get_pos())    # doesn't seem to work when it's hovering?


# desired_speed = 5 #[m/s]

# start_pos = 
# end_pos = (48.878932,2.367982)

# lat1, lon1 = start_pos
# lat2, lon2 = end_pos

# dlat = lat2-lat1
# dlon = lon2-lon1

# overall_dist = Util.haversine(start_pos, end_pos)
# bearing = Util.bearing(start_pos, end_pos)
# print(overall_dist, bearing)

# # take and get pos
# drone.takeoff()
# pos = drone.get_pos()


# total_dist = 0
# dt = 0.25
# time_prev =  0
# while total_dist < overall_dist:
#     # give command every dt seconds
#     time_now = time.time()
#     if time_now - time_prev > dt:
#         dist_ahead = desired_speed*dt
#         total_dist += dist_ahead
#         frac = total_dist/overall_dist
#         # linear interpolation
#         new_lat = lat1 + frac*(lat2 - lat1)
#         new_lon = lon1 + frac*(lon2 - lon1)

#         drone.moveto(new_lat, new_lon, pos["altitude"], _async=True)
#         print(new_lat, new_lon, drone.get_air_speed())

#         # set prev time
#         time_prev = time_now

# # make sure drone ends up at the final position
# drone.moveto(lat2, lon2, pos["altitude"])

# print(drone.get_pos())
# # drone.disconnect()

# # drone.land()


