
import time
from drone_controller import HiDrone, automission
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State
from olympe.messages.common.Mavlink import Start, Stop, Pause
from olympe.messages.common.MavlinkState import (
    MavlinkFilePlayingStateChanged,
    MissionItemExecuted,
)
from olympe.enums.common.MavlinkState import MavlinkFilePlayingStateChanged_State

try:
    # DEFAULT IP ADDRESSES
    # DRONE2_IP = "10.202.0.1"  # simulated drone
    # DRONE2_IP = "192.168.42.1"  # real drone

    # MULTIWIFI
    DRONE1_IP = "192.168.44.1" #ANAFI-H066520 (YELLOW) (external wifi)
    DRONE2_IP = "192.168.45.1" #ANAFI-H065691 (built in wifi)

    # create high level class object
    hdrone1 = HiDrone(DRONE1_IP, "Drone_1")
    hdrone2 = HiDrone(DRONE2_IP, "Drone_2")

    # setup threads for mavlink and state data recording (.csv)
    # recordings are saved under ~/.drone_data_recordings
    hdrone1.setup()
    hdrone2.setup()

    # goes to the left
    mission1 = automission("copter")
    mission1.takeoff()
    mission1.start_video_capture()
    mission1.set_cam_roi(30.644883, -96.298987, 1)
    mission1.set_view_mode(2,mission1.get_roi_ind())
    mission1.waypoint(30.644332, -96.299316, 10)
    mission1.waypoint(30.644370, -96.299337, 10)
    mission1.waypoint(30.644569, -96.299495, 10)
    mission1.waypoint(30.644798, -96.299323, 10)
    mission1.waypoint(30.644370, -96.299337, 10)
    mission1.waypoint(30.644332, -96.299316, 10)
    mission1.stop_video_capture()
    mission1.land()

    # goes to the right
    mission2 = automission("copter")
    mission2.takeoff()
    mission2.start_video_capture()
    mission2.set_cam_roi(30.644883, -96.298987, 1)
    mission2.set_view_mode(2,mission2.get_roi_ind())
    mission2.waypoint(30.644313, -96.299256, 10)
    mission2.waypoint(30.644335, -96.299111, 10)
    mission2.waypoint(30.644203, -96.298827, 10)
    mission2.waypoint(30.644499, -96.298715, 10)
    mission2.waypoint(30.644335, -96.299111, 10)
    mission2.waypoint(30.644313, -96.299256, 10)
    mission2.stop_video_capture()
    mission2.land()

    # send flight plan to drone
    # THIS PART NEEDS TO BE MORE SYNCHORNIZED. SOMEHOW START BOTH
    hdrone1.set_flightplan(mission1)
    hdrone2.set_flightplan(mission2)

    # start flight plan
    hdrone1.start_flightplan()
    hdrone2.start_flightplan()

    # keep main thread alive
    start_time = time.time()
    while True:
        print("main thread is still alive: {}".format(time.time()-start_time))
        # exit script if both drones have landed
        # if hdrone1.drone.get_state(FlyingStateChanged)["state"] is FlyingStateChanged_State.landed \
        #     and \
        #     hdrone2.drone.get_state(FlyingStateChanged)["state"] is FlyingStateChanged_State.landed:
        #     break

        # exit script when both drones have finished playing their flight plan
        time.sleep(1)
        if hdrone1.drone.get_state(MavlinkFilePlayingStateChanged)["state"] is \
            MavlinkFilePlayingStateChanged_State.stopped and \
            hdrone2.drone.get_state(MavlinkFilePlayingStateChanged)["state"] is \
            MavlinkFilePlayingStateChanged_State.stopped: 
            break

    # # TESTING PURPOSES: just try taking off COMMENT OUT LATER
    # # both vehicles take-off
    # ##########################################
    # hdrone1.set_func(hdrone1.takeoff)
    # hdrone2.set_func(hdrone2.takeoff)
    # HiDrone.start_and_join()

    # # collect data for 10 seconds
    # start_time = time.time()
    # while time.time() - start_time <= 30:
    #     time.sleep(1)
    #     print(time.time()-start_time)

    # hdrone1.set_func(hdrone1.land)
    # hdrone2.set_func(hdrone2.land)
    # HiDrone.start_and_join()
    # ##########################################

    # # # collect data for 10 seconds
    # # start_time = time.time()
    # # while time.time() - start_time <= 10:
    # #     time.sleep(1)
    # #     print(time.time()-start_time)
finally:
    # hdrone1.disconnect()
    # hdrone2.disconnect()
    hdrone1.set_func(hdrone1.disconnect)
    hdrone2.set_func(hdrone2.disconnect)
    HiDrone.start_and_join()
