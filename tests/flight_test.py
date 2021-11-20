
import time
from drone_controller import HiDrone

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

    # TESTING PURPOSES: just try taking off COMMENT OUT LATER
    # both vehicles take-off
    ##########################################
    hdrone1.set_func(hdrone1.takeoff)
    hdrone2.set_func(hdrone2.takeoff)
    HiDrone.start_and_join()

    # collect data for 10 seconds
    start_time = time.time()
    while time.time() - start_time <= 30:
        time.sleep(1)
        print(time.time()-start_time)

    hdrone1.set_func(hdrone1.land)
    hdrone2.set_func(hdrone2.land)
    HiDrone.start_and_join()
    ##########################################

    # # collect data for 10 seconds
    # start_time = time.time()
    # while time.time() - start_time <= 10:
    #     time.sleep(1)
    #     print(time.time()-start_time)
finally:
    # hdrone1.disconnect()
    # hdrone2.disconnect()
    hdrone1.set_func(hdrone1.disconnect)
    hdrone2.set_func(hdrone2.disconnect)
    HiDrone.start_and_join()
