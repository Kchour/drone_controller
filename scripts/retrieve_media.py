"""Retrieve all media from drones

default saved location is at ~/.drone_media_recordings

First connect to each drone via wifi cards (use the setup_drones.sh script)

"""
from drone_controller import HiDrone
import requests
import os
import shutil
from os.path import expanduser
import threading


registered_threads = []

def retrieve():
    """Run all registered threads and wait for them to join main"""
    for t in registered_threads:
        t.start()
    
    for t in registered_threads:
        t.join()

class MediaRetriever(threading.Thread):
    """Retrieve media using multiple threads (1 thread per drone)"""

    def __init__(self, drone_ip, drone_name, download_dir=None):
        self.drone_ip = drone_ip
        self.drone_name = drone_name
        self.download_dir = download_dir
        # register threads
        registered_threads.append(self)
        # to stop a thread
        self.event = threading.Event()

        super().__init__()
        self.daemon = True
    
    def run(self):
        self.retrieve_all_media(self.drone_ip, self.drone_name, self.download_dir) 
        # to stop thread
        print("stopping thread")
        self.event.set()

    def retrieve_all_media(self, drone_ip, drone_name, download_dir=None):
        """Utility function to download all drone media"""
        # create directory if not existing
        if download_dir is None:
            home = expanduser("~")
            download_dir = os.path.join(home, ".drone_media_recordings", drone_name)
            if not os.path.exists(download_dir):
                try:
                    os.makedirs(download_dir)
                except OSError as exc:
                    # TODO guard against raise condition
                    raise
            
        # drone web server URL
        ANAFI_URL = "http://{}/".format(drone_ip)
        ANAFI_MEDIA_API_URL = ANAFI_URL + "api/v1/media/medias"
        # get a list of media
        media_info_response = requests.get(ANAFI_MEDIA_API_URL, stream=True)

        for resource in media_info_response.json():
            # send an HTTTP request to the server
            download_path = os.path.join(download_dir, resource["resources"][0]["resource_id"])
            # skip trying to retrieve existing files
            if os.path.exists(download_path):
                print("Skipping {}".format(download_path))
                continue
            print("requesting: {}".format(ANAFI_URL + resource["resources"][0]["url"]))
            media_object_response = requests.get(ANAFI_URL + resource["resources"][0]["url"])
            media_object_response.raise_for_status()
            # save http response
            download_path = os.path.join(download_dir, resource["resources"][0]["resource_id"])
            with open(download_path, "wb") as media_file:
                print("downloading to: {}".format(download_path))
                # shutil.copyfileobj(media_object_response.raw, media_file)
                media_file.write(media_object_response.content)
        print("FINISHED with {}".format(drone_name))        

# DEFAULT IP ADDRESSES
# DRONE2_IP = "10.202.0.1"  # simulated drone
# DRONE2_IP = "192.168.42.1"  # real drone

# MULTIWIFI
DRONE1_IP = "192.168.44.1" #ANAFI-H066520 (YELLOW) (external wifi) (left)
DRONE2_IP = "192.168.45.1" #ANAFI-H065691 (built in wifi) (right)

# create high level class object
hdrone1 = HiDrone(DRONE1_IP, "Drone_1")
hdrone2 = HiDrone(DRONE2_IP, "Drone_2")

try:
    MediaRetriever(DRONE1_IP, hdrone1.name)
    MediaRetriever(DRONE2_IP, hdrone2.name)
    retrieve()

finally:
    # disconnect from drone when done!
    hdrone1.disconnect()
    hdrone2.disconnect()