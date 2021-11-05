"""Work in progress"""

# import olympe
import requests


drone_ip = "10.202.0.1"
drone_url = "http://{}/".format(drone_ip)
drone_media_api_url = drone_url + 'api/v1/media/medias'

# try to download photos associated with this media id
media_info_response = requests.get(drone_media_api_url)

pass