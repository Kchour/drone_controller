#!/bin/bash

# Please change your device name accordingly! i.e. wlan0, wlan1, ...
# ./config_anafi_ip.sh setup wlan0 192.168.44.1 201
# ./config_anafi_ip.sh setup wlan1 192.168.45.1 202
# ./config_anafi_ip.sh setup wlan2 192.168.46.1 203

# connect this to ANAFI-h066520 (Yellow)
./config_anafi_ip.sh setup wlx0013eff470a8 192.168.44.1 201

# connect this to ANAFI-H065691 (now yellow)
./config_anafi_ip.sh setup wlp60s0 192.168.45.1 202


# try running hello world
sh ./hello_world.sh