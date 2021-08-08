# Overview 
Some higher level API implementation wrapper for Olympe

## Getting Started

First install `parrot-sphinx` and `parrot-olympe`. The latest version of olympe 1.71 (at the time of writing) has some issues 
with video streaming, so consider using 1.70

# Terminal 1

Start firmware daemon.

`$ sudo systemctl start firmwared"`

start olympe virtual environment.
`$ source /path_to/parrot-groundsdk/products/olympe/linux/env/shell`

export models path to GAZEBO_MODEL_PATH
`$ source export_gazebo.sh`

Start Sphinx simulator with a specific drone (replace xxx).

`$ sphinx drone_control/drones/xxx.drone`

Also, you can also set simulation parameters at run-time

`$ sphinx drone_control/drones/xxx.drone::stolen_interface=::simple_front_cam=true drone_control/worlds/rellis_campus.world`

# Terminal 2

source environmental variables before running any of the scripts
`$ source /path_to/parrot-groundsdk/products/olympe/linux/env/setenv` 

## Multi-drone simulation

Simply pass two `.drone` files to sphinx, but each drone must have a unique name and different starting pose!
e.g.
`sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone::name=other::stolen_interface=`

Follow all the steps in the previous section except for what you will pass to sphinx:
`sphinx drones/anafi4k.drone drones/anafi4k.drone::name="blah"::pose="5 0 0.2 0 0 0" worlds/rellis_campus.world`

If using ethernet interface, the default address for the first drone is 10.202.0.1 (then 10.202.1.1 for the second drone, …).

Verify using `fdc list instances` to see simulated instances. More info at [https://developer.parrot.com/docs/sphinx/firmwared.html](https://developer.parrot.com/docs/sphinx/firmwared.html)

Now trying running `multi_waypoint_following.py` in terminal 2

## Misc

- Reset the gazebo simulator often when testing  via CTRL+R

## Issues

- VideoStream will have `olympe.pdraw.ANAFI-0000000 - _unregister_future - Failed to unregister future`. Seems like an issue with the latest version of olympe?
- How to send mavlink files to the drone?