# Overview 
This repo contains a simple, high level wrapper for Olympe (only a few things implemented), that utlizes threading to do multi-drone control. An example world and a multi-vehicle simulation demo is included.
![Screenshot from 2021-08-08 02-14-10](https://user-images.githubusercontent.com/21199708/128624401-812ba564-a14a-47d8-8a1f-224a3def077c.png)

![Screenshot from 2021-08-08 02-15-52](https://user-images.githubusercontent.com/21199708/128624406-8e4f36a2-fbd3-405b-8dcf-8d5aca879603.png)


## Getting Started

First install [parrot-sphinx](https://developer.parrot.com/docs/sphinx/) and [parrot-olympe](https://developer.parrot.com/docs/olympe/).

Then clone this repo somewhere say 

```
$ cd ~/tests
$ git clone https://github.com/Kchour/drone_controller
```

Install this package in editable mode:

```
$ cd ~/tests/drone_controller
$ python3 -m pip install -e .
```

# Terminal 1

Start firmware daemon.

`$ sudo systemctl start firmwared.service`

Start olympe virtual environment.

`$ source /path_to/parrot-groundsdk/products/olympe/linux/env/shell`

Export models path to GAZEBO_MODEL_PATH.

`$ source export_gazebo.sh`

Start Sphinx simulator with a specific drone (replace xxx).

`$ sphinx drones/xxx.drone`

Also, you can also set simulation parameters at run-time

`$ sphinx drones/anafi4k.drone::stolen_interface=::simple_front_cam=true worlds/rellis_campus.world`

# Terminal 2

Source environmental variables before running any of the scripts

`$ source /path_to/parrot-groundsdk/products/olympe/linux/env/setenv` 

## Multi-drone simulation

Simply pass two `.drone` files to sphinx, but each drone must have a unique name and different starting pose!
e.g.

`sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone::name=other::stolen_interface=`

Follow all the steps in the previous section except for what you will pass to sphinx:

`sphinx drones/anafi4k.drone drones/anafi4k.drone::name="blah"::pose="5 0 0.2 0 0 0" worlds/rellis_campus.world`

If using ethernet interface, the default address for the first drone is 10.202.0.1 (then 10.202.1.1 for the second drone, …).

Verify using `fdc list instances` to see simulated instances. More info at [https://developer.parrot.com/docs/sphinx/firmwared.html](https://developer.parrot.com/docs/sphinx/firmwared.html)

Now trying running `multi_waypoint_following.py` in terminal 2. You should see the following:

https://user-images.githubusercontent.com/21199708/128624793-622278a7-9e43-4e24-b079-61cd6155c03b.mp4

Here's a view from the drone1 camera:


https://user-images.githubusercontent.com/21199708/128624792-10751b8b-e797-487a-b063-8ae7e5c403a9.mp4


## Misc

- Reset the gazebo simulator often when testing  via CTRL+R

## Issues

- VideoStream will have `olympe.pdraw.ANAFI-0000000 - _unregister_future - Failed to unregister future`. Seems like an issue with the latest version of olympe?
- How to send mavlink files to the drone?
- Unable to control drone horizontal speed via `moveTo` command...perhaps consider moving via [pcmd](https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.PCMD)?
