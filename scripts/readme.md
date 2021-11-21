# Info
Make sure all script here are executable via `chmod +x *.sh`

All scripts must be run from this directory

First replace the device name in both the setup and clean up script with your device names

Run the setup scripts to route traffic data to ip aliases: `$ ./setup_drones.sh` 

Note that if you ever disconnect from any of the drones due to connection failure (turning off wifi card, drone turned off, etc..), 
you must rerun the setup script

Run your flight controllers, tests, etc...

When you're completely finished, run clean up scripts: `$ ./cleanup_drones.sh`