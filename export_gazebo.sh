#!/bin/bash
#export GAZEBO_MODEL_PATH=/home/kenny/Development/Projects/OSM2World/target/my_models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=$PWD/models:${GAZEBO_MODEL_PATH}
echo $GAZEBO_MODEL_PATH
#export GAZEBO_MODEL_DATABASE_URI=""
