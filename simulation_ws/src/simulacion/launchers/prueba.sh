#!/bin/bash

export GAZEBO_RESOURCE_PATH=$(rospack find alpha_sim)/../Worlds:$GAZEBO_RESOURCE_PATH

DRONE_SWARM_MEMBERS=$1

if [ -z $DRONE_SWARM_MEMBERS ] # Check if NUMID_DRONE is NULL
  then
  	#Argument 1 empty
    	echo "-Setting Swarm Members = 1"
    	DRONE_SWARM_MEMBERS=1
  else
    	echo "-Setting DroneSwarm Members = $1"
fi

