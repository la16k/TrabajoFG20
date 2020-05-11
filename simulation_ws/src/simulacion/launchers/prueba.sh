#!/bin/bash

export GAZEBO_RESOURCE_PATH=$(rospack find )/../Worlds:$GAZEBO_RESOURCE_PATH

DRONE_SWARM_MEMBERS=$1

if [ -z $DRONE_SWARM_MEMBERS ] # Check if NUMID_DRONE is NULL
  then
  	#Argument 1 empty
    	echo "-Setting Swarm Members = 1"
    	DRONE_SWARM_MEMBERS=1
  else
    	echo "-Setting DroneSwarm Members = $1"
fi

for (( c=1; c<=$DRONE_SWARM_MEMBERS; c++ ))
do 
gnome-terminal  \
  	--tab --title "DroneRotorsSimulator" --command "bash -c \"
roslaunch colworld colwo.launch world_name:=$(realpath --relative-to=$(rospack find rotors_gazebo)/worlds/ $(rospack find colworld)/../Worlds)/colwo models_path:=$(rospack find colworld)/../models;
exec bash\""  &
done

