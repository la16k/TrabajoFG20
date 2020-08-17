#!/bin/bash

DRONE_SWARM_MEMBERS=$1

if [ -z $DRONE_SWARM_MEMBERS ] # Check if NUMID_DRONE is NULL
  then
  	#Argument 1 empty
    	echo "-Setting Swarm Members = 1"
    	DRONE_SWARM_MEMBERS=1
  else
    	echo "-Setting DroneSwarm Members = $1"
fi

gnome-terminal  \
   	--tab --title "DroneRotorsSimulator" --command "bash -c \"
roslaunch aerostack_copy env_mav.launch;
						exec bash\""  &

for (( c=1; c<=$DRONE_SWARM_MEMBERS; c++ ))
do  
gnome-terminal  \
   	--tab --title "DroneRotorsSimulator" --command "bash -c \"
roslaunch aerostack_copy mav_swarm.launch --wait drone_swarm_number:=$c mav_name:=hummingbird;
						exec bash\""  &
done