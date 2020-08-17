#!/bin/bash

NUMID_DRONE=111
DRONE_SWARM_ID=1
export AEROSTACK_PROJECT=${AEROSTACK_COPY}/src/teleoperation

. ${AEROSTACK_COPY}/setup.sh

#---------------------------------------------------------------------------------------------
# INTERNAL PROCESSES
#---------------------------------------------------------------------------------------------
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# Rotors Interface                                                                            ` \
`#---------------------------------------------------------------------------------------------` \
  --tab --title "Rotors Interface" --command "bash -c \"
roslaunch rotors_interface rotors_interface.launch --wait \
    drone_namespace:=drone$NUMID_DRONE \
    rotors_drone_id:=$DRONE_SWARM_ID \
    rotors_drone_model:=hummingbird \
    my_stack_directory:=${AEROSTACK_PROJECT};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Midlevel Controller                                                                         ` \
`#---------------------------------------------------------------------------------------------` \
  --tab --title "Midlevel Controller" --command "bash -c \"
roslaunch drone_mid_level_autopilot_process droneMidLevelAutopilotROSModule.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id_int:=$NUMID_DRONE \
    estimated_speeds_topic_name:=gazebo_estimated_speed \
    estimated_pose_topic_name:=gazebo_estimated_pose \
    my_stack_directory:=${AEROSTACK_PROJECT};
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Quadrotor controller                                                                       ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Quadrotor Controller" --command "bash -c \"
roslaunch quadrotor_pid_controller_process quadrotor_pid_controller_process.launch --wait \
    robot_namespace:=drone$NUMID_DRONE \
    robot_config_path:=${AEROSTACK_PROJECT}/configs/drone$NUMID_DRONE
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Process Manager                                                                             ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Process Manager" --command "bash -c \"
roslaunch process_manager_process process_manager_process.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_PROJECT};
exec bash\""  &
#---------------------------------------------------------------------------------------------
# SHELL INTERFACE
#---------------------------------------------------------------------------------------------
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# alphanumeric_viewer                                                                         ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "alphanumeric_viewer"  --command "bash -c \"
roslaunch alphanumeric_viewer alphanumeric_viewer.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    my_stack_directory:=${AEROSTACK_PROJECT};
exec bash\""  &
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# keyboard_teleoperation_interface                                                            ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "keyboard_teleoperation_interface"  --command "bash -c \"
roslaunch keyboard_teleoperation_interface keyboard_teleoperation_interface.launch --wait \
  drone_id_namespace:=drone$NUMID_DRONE \
  my_stack_directory:=${AEROSTACK_COPY}/src/teleoperation;
exec bash\""  &