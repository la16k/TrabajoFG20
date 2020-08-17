##  Teleoperation with Rotors simulator

In order to install and execute this project, perform the following steps:

- Requirements: Linux Ubuntu 18.04 with ROS Melodic or Linux Ubuntu 16.04 with ROS Kinetic.

- Download the installation files:

        $ git clone https://bitbucket.org/visionaerialrobotics/aerostack_installers.git ~/temp

- Run the following installation script to install the project "teleoperation_pid_rotors_simulator":

        $ ~/temp/install_project_from_source.sh projects_cvar/teleoperation_pid_rotors_simulator

- Change directory to this project:

        $ cd $AEROSTACK_STACK/projects_cvar/teleoperation_pid_rotors_simulator

- Execute the script that launches Gazebo:

        $ ./launcher_gazebo.sh

- Wait until the following window is presented:

![capture-gazebo.png](https://bitbucket.org/repo/rokr9B/images/916057309-capture-gazebo.png)

- Open a new terminal and change directory to the project:

        $ cd $AEROSTACK_STACK/projects_cvar/teleoperation_pid_rotors_simulator

- Execute the script that launches the Aerostack components for this project:
 
        $ ./main_launcher.sh

- The following windows for teleoperation are presented:

![Alphanumeric Viewer](https://i.ibb.co/vLhJpbn/alphanumeric.png)

![Ground speed teleoperation control mode](https://i.ibb.co/m5ngjvQ/keyboardground.png)

- Click on the window corresponding to the keyboard teleoperation interface and teleoperate the drone using the appropriate keys.

- To stop the processes execute the following script:

        $ ./stop.sh

- To close the inactive terminals:

        $ killall bash
