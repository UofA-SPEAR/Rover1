# SPEAR ROVER 1 
This directory is the main catkin package for the rover. This README will describe the important files, possible errors, and other info. 

# Dependencies:
To run the project we need:
- ps3joy (http://wiki.ros.org/ps3joy)
- serial library (catkin package - https://github.com/wjwwood/serial)

# Project Heirarchy
rover1/msg/ contains all custom .msg files used by ROS. More info on .msg files can be found on the ROS wiki

rover1/src/ contains all src files for the project. This includes the code for each node on the Rover, and the main launch file.

rover1/src/launch/ contains the launch file used for the project. The launch file is xml code that allows us to run all of the ROS nodes at the same time. More info can found below in "Running in the Project"

# Building the Project
When using this project, it should be placed within the src folder of a catkin workspace. To build the project, navigate to the main directory of the catkin workspace and run: `catkin_make`. This will build the projects located in the catkin workspace.

# Running the Project
To run the project we need to run two things: the main ROS rover project and ps3joy. Ps3joy is a python script that creates a ROS node to connect to a ps3 controller

Therefore to run the project first navigate to the catkin workspace and start the ps3joy node and follow the prompts to connect the ps3 controller:
`rosrun ps3joy ps3joy.py`

Next, in a seperate terminal navigate to the catkin workspace and start the main project:
`roslaunch rover1 rover1.launch1`

# Common Errors when Running:
"Unable to contact my own server at ..." : To run ROS we have static IPS setup on both the Raspberry Pi and the Nvidia TX2 under the "Static IP" network connection. Therefore to fix this error, make sure that you are connected to the "Static IP" network on the Jetson. 

"NO ARDUINO CONNECTED, PLEASE RESTART THE PROGRAM WITH ARDUINO CONNECTED TO THE USB HUB." : The program is looking for an arduino on the /dev/ttyASM0 port and cannot find it. Therefore, restart the ROS program and make sure the Arduino is connected. 

"Couldn't open joystick /dv/input/js0. Will retry every second.": This error means that the program cannot find the ps3 controller. Make sure that you have run the ps3joy script before the main rover program. 

Generally unable to read any data from the USB port: We need to check that the Arduino connection was properly initialized. Run 'dmesg' in the terminal and reconnect the arduino. If "failed to set dtr/rts" appears, then check that the Arduino is connected to the main USB port on the Jetson and not the micro USB port. 



