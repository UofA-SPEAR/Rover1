# SPEAR ROVER 1 
This directory is the main catkin package for the rover. This README will describe the important files, possible errors, and other info. 

# Project Heirarchy
rover1/msg/ contains all custom .msg files used by ROS. More info on .msg files can be found on the ROS wiki

rover1/src/ contains all src files for the project. This includes the code for each node on the Rover, and the main launch file.

rover1/src/launch/ contains the launch file used for the project. The launch file is xml code that allows us to run all of the ROS nodes at the same time. More info can found below in "Running in the Project"

# Building the Project
When using this project, it should be placed within the src folder of a catkin workspace. To build the project, navigate to the main directory of the catkin workspace and run: `catkin-make`. This will build the projects located in the catkin workspace.

# Running the Project
To run the project we need to run two things: the main ROS rover project and ps3joy. Ps3joy is a python script that creates a ROS node to connect to a ps3 controller (more info can be found here: http://wiki.ros.org/ps3joy)

Therefore to run the project first navigate to the catkin workspace and start the ps3joy node and follow the prompts to connect the ps3 controller:
`rosrun ps3joy ps3joy.py`

Next, in a seperate terminal navigate to the catkin workspace and start the main project:
`roslaunch rover1 rover1.launch1`


