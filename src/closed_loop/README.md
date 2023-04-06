# closed_loop node

This node implements a closed loop controller for the Jiwy simulator of assignment 1 for ASDfR.

# Usage 
* Unzip into your ROS2 workspace
* `colcon build`
* Then, in another terminal:
* `. install\local_setup.sh`
* `ros2 run closed_loop controller`

# Hints
* Have a look in the code to see the topic names, available parameters etc. You might want to change the code so that it matches your choices on e.g. the image size from your webcam.
* Do not forget to set the appropriate topic remappings in the latter command.
* We recommend to use a launch file for launching all ROS2 nodes simultaneously.