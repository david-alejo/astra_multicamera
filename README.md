# openni2-multicamera

This package includes several utilities that make it easier to work with an array of cameras compatible with openni2. 

In the current states there are two different utilities

- generate_launch_file --> generates delayed launch file for opening more than one camera. It should be used in an empty folder. As input file, the openni2.launch file from the openni2_launch package can be used. Use (please make an empty call rosrun openni2_multicamera generate_launch_file for more info): 

rosrun openni2_multicamera generate_launch_file \<output_file> \<input_file> [ \<data_skip> [\<delay_time>]]

- show_info --> shows relevant info of all devices connected to the computer. Usage:

rosrun openni2_multicamera show_info

# Install

It assumes that ROS has been installed (tested with indigo and kinetic). Then, download the dependencies (should not be required, as these are also ROS dependencies) and download the code into a catkin workspace.

# Dependencies:

- tinyxml
- boost (shared_ptr)

