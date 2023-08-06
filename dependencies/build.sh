#!/bin/bash

# build the custom ROS2 messages
colcon build --executor sequential

# source the setup script to make the messages available to other ROS2 packages
source install/setup.bash

# add source command to bashrc to make the messages available to all future terminal sessions
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc

cd ~
source ~/.bashrc
