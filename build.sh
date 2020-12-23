#!/bin/bash

## source ros environment variables
source ~/repos/ros_noetic/install_isolated/setup.bash

## we use colcon build system as this is default for ros2 and future plan is to port to ros2
## usefull options:
## --packages-up-to <pkg>: build <pkg> and all its deps
## --packages-select <pkg>: build <pkg> only, skipping deps
colcon build \
--cmake-args "-Wno-dev -Wno-deprecated -DCMAKE_BUILD_TYPE=Debug -DCMAKE_MESSAGE_LOG_LEVEL=STATUS" \
--packages-select pigeon_sim #rotors_gazebo_plugins
# --packages-up-to 
# --symlink-install \
