#!/bin/bash

# source ros environment variables
source ~/repos/ros_noetic/install_isolated/setup.bash
# import source of required dependencies, with compatible branches
vcs import --recursive --input dependencies.repos --repos src
# install binary dependencies using system's package manager(dnf/apt)
# rosdep install -y -r -q --from-paths src --ignore-src --rosdistro noetic
# dnf install octomap-devel