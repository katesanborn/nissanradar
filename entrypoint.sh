#!/bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
exec roslaunch nissanradar radar_viz.launch "$@"