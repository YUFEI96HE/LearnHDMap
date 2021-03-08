#!/bin/bash

#APP急停
source /home/nvidia/sweeper/catkin_ws/devel/setup.bash
source /opt/ros/melodic/setup.bash

rosnode kill drive_ethernet_node
