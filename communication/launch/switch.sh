#!/bin/bash

#APP关机脚本
source /home/nvidia/sweeper/catkin_ws/devel/setup.bash
source /opt/ros/melodic/setup.bash

rosnode kill --all && sleep 5 && echo 'nvidia' |sudo -S poweroff
