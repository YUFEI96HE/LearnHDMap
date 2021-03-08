#!/bin/bash
echo 'transfer video'
gnome-terminal -x bash -c "source /home/nvidia/sweeper/catkin_ws/devel/setup.bash && bash /home/nvidia/sweeper/catkin_ws/src/AutoSweeper/modules/communication/communication/launch/transfer_video_front.sh $1; exec bash ;" &
