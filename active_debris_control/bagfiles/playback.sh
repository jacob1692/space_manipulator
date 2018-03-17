gnome-terminal --tab -e "roscore"
sleep 10 && rosparam set /use_sim_time "true"
rosrun rqt_bag rqt_bag
#EOF#
