# Save everything but the images from the camera
gnome-terminal --tab -e "roslaunch active_debris_control gazebo.launch"
sleep 15 && rosbag record -a -o datalog.bag -x "(.*)/image_raw/(.*)"
#EOF#
