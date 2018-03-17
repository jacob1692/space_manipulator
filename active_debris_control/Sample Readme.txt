1. Make sure that Gazebo and ROS are properlly installed.

2. Create a catkin_ws ( catkin workspace ) by executing
        $ source /opt/ros/kinetic/setup.sh
	$ mkdir -p ~/catkin_ws
	$ catkin_init_workspace
	$ cd ~/catkin_ws/
	$ catkin_make
3. Overlay the setup.sh of the new workspace to the environment
        $ source devel/setup.sh

4. Make sure that the $ROS_PACKAGE_PATH is correct

	$echo $ROS_PACKAGE_PATH
	/home/youruser/catkin_ws/src:/opt/ros/kinetic/share:/opt/ros/kinetic/stacks

5. Copy the space robot folder to ~/catkin_ws/src/

6. Prepare the environment for your new space_robot package using /devel/setup.sh
	$ . ~/catkin_ws/src/space_robot/devel/setup.sh
7. Copy the following entries at the end of your ~/.bashrc file

	source /opt/ros/kinetic/setup.bash
	source /opt/ros/kinetic/setup.sh
	source /usr/share/gazebo/setup.sh
	source /usr/share/gazebo-7/setup.sh
	export EDITOR='nano -w'
	#export ROS_HOSTNAME=localhost
	#export ROS_MASTER_URI=http://localhost:11311
	export GAZEBO_MODEL_PATH=~/catkin_ws/src/space_robot_vis/models:~/catkin_ws/src/target_description/models
	#export ROS_HOME=~/.ros
	export ROS_PACKAGE_PATH=/home/jacob/catkin_ws/src/chaser_moveit_config:/home/jacob/catkin_ws/src:/opt/ros/kinetic/share
	export GAZEBO_PLUGIN_PATH=~/catkin_ws/src/tohoku_space_manipulator/chaser_control/build/devel/lib:$GAZEBO_PLUGIN_PATH
	export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/src/tohoku_space_manipulator/chaser_spacedyn/build/devel/lib
	. ~/catkin_ws/devel/setup.bash
	#remember to install GSL library
	#remember to install ros control

8. 	Install PyQt Graph from www.pyqtgraph.org

9.  Replace using administrative rights file pyqtgraph_data_plot.py from active_debris_control/etc/ to /opt/ros/kinetic/lib/python2.7/dist-packages/rqt_plot/data_plot/

10. Install  ignition library : ignitionrobotics.org/libraries/math

11. Install move it: sudo-apt-get install ros-kinetic-moveit


	
--------------------------To execute the visualization----------------------

a. Execute the initialization file
	$ roscd space_robot/devel
	$ . init.sh

b. Follow the instructions in the command prompt.

c. If you want to execute again the visualization type 
	$ . ~/catkin_ws/src/space_robot/devel/pub.sh
	
----

sudo cp SkyX.material `rospack find space_robot`/worlds /usr/share/gazebo7/media/skyx



add to ~/.bashrc

source /opt/ros/kinetic/setup.bash
source /opt/ros/kinetic/setup.sh
source /usr/share/gazebo-7/setup.sh
export EDITOR='nano -w'
#export ROS_HOSTNAME=localhost
#export ROS_MASTER_URI=http://localhost:11311
export GAZEBO_MODEL_PATH=~/catkin_ws/src/space_robot_vis/models:~/catkin_ws/src$
#export ROS_HOME=~/.ros
export ROS_PACKAGE_PATH=/home/jacob/catkin_ws/src/chaser_moveit_config:/home/ja$
export GAZEBO_PLUGIN_PATH=~/catkin_ws/src/chaser_control/build/devel/lib:$GAZEB$



