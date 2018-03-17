cmake_minimum_required(VERSION 2.8.3)
project(chaser_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  controller_manager_msgs
  controller_manager_tests
  gazebo_ros
  controller_interface
  hardware_interface
  joint_limits_interface
  transmission_interface
  realtime_tools 
  roscpp
  rospy
  std_msgs
  spacedyn_integration
  spacedyn_ros
)


catkin_package(
INCLUDE_DIRS include
LIBRARIES compliant_wrist gazebo_ros_3D_thruster gazebo_ros_bumper_flag gazebo_joints_pid
CATKIN_DEPENDS 
		 spacedyn_integration spacedyn_ros controller_manager controller_manager_msgs realtime_tools transmission_interface hardware_interface controller_manager_tests  joint_limits_interface gazebo_ros hardware_interface roscpp rospy std_msgs
DEPENDS
		roscpp
		rospy
		gazebo_ros
		std_msgs
)


find_package(gazebo REQUIRED)

include_directories(include 
					${Boost_INCLUDE_DIR}
					${catkin_INCLUDE_DIRS}
					${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(compliant_wrist SHARED 
			src/compliant_wrist.cc
			)

add_library(gazebo_ros_3D_thruster SHARED 
			src/gazebo_ros_3D_thruster.cc
			src/pid_controller.cpp
			)
			
add_library(gazebo_ros_bumper_flag SHARED 
			src/gazebo_ros_bumper_flag.cpp
			)
			
add_library(gazebo_ros_f3d_rel SHARED 
			src/gazebo_ros_f3d.cpp
			)
			
add_library(gazebo_joints_pid SHARED 
			src/gazebo_joints_PID.cpp
			)
			
target_link_libraries(compliant_wrist
					  ${catkin_LIBRARIES}
					  ${GAZEBO_LIBRARIES}
					  ${Boost_LIBRARIES})
					  
target_link_libraries(gazebo_ros_3D_thruster
					  ${catkin_LIBRARIES}
					  ${GAZEBO_LIBRARIES}
					  ${Boost_LIBRARIES})

target_link_libraries(gazebo_ros_bumper_flag
					  ${catkin_LIBRARIES}
					  ${GAZEBO_LIBRARIES}
					  ${Boost_LIBRARIES})
					  
target_link_libraries(gazebo_joints_pid
					  ${catkin_LIBRARIES}
					  ${GAZEBO_LIBRARIES}
					  ${Boost_LIBRARIES})
					  
target_link_libraries(gazebo_ros_f3d_rel
					  ${catkin_LIBRARIES}
					  ${GAZEBO_LIBRARIES}
					  ${Boost_LIBRARIES})

#New from gazebo6 : to compile c++11 flags against gazebo (required)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
