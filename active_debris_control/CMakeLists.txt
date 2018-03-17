cmake_minimum_required(VERSION 2.8.3)
project(active_debris_control)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros	
	roscpp 
	rospy 
	std_msgs
	message_generation 	
)

find_package(gazebo REQUIRED)

find_package(message_generation)


add_message_files(
	DIRECTORY msg
	FILES
	StampedDepths.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	gazebo_msgs
)


include_directories(${Boost_INCLUDE_DIR}
					${catkin_INCLUDE_DIRS}
					${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
	DEPENDS
		roscpp
		rospy
		gazebo_ros
		message_runtime
)

#add_dependencies(StampedDepths ${catkin_EXPORTED_TARGETS})

#New from gazebo6 : to compile c++11 flags against gazebo (required)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

