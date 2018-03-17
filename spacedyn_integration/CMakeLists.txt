cmake_minimum_required(VERSION 2.8.3)
project(spacedyn_integration)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  spacedyn_ros
  cmake_modules
  roslib
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES targetspd chaserspd
CATKIN_DEPENDS  spacedyn_ros roscpp rospy gazebo_ros std_msgs roslib
DEPENDS
		cmake_modules
)

find_package(GSL REQUIRED)
find_package(gazebo REQUIRED)
include_directories(include
					${Boost_INCLUDE_DIR}
					${catkin_INCLUDE_DIRS}
					${GSL_INCLUDE_DIR} # GNU Scientific Library required for SpaceDyn Libraries
					${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set ( CHASER_SRC
	src/chaser_load.cpp
	src/chaser_mapper.cpp
	src/chaser_controller.cpp
	src/dataflag.cpp
	src/impulse_controller.cpp
	#src/timer.cpp
	src/dataext.cpp
)

add_library(chaserspd SHARED ${CHASER_SRC}) 

target_link_libraries(chaserspd ${catkin_LIBRARIES}
							 ${GSL_LIBRARIES}
							 ${GAZEBO_LIBRARIES}
							 ${Boost_LIBRARIES})

set ( TARGET_SRC
	src/target_load.cpp
	src/target_mapper.cpp
	src/dataflag.cpp
	#src/timer.cpp
	src/dataext.cpp
	src/target_parser.cpp
	src/impulse_controller.cpp
)

add_library(targetspd SHARED ${TARGET_SRC}) 

target_link_libraries(targetspd ${catkin_LIBRARIES}
							 ${GSL_LIBRARIES}
							 ${GAZEBO_LIBRARIES}
							 ${Boost_LIBRARIES})



#New from gazebo6 : to compile c++11 flags against gazebo (required)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
