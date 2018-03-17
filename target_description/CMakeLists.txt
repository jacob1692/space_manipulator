cmake_minimum_required(VERSION 2.8.3)
project(target_description)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  spacedyn_integration
  spacedyn_ros
)

find_package(gazebo REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES targetm alwaysenable
CATKIN_DEPENDS
	gazebo_ros roscpp rospy std_msgs spacedyn_integration
	spacedyn_ros
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)


add_library(alwaysenable SHARED lib/alwaysenable.cc)
link_directories(${GAZEBO_LIBRARY_DIRS})
target_link_libraries(alwaysenable ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


add_library(targetm SHARED
			src/gazebo_target_m.cpp)
			
target_link_libraries(targetm
					  ${catkin_LIBRARIES}
					  ${GAZEBO_LIBRARIES}
					  ${Boost_LIBRARIES})			


#c++11 flags compiled against gazebo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
