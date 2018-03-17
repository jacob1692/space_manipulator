cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 2.8.3)
project(spacedyn_ros)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
)

find_package(GSL REQUIRED)

catkin_package(CATKIN_DEPENDS cmake_modules roscpp rospy std_msgs
               DEPENDS 
               INCLUDE_DIRS include
               LIBRARIES libspd)

include_directories( include
  include/${PROJECT_NAME}/matrix
  include/${PROJECT_NAME}/spd
  ${catkin_INCLUDE_DIRS}
  ${GSL_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

#-----------------------------------SPD PACKAGE MAKE CONDIGURATION -----------------
set (SPDLIB_SOURCES
	src/spd/model.cpp
	src/spd/model_init.cpp
	src/spd/calc_SP.cpp
	src/spd/calc_SPN.cpp
	src/spd/calc_hh.cpp
	src/spd/calc_Rg.cpp
	src/spd/calc_Je.cpp
	src/spd/calc_Jb.cpp
	src/spd/calc_GJe.cpp
	src/spd/calc_Lg.cpp
	src/spd/calc_JJ.cpp
	src/spd/calc_SS.cpp
	src/spd/calc_Xup_I.cpp
	src/spd/j_num.cpp
	src/spd/f_kin_e.cpp
	src/spd/f_kin_j.cpp
	src/spd/i_dyn.cpp
	src/spd/i_dyn_fix.cpp
	src/spd/f_dyn.cpp
	src/spd/f_dyn_fix.cpp
	src/spd/inner_force.cpp
	src/matrix/matrix.cpp
	src/matrix/vector.cpp
	src/spn/spn.cpp
	src/spn/rot.cpp
	src/spd/rref.cpp
	src/spd/aw
	src/spd/w2dQtn.cpp
	src/spd/f_dyn_rk.cpp
	src/spd/model_test.cpp
	src/spd/calc_GJb.cpp
	src/spd/calc_GJe_r.cpp
	src/spd/calc_GH.cpp
	src/spd/calc_Gc.cpp
	src/spd/calc_dxgh.cpp
	src/spd/calc_C.cpp
#	src/spd/calc_tdxhr.cpp remember to change the spd_m.h
	src/spd/calc_tdxghI.cpp
	)
add_library(libspd ${SPDLIB_SOURCES})

target_link_libraries( libspd
		${GSL_LIBRARIES} 
		${catkin_LIBRARIES}
		${Boost_LIBRARIRES}
)

#---------------installing the libraries -----------------------------

install(TARGETS libspd
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})        
        
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
