cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

FIND_PACKAGE( YARP REQUIRED )
#FIND_PACKAGE( ICUB REQUIRED )
#INCLUDE_DIRECTORIES( ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${YARP_INCLUDE_DIRS} )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#if(YARP_FOUND AND ICUB_FOUND)
if(YARP_FOUND)
    INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
    rosbuild_add_executable(hri_main hri_main.cpp)
    target_link_libraries(hri_main icubmod)
endif(statement)
