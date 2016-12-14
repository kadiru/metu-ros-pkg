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

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(Curses REQUIRED)
add_definitions(-lncurses)
include_directories(${CURSES_INCLUDE_DIR})

#rosbuild_find_ros_package (arff)
#include_directories(${arff_PACKAGE_PATH}/src)

#SET(CMAKE_CXX_FLAGS "-Wall -g -fPIC")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/al_utils.cpp)
#find_library(ARFF arff ${arff_PACKAGE_PATH})
#target_link_libraries(${PROJECT_NAME} ${ARFF})

#rosbuild_add_executable(collision_space_viz src/collision_space_viz.cpp src/al_utils.cpp)
#rosbuild_add_executable(request_object src/request_object.cpp src/al_utils.cpp)
#rosbuild_add_executable(request_behavior src/request_behavior.cpp src/al_utils.cpp)
#target_link_libraries(request_behavior ${CURSES_LIBRARY})
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
