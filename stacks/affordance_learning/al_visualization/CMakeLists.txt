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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_find_ros_package (al_utils)
include_directories(${al_utils_PACKAGE_PATH}/include /usr/local/MATLAB/R2012a/extern/include)
#include_directories(${al_utils_PACKAGE_PATH}/include)

#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules(MATLAB_ROOT REQUIRED Matlab)
#include_directories(${MATLAB_INCLUDE_DIRS})
#link_directories(${MATLAB_LIBRARY_DIRS})

#rosbuild_add_executable(viz_features src/viz_features.cpp)
#rosbuild_add_executable(viz_objects_boxes src/viz_objects_boxes.cpp)
rosbuild_add_executable(viz_object_features src/viz_object_features.cpp)

find_library(AL_UTILS al_utils ${al_utils_PACKAGE_PATH}/lib)
#target_link_libraries(viz_features ${AL_UTILS} )
target_link_libraries(viz_object_features ${AL_UTILS} )
#target_link_libraries(viz_objects_boxes ${AL_UTILS} /usr/local/MATLAB/R2012a/bin/glnxa64/libeng.so /usr/local/MATLAB/R2012a/bin/glnxa64/libmex.so /usr/local/MATLAB/R2012a/bin/glnxa64/libmx.so)

#target_link_libraries(viz_objects_boxes ${AL_UTILS} /usr/local/MATLAB/R2012a/bin/glnxa64/libeng.so /usr/local/MATLAB/R2012a/bin/glnxa64/libmex.so /usr/local/MATLAB/R2012a/bin/glnxa64/libmx.so)

#find_package(Matlab REQUIRED)
#TODO: Add more genious path variables around
#target_link_libraries(  al_perception_perceptors 
#                        /usr/local/MATLAB/R2011a/bin/glnxa64/libeng.so 
#                        /usr/local/MATLAB/R2011a/bin/glnxa64/libmex.so 
#                        /usr/local/MATLAB/R2011a/bin/glnxa64/libmx.so)