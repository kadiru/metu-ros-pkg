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

rosbuild_find_ros_package (al_utils)
rosbuild_find_ros_package (al_acts)

include_directories(${al_utils_PACKAGE_PATH}/include ${al_acts_PACKAGE_PATH}/include)
rosbuild_add_executable(text_to_speech text_to_speech.cpp)
find_library(AL_UTILS al_utils ${al_utils_PACKAGE_PATH}/lib)
target_link_libraries(text_to_speech ${AL_UTILS})
