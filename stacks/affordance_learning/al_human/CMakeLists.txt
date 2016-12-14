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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_library(human_controller src/human_controller.cpp)

rosbuild_find_ros_package (al_utils)
include_directories(${al_utils_PACKAGE_PATH}/include)

rosbuild_add_executable(human_tf_publisher human_tf_publisher.cpp)
#rosbuild_add_executable(gazebo_st_publisher gazebo_st_publisher.cpp)
rosbuild_add_executable(test_ src/test.cpp)
rosbuild_add_executable(human_state_pub src/human_state_pub.cpp)
rosbuild_add_executable(camera_state_pub src/camera_state_pub.cpp)
rosbuild_add_executable(wiimote_state_pub src/wiimote_state_pub.cpp)
find_library(AL_UTILS al_utils ${al_utils_PACKAGE_PATH}/lib)
target_link_libraries(test_ ${AL_UTILS})
target_link_libraries(human_state_pub ${AL_UTILS})
target_link_libraries(wiimote_state_pub ${AL_UTILS})
