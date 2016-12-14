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

find_package(YARP REQUIRED)
rosbuild_find_ros_package (al_utils)
include_directories(${al_utils_PACKAGE_PATH}/include)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_executable(sr4k_tf_pub src/sr4k_tf_pub.cpp)
rosbuild_add_executable(kinect_tf_pub src/kinect_tf_pub.cpp)
rosbuild_add_executable(kinect_tf_cal_pub src/kinect_tf_cal_pub.cpp)
rosbuild_add_executable(multi_kinect_tf_pub src/multi_kinect_tf_pub.cpp)
rosbuild_add_executable(hat_tf_pub src/hat_tf_pub.cpp)
rosbuild_add_executable(hat_tf_pub_w_calibration src/hat_tf_pub_w_calibration.cpp)
rosbuild_add_executable(rand_torso_hat_tf_gen src/rand_torso_hat_tf_gen.cpp)
target_link_libraries(sr4k_tf_pub ${YARP_LIBRARIES})
target_link_libraries(kinect_tf_pub ${YARP_LIBRARIES})
target_link_libraries(kinect_tf_cal_pub ${YARP_LIBRARIES})
target_link_libraries(multi_kinect_tf_pub ${YARP_LIBRARIES})
target_link_libraries(hat_tf_pub ${YARP_LIBRARIES})
target_link_libraries(hat_tf_pub_w_calibration ${YARP_LIBRARIES})

find_library(AL_UTILS al_utils ${al_utils_PACKAGE_PATH}/lib)
target_link_libraries(rand_torso_hat_tf_gen ${AL_UTILS})
