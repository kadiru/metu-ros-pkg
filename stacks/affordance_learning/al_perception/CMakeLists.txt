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

#rosbuild_genmsg()
#rosbuild_gensrv()

rosbuild_find_ros_package (al_utils)
#TODO: Add more genious path variables around
#include_directories(${al_utils_PACKAGE_PATH}/include /usr/local/MATLAB/R2011a/extern/include)
include_directories(${al_utils_PACKAGE_PATH}/include)

#rosbuild_add_library(   al_perception_perceptors
#                        src/perceptors/Visual3D.cpp 
#                        src/perceptors/Perceptor.cpp
#                        src/feature_extractors/HumanFExtractor.cpp
#                        src/feature_extractors/SurfaceFExtractor.cpp
#                        src/feature_extractors/SpatialFExtractor.cpp
#                        src/feature_extractors/FeatureExtractor.cpp
#                        src/objects/Entity.cpp)
                        
#find_package(Matlab REQUIRED)
#TODO: Add more genious path variables around
#target_link_libraries(  al_perception_perceptors 
#                        /usr/local/MATLAB/R2011a/bin/glnxa64/libeng.so 
#                        /usr/local/MATLAB/R2011a/bin/glnxa64/libmex.so 
#                        /usr/local/MATLAB/R2011a/bin/glnxa64/libmx.so)

#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules(MATLAB_ROOT REQUIRED Matlab)
#include_directories(${MATLAB_INCLUDE_DIRS})
#link_directories(${MATLAB_LIBRARY_DIRS})
#target_link_libraries(al_perception_perceptors ${MATLAB_LIBRARIES})

rosbuild_add_executable(   ${PROJECT_NAME}
                        src/main.cpp
                        src/perception/Perception.cpp
                        src/perceptors/Visual3D.cpp 
                        src/perceptors/Perceptor.cpp
                        src/feature_extractors/HumanFExtractor.cpp
                        src/feature_extractors/SurfaceFExtractor.cpp
                        src/feature_extractors/SpatialFExtractor.cpp
                        src/feature_extractors/FeatureExtractor.cpp
                        src/objects/Entity.cpp)
                        
find_library(AL_UTILS al_utils ${al_utils_PACKAGE_PATH}/lib)
#target_link_libraries(al_perception_perceptors ${AL_UTILS})

#rosbuild_add_library(   al_perception_perception
#                        src/perception/Perception.cpp )
target_link_libraries(${PROJECT_NAME} ${AL_UTILS})                        
#target_link_libraries(al_perception_perception al_perception_perceptors)
#rosbuild_add_executable(${PROJECT_NAME} src/main.cpp src/perception/Perception.cpp)
#target_link_libraries(${PROJECT_NAME} al_perception_perception)


