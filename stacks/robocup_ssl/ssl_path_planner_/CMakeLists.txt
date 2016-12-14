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
set(OMPL_HOME /home/kadir/interconnection/workspace/packages/omplapp)
add_definitions(-I${OMPL_HOME}/ompl/src -DLINUX -g -O0 -Wall -fmessage-length=0 -frounding-math)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_executable(ssl_path_planner src/main.cpp src/SSLPathPlanner.cpp)
find_library(OMPL_LIBRARY ompl ${OMPL_HOME}/build/Release/lib)
find_library(CGAL_LIBRARY CGAL /usr/local/lib)
target_link_libraries(ssl_path_planner ${OMPL_LIBRARY} ${CGAL_LIBRARY})
