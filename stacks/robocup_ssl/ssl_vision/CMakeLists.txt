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
set(WEBOTS_HOME $ENV{WEBOTS_HOME})
add_definitions(-I${WEBOTS_HOME}/include/controller/cpp -I${WEBOTS_HOME}/include -Wall -I${WEBOTS_HOME}/include/controller/c -DLINUX)

rosbuild_find_ros_package (ssl_ext_utils)
set(SSLVISION_HOME ${ssl_ext_utils_PACKAGE_PATH}/ssl-vision)
find_library(SSLVISION_LIBRARY sslvision ${SSLVISION_HOME}/build)
add_definitions(-I${SSLVISION_HOME}/build -I${SSLVISION_HOME}/src/shared/net)
message( STATUS "SSLVISION_LIBRARY: ${SSLVISION_LIBRARY}")

find_package ( Qt4 REQUIRED )
include ( ${QT_USE_FILE} )
set(SSLVISION_LIBS ${QT_LIBRARIES} dc1394 jpeg png protobuf pthread GL GLU ${SSLVISION_LIBRARY})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package (ssl_utils)
include_directories(${ssl_utils_PACKAGE_PATH}/include)

rosbuild_add_executable(${PROJECT_NAME} src/main.cpp 
                                        src/SSLVision.cpp 
                                        src/SSLWSimVision.cpp 
                                        src/SSLRealVision.cpp)

find_library(SSL_UTILS ssl_utils ${ssl_utils_PACKAGE_PATH}/lib)
find_library(CONTROLLER_LIBRARY CppController ${WEBOTS_HOME}/lib)
target_link_libraries(ssl_vision ${SSL_UTILS} ${CONTROLLER_LIBRARY} ${SSLVISION_LIBS})
